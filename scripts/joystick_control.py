#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import serial
import threading
import os
import math
import json
import numpy as np
# pygame импортируется внутри функции для обработки ошибок
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import Trigger, TriggerResponse

# Матрица преобразования осей (как в imu_visualizer.py)
AXIS_TRANSFORM_MATRIX = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, -1.0, 0.0],
])

def transform_axes_vector(vec):
    """Преобразование вектора согласно матрице преобразования осей"""
    return AXIS_TRANSFORM_MATRIX.dot(vec)

# --- Constants and Mappings ---
AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', 'x_button', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState:
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

# --- Serial Reader Thread ---
class SerialReader(threading.Thread):
    def __init__(self, ser_getter, hit_detection_pub_instance, robot_id):
        super().__init__()
        self.ser_getter = ser_getter  # Function to get serial instance (handles None)
        self.hit_detection_pub = hit_detection_pub_instance
        self.robot_id = robot_id
        self.running = True
        self.consecutive_errors = 0
        self.max_consecutive_errors = 10

    def run(self):
        rospy.loginfo("Serial port reading thread started.")
        while self.running and not rospy.is_shutdown():
            try:
                ser = self.ser_getter()
                if ser is None or not ser.is_open:
                    # Serial port not available, wait and retry
                    time.sleep(1.0)
                    continue
                
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        rospy.loginfo(f"Received from Arduino: '{line}'")
                        if line == "HIT":
                            # Publish hit detection to game server with JSON format
                            hit_data = {
                                'robot_id': self.robot_id,
                                'timestamp': rospy.get_time(),
                                'hit_detected': True
                            }
                            hit_msg = String()
                            hit_msg.data = json.dumps(hit_data)
                            self.hit_detection_pub.publish(hit_msg)
                            rospy.loginfo(f"Published hit detection to /game/hit_detection: {hit_msg.data}")
                        else:
                            rospy.logwarn(f"Received unknown response from Arduino: '{line}'")
                    self.consecutive_errors = 0  # Reset error counter on successful read
            except serial.SerialException as e:
                self.consecutive_errors += 1
                if self.consecutive_errors >= self.max_consecutive_errors:
                    rospy.logwarn(f"Serial port error in reading thread (attempt {self.consecutive_errors}): {e}")
                    rospy.logwarn("Serial port appears disconnected. Will retry when connection is restored.")
                    self.consecutive_errors = 0  # Reset to avoid spam
                time.sleep(1.0)  # Wait longer on error
            except AttributeError:
                # Serial port is None or not initialized
                time.sleep(1.0)
            except Exception as e:
                rospy.logwarn(f"Unexpected error in serial reading thread: {e}")
                time.sleep(0.5)
            else:
                time.sleep(0.01)  # Normal sleep when everything is OK
        rospy.loginfo("Serial port reading thread stopped.")

    def stop(self):
        self.running = False

# --- Main Controller Class ---
class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)

        # --- Hardware and ROS Initialization ---
        self.board = Board()
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager()
        
        # --- State Variables ---
        self.speed_mode = 1
        self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude = 0, 0, 0
        self.init_z_offset = 0.025
        self.time_stamp_ry = 0
        self.status = 'stop'
        self.update_param, self.update_height = False, False
        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP)))

        # --- Game State Variables ---
        self.max_hp = 100
        self.current_hp = 100
        self.is_locked = False  # Locked when HP = 0 or match not started (legacy)
        self.robot_id = rospy.get_param('~robot_id', 'robot_1')
        self.is_firing = False  # Track firing state
        
        # --- Control Permissions (Detailed Lock System) ---
        self.permissions = {
            'movement': False,  # Разрешено ли движение
            'head': True,       # Разрешено ли управление головой
            'firing': False,    # Разрешено ли стрельба
            'camera': True      # Разрешен ли доступ к камере
        }
        self.use_detailed_permissions = False  # Флаг использования детальных разрешений
        
        # --- Serial Port Configuration ---
        self.serial_port = None
        self.serial_port_name = rospy.get_param('~port', '/dev/ttyUSB0')
        self.serial_baudrate = rospy.get_param('~baudrate', 9600)
        self.serial_lock = threading.Lock()  # Lock for thread-safe serial access

        # --- NEW: State variables for Get-Up functionality (from joystick_control 1.py) ---
        self.robot_state = 'stand'
        self.lie_to_stand_action_name = 'lie_to_stand'
        self.recline_to_stand_action_name = 'recline_to_stand'
        self.count_lie = 0
        self.count_recline = 0
        self.FALL_COUNT_THRESHOLD = 50
        self.fall_check_cooldown = 0  # Задержка после подъема перед следующей проверкой падения

        # --- IMU-based gait adaptation variables ---
        # История IMU данных для расчета амплитуды
        self.imu_history_size = 50  # Количество последних измерений для анализа
        self.imu_history = {
            'ax': [],  # linear_acceleration.x
            'ay': [],  # linear_acceleration.y
            'az': [],  # linear_acceleration.z
            'roll': [],  # orientation roll (из quaternion)
            'pitch': [],  # orientation pitch
            'yaw': []   # orientation yaw
        }
        
        # Параметры для автоматических шагов в покое
        self.auto_balance_enabled = True
        # Используем ЛИНЕЙНЫЕ УСКОРЕНИЯ для балансировки
        # Y- = падение назад, Y+ = падение вперед
        # X+ = вправо, X- = влево
        
        self.accel_change_threshold = 0.2  # Порог ИЗМЕНЕНИЯ ускорения (м/с²) для балансировки - строгий порог
        self.accel_change_min_threshold = 0.1  # Минимальный порог изменения - игнорируем шум
        self.auto_step_amplitude_base = 0.008  # Базовая амплитуда автоматического шага (увеличена)
        self.auto_step_amplitude_lateral_base = 0.006  # Базовая амплитуда для шагов влево/вправо (увеличена)
        self.auto_step_amplitude_max = 0.020  # Максимальная амплитуда шага (увеличена)
        self.accel_max_for_max_step = 2.0  # Максимальное отклонение ускорения для максимального шага
        self.last_auto_step_time = 0
        self.auto_step_interval_first = 0.1  # Интервал для первого шага (МАКСИМАЛЬНО БЫСТРАЯ реакция)
        self.auto_step_interval_second = 0.4  # Интервал для второго шага (медленный для успокоения)
        self.balance_step_count = 0  # Счетчик шагов: 0 = нет шагов, 1 = первый быстрый, 2 = второй медленный
        self.last_balance_direction = None  # Направление последнего шага балансировки ('forward', 'backward', 'left', 'right')
        self.balance_cooldown_end_time = 0  # Время окончания охлаждения после завершения балансировки (сек)
        self.balance_cooldown_duration = 1.0  # Длительность охлаждения после завершения балансировки (сек)
        
        # История ориентации для анализа наклона (roll и pitch)
        self.orientation_history = {
            'roll': [],   # Вперед/назад (ROLL + = назад, ROLL - = вперед)
            'pitch': []   # Влево/вправо (PITCH + = вправо, PITCH - = влево)
        }
        self.orientation_history_size = 8  # Размер истории для анализа ориентации
        
        # История ускорений для анализа ИЗМЕНЕНИЯ (производной) - не абсолютные значения!
        self.accel_history = {
            'x': [],  # Влево/вправо
            'y': []   # Вперед/назад (относительно гравитации)
        }
        self.accel_history_size = 4  # Размер истории для анализа изменений (МИНИМУМ для быстрой реакции)
        
        # Калибровка базового значения гравитации и уровня шумов при запуске
        self.gravity_base_y = 9.8  # Начальное значение, будет обновляться при калибровке
        self.gravity_calibration_samples = 30  # Количество образцов для калибровки при запуске
        self.gravity_calibrated = False  # Флаг калибровки
        self.calibration_data = {'x': [], 'y': []}  # Данные для калибровки
        self.calibration_in_progress = True  # Флаг процесса калибровки
        self.noise_level_x = 0.1  # Уровень шума по оси X (будет определен при калибровке)
        self.noise_level_y = 0.1  # Уровень шума по оси Y (будет определен при калибровке)
        self.last_stable_accel_y = None  # Последнее стабильное значение ay
        
        # Параметры для автоматического подъема при падении
        self.auto_getup_enabled = True
        self.fall_time_threshold = 3.0  # Время в секундах, после которого автоматически подниматься
        self.fall_start_time = None  # Время начала падения
        self.last_auto_getup_time = 0  # Время последнего автоматического подъема
        self.auto_getup_interval = 5.0  # Минимальный интервал между попытками подъема (сек)
        self.getup_action_in_progress = False  # Флаг что действие подъема уже выполняется
        
        # Параметры для отслеживания резонанса
        self.resonance_detection_enabled = True
        self.max_safe_amplitude = 0.3  # Максимальная безопасная амплитуда качания (м/с²)
        self.critical_amplitude = 0.5  # Критическая амплитуда, требующая немедленного уменьшения шага
        self.base_gait_params = None  # Базовые параметры походки для восстановления
        self.current_adaptation_factor = 1.0  # Фактор адаптации (1.0 = без изменений, <1.0 = уменьшение)
        
        # Блокировка для thread-safe доступа к IMU данным
        self.imu_lock = threading.Lock()
        
        # Переменная для периодического логирования
        self.last_log_time = 0
        self.log_interval = 2.0  # Интервал логирования в секундах
        self.imu_data_received = False  # Флаг первого получения IMU данных

        # --- NEW: Refactored speed parameters into a dictionary ---
        self.setup_speed_parameters()
        
        # Логирование инициализации
        rospy.loginfo("=" * 60)
        rospy.loginfo("🤖 IMU-based Gait Adaptation System Initialized")
        rospy.loginfo(f"   Auto-balance: enabled={self.auto_balance_enabled}")
        rospy.loginfo(f"   Auto-balance uses LINEAR ACCELERATION CHANGE (производная)")
        rospy.loginfo(f"   Acceleration mapping: Y- = backward, Y+ = forward, X+ = right, X- = left")
        rospy.loginfo(f"   Change threshold: {self.accel_change_threshold:.3f} m/s², min: {self.accel_change_min_threshold:.3f} m/s²")
        rospy.loginfo(f"   Auto-step intervals: first={self.auto_step_interval_first}s (fast), second={self.auto_step_interval_second}s (slow)")
        rospy.loginfo(f"   Auto-step amplitudes: base={self.auto_step_amplitude_base}, max={self.auto_step_amplitude_max}, lateral_base={self.auto_step_amplitude_lateral_base}")
        rospy.loginfo(f"   Balance logic: 1 fast long step + 1 slow short step for stabilization")
        rospy.loginfo(f"   History window: {self.accel_history_size} samples")
        rospy.loginfo(f"   Auto-getup: enabled={self.auto_getup_enabled}, threshold={self.fall_time_threshold}s")
        rospy.loginfo(f"   Resonance detection: enabled={self.resonance_detection_enabled}")
        rospy.loginfo(f"   Resonance thresholds: safe={self.max_safe_amplitude}, critical={self.critical_amplitude}")
        rospy.loginfo("=" * 60)

        # --- Initialization from primer.py ---
        self.sound = None
        self.setup_primer_features()

        # --- Start ROS Subscribers ---
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        # NEW: Subscriber for robot state from IMU
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        rospy.loginfo("Joystick Controller Initialized. Speed Mode: 1")
        time.sleep(0.2)

    def setup_speed_parameters(self):
        """Initializes a dictionary with all walking parameters for each speed mode."""
        self.speed_params = {
            1: {'period_time': [400, 0.2, 0.022], 'x_amp': 0.01, 'y_amp': 0.015, 'angle_amp': 8, 'z_move_amplitude': 0.025},
            2: {'period_time': [500, 0.2, 0.028], 'x_amp': 0.015, 'y_amp': 0.015, 'angle_amp': 10, 'z_move_amplitude': 0.02, 'gait_base': {'dsp_ratio': 0.2, 'step_fb_ratio': 0.028, 'y_swap_amplitude': 0.02, 'z_swap_amplitude': 0.006, 'init_y_offset': -0.008}},
            3: {'period_time': [400, 0.2, 0.028], 'x_amp': 0.01, 'y_amp': 0.015, 'angle_amp': 10, 'z_move_amplitude': 0.02, 'gait_base': {'dsp_ratio': 0.2, 'init_y_offset': -0.005, 'step_fb_ratio': 0.028, 'y_swap_amplitude': 0.02, 'z_swap_amplitude': 0.006}},
            4: {'period_time': [300, 0.2, 0.028], 'x_amp': 0.01, 'y_amp': 0.015, 'angle_amp': 8, 'z_move_amplitude': 0.015, 'gait_base': {'dsp_ratio': 0.2, 'init_y_offset': -0.008, 'step_fb_ratio': 0.028, 'y_swap_amplitude': 0.021, 'z_swap_amplitude': 0.006, 'pelvis_offset': 5, 'arm_swing_gain': 0.5}}
        }

    def get_serial_port(self):
        """Thread-safe method to get serial port instance."""
        with self.serial_lock:
            return self.serial_port

    def reconnect_serial(self):
        """Attempts to reconnect to Arduino serial port."""
        with self.serial_lock:
            # Check if already connected
            if self.serial_port is not None and self.serial_port.is_open:
                try:
                    # Quick check if port is still valid
                    self.serial_port.in_waiting
                    return True  # Already connected and working
                except:
                    # Port is open but not working, close it
                    try:
                        self.serial_port.close()
                    except:
                        pass
                    self.serial_port = None
            
            # Close existing connection if any (redundant check)
            if self.serial_port is not None:
                try:
                    if self.serial_port.is_open:
                        self.serial_port.close()
                except Exception as e:
                    rospy.logdebug(f"Error closing serial port: {e}")
                self.serial_port = None
            
            # Try to reconnect
            try:
                if os.path.exists(self.serial_port_name):
                    self.serial_port = serial.Serial(self.serial_port_name, self.serial_baudrate, timeout=1)
                    rospy.loginfo(f"Successfully connected to Arduino on {self.serial_port_name} at {self.serial_baudrate} baud.")
                    return True
                else:
                    rospy.logdebug(f"Serial port {self.serial_port_name} does not exist yet.")
                    return False
            except serial.SerialException as e:
                rospy.logdebug(f"Failed to connect to serial port: {e}")
                return False
            except Exception as e:
                rospy.logwarn(f"Unexpected error during serial reconnection: {e}")
                return False

    def check_and_reconnect_serial(self, event):
        """Periodically checks serial connection and attempts reconnection if needed."""
        ser = self.get_serial_port()
        if ser is None or not ser.is_open:
            # Try to reconnect (will log success internally)
            self.reconnect_serial()

    def setup_primer_features(self):
        """Initializes Serial, Sound, Publisher, and Thread."""
        # Try to connect to serial port (non-blocking - will retry in background)
        self.reconnect_serial()
        
        try:
            import pygame.mixer
            pygame.mixer.init()
            sound_file_path = os.path.abspath("/home/ubuntu/ros_ws/src/proverka_nod/scripts/FIRED.wav")
            if os.path.exists(sound_file_path):
                self.sound = pygame.mixer.Sound(sound_file_path)
                rospy.loginfo(f"Sound file loaded: {sound_file_path}")
            else:
                rospy.logwarn(f"Sound file not found: {sound_file_path}")
        except ImportError:
            rospy.logwarn("pygame module not available - sound features disabled")
            self.sound = None
        except Exception as e:
            rospy.logwarn(f"Failed to initialize pygame mixer or load sound: {e}")
            self.sound = None
        
        # Game-related publishers
        self.hit_detection_pub = rospy.Publisher('/game/hit_detection', String, queue_size=10)
        self.hp_pub = rospy.Publisher('/game/robot_hp', Int32, queue_size=10)
        self.firing_state_pub = rospy.Publisher('/game/firing_state', Bool, queue_size=10)
        self.robot_status_pub = rospy.Publisher('/game/robot_status', String, queue_size=10)
        
        # Game-related subscribers
        self.damage_sub = rospy.Subscriber('/game/validated_damage', Int32, self.damage_callback)
        self.control_lock_sub = rospy.Subscriber('/game/control_lock', Bool, self.control_lock_callback)  # Legacy
        self.control_permissions_sub = rospy.Subscriber('/game/control_permissions', String, self.control_permissions_callback)
        self.match_start_sub = rospy.Subscriber('/game/match_start', Bool, self.match_start_callback)
        
        # Initialize serial reader thread with hit detection publisher
        # Pass getter function instead of direct reference
        self.serial_reader_thread = SerialReader(self.get_serial_port, self.hit_detection_pub, self.robot_id)
        self.serial_reader_thread.daemon = True
        self.serial_reader_thread.start()
        rospy.on_shutdown(self.serial_reader_thread.stop)
        
        # Health check service
        self.health_service = rospy.Service('/game/robot_health', Trigger, self.health_check_service)
        
        # Reset HP service
        self.reset_hp_service = rospy.Service('/game/reset_hp', Trigger, self.reset_hp_service_handler)
        
        # Start HP publishing timer (1-2Hz)
        self.hp_timer = rospy.Timer(rospy.Duration(0.5), self.publish_hp_status)
        
        # Start serial reconnection check timer (every 5 seconds)
        self.serial_reconnect_timer = rospy.Timer(rospy.Duration(5.0), self.check_and_reconnect_serial)

    def damage_callback(self, msg):
        """Handle validated damage from server."""
        damage_amount = msg.data
        if damage_amount <= 0:
            return
        
        old_hp = self.current_hp
        self.current_hp = max(0, self.current_hp - damage_amount)
        
        rospy.loginfo(f"💥 Damage received: {damage_amount}, HP: {old_hp} → {self.current_hp}")
        
        # КРИТИЧЕСКИ ВАЖНО: Публиковать обновленное HP сразу после получения урона
        hp_msg = Int32()
        hp_msg.data = self.current_hp
        self.hp_pub.publish(hp_msg)
        rospy.loginfo(f"📤 Published HP update: {self.current_hp}")
        
        # Если HP достигло 0, заблокировать робота
        if self.current_hp <= 0:
            self.is_locked = True
            # Блокируем все разрешения кроме камеры при HP = 0
            self.permissions['movement'] = False
            self.permissions['head'] = False
            self.permissions['firing'] = False
            self.permissions['camera'] = True  # Камера остается доступной
            rospy.logwarn(f"💀 Robot HP reached 0, locking robot")
            # Останавливаем движение
            if self.status == 'move':
                self.status = 'stop'
                self.gait_manager.stop()
            self.publish_robot_status("hp_zero_locked")

    def control_permissions_callback(self, msg):
        """Handle detailed control permissions from server."""
        try:
            data = json.loads(msg.data)
            # Обновляем только те разрешения, которые пришли в сообщении
            if 'movement' in data:
                self.permissions['movement'] = bool(data['movement'])
            if 'head' in data:
                self.permissions['head'] = bool(data['head'])
            if 'firing' in data:
                self.permissions['firing'] = bool(data['firing'])
            if 'camera' in data:
                self.permissions['camera'] = bool(data['camera'])
            
            self.use_detailed_permissions = True
            rospy.loginfo(f"Updated permissions: movement={self.permissions['movement']}, head={self.permissions['head']}, firing={self.permissions['firing']}, camera={self.permissions['camera']}")
            
            # Останавливаем движение, если оно было заблокировано
            if not self.permissions['movement'] and self.status == 'move':
                self.status = 'stop'
                self.gait_manager.stop()
                rospy.logwarn("Movement stopped due to permission change")
            
            self.publish_robot_status("permissions_changed")
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing control permissions JSON: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing control permissions: {e}")
    
    def control_lock_callback(self, msg):
        """Handle legacy control lock/unlock commands from server (backward compatibility)."""
        # Если используются детальные разрешения, игнорируем legacy команды
        if self.use_detailed_permissions:
            rospy.logdebug("Ignoring legacy control_lock message - using detailed permissions")
            return
        
        self.is_locked = msg.data
        if self.is_locked:
            # Полная блокировка (legacy)
            self.permissions = {
                'movement': False,
                'head': False,
                'firing': False,
                'camera': True  # Камера остается доступной
            }
            rospy.loginfo("Robot control locked by server (legacy mode).")
            # Stop movement if locked
            if self.status == 'move':
                self.status = 'stop'
                self.gait_manager.stop()
        else:
            # Полная разблокировка (legacy)
            self.permissions = {
                'movement': True,
                'head': True,
                'firing': True,
                'camera': True
            }
            rospy.loginfo("Robot control unlocked by server (legacy mode).")
        self.publish_robot_status("lock_changed" if self.is_locked else "unlock_changed")

    def publish_hp_status(self, event):
        """Публикует HP каждые 0.5 секунды (2Hz) для синхронизации."""
        hp_msg = Int32()
        hp_msg.data = self.current_hp
        self.hp_pub.publish(hp_msg)

    def reset_hp(self):
        """Сбросить HP на 100 (вызывается при начале нового матча)."""
        self.current_hp = 100
        hp_msg = Int32()
        hp_msg.data = self.current_hp
        self.hp_pub.publish(hp_msg)
        rospy.loginfo(f"💚 HP reset to 100")
        
        # Разблокировать робота при сбросе HP
        self.is_locked = False
        self.permissions = {
            'movement': True,
            'head': True,
            'firing': True,
            'camera': True
        }
        self.publish_robot_status("hp_reset")

    def match_start_callback(self, msg):
        """Обработчик начала нового матча."""
        if msg.data:  # Если матч начался
            rospy.loginfo("🎮 Match started, resetting HP")
            self.reset_hp()

    def reset_hp_service_handler(self, req):
        """Service handler для сброса HP."""
        response = TriggerResponse()
        try:
            self.reset_hp()
            response.success = True
            response.message = "HP reset to 100"
        except Exception as e:
            response.success = False
            response.message = f"Error resetting HP: {str(e)}"
            rospy.logerr(response.message)
        return response

    def publish_robot_status(self, status_type):
        """Publish general robot status to server."""
        status_data = {
            'robot_id': self.robot_id,
            'status_type': status_type,
            'hp': self.current_hp,
            'locked': self.is_locked,
            'timestamp': rospy.get_time()
        }
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.robot_status_pub.publish(status_msg)

    def can_move(self):
        """Проверка разрешения на движение."""
        if self.use_detailed_permissions:
            return self.permissions['movement']
        else:
            return not self.is_locked
    
    def can_control_head(self):
        """Проверка разрешения на управление головой."""
        if self.use_detailed_permissions:
            return self.permissions['head']
        else:
            return not self.is_locked
    
    def can_fire(self):
        """Проверка разрешения на стрельбу."""
        if self.use_detailed_permissions:
            return self.permissions['firing']
        else:
            return not self.is_locked
    
    def can_access_camera(self):
        """Проверка разрешения на доступ к камере."""
        if self.use_detailed_permissions:
            return self.permissions['camera']
        else:
            return True  # Камера всегда доступна в legacy режиме
    
    def health_check_service(self, req):
        """Service handler for robot health check."""
        response = TriggerResponse()
        try:
            ser = self.get_serial_port()
            arduino_connected = ser is not None and ser.is_open if ser else False
            
            # Определяем состояние блокировки на основе разрешений
            if self.use_detailed_permissions:
                locked = not (self.permissions['movement'] or self.permissions['firing'])
            else:
                locked = self.is_locked
            
            health_data = {
                'available': True,
                'hp': self.current_hp,
                'locked': locked,
                'connected': arduino_connected,
                'robot_id': self.robot_id
            }
            response.success = True
            response.message = json.dumps(health_data)
            rospy.logdebug(f"Health check requested. Response: {response.message}")
        except Exception as e:
            response.success = False
            response.message = f"Error in health check: {str(e)}"
            rospy.logerr(response.message)
        return response

    # NEW: IMU Callback from joystick_control 1.py
    def imu_callback(self, msg: Imu):
        """
        Processes IMU data to determine fall state and adapt gait parameters.
        Logic adapted from mobile app example, using ay and az.
        Now also handles automatic balancing and resonance detection.
        """
        try:
            with self.imu_lock:
                # Получаем сырые данные
                accel_raw = np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])
                
                # Сохраняем ИСХОДНЫЕ значения для логики падения (не трогаем логику падений)
                ay_original = msg.linear_acceleration.y
                az_original = msg.linear_acceleration.z
                
                # Применяем преобразование осей для калибровки и балансировки (как в imu_visualizer.py)
                accel_transformed = transform_axes_vector(accel_raw)
                ax = accel_transformed[0]
                ay = accel_transformed[1]
                az = accel_transformed[2]
                
                # Гироскоп тоже преобразуем
                gyro_raw = np.array([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ])
                gyro_transformed = transform_axes_vector(gyro_raw)
                gx = gyro_transformed[0]
                gy = gyro_transformed[1]
                gz = gyro_transformed[2]

                # Извлечение углов ориентации из quaternion
                qx = msg.orientation.x
                qy = msg.orientation.y
                qz = msg.orientation.z
                qw = msg.orientation.w
                
                # Преобразование quaternion в углы Эйлера (roll, pitch, yaw)
                # Roll (вращение вокруг оси X)
                sinr_cosp = 2 * (qw * qx + qy * qz)
                cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                roll = math.atan2(sinr_cosp, cosr_cosp)
                
                # Pitch (вращение вокруг оси Y)
                sinp = 2 * (qw * qy - qz * qx)
                if abs(sinp) >= 1:
                    pitch = math.copysign(math.pi / 2, sinp)
                else:
                    pitch = math.asin(sinp)
                
                # Yaw (вращение вокруг оси Z)
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                # Обновление истории IMU данных
                self.imu_history['ax'].append(ax)
                self.imu_history['ay'].append(ay)
                self.imu_history['az'].append(az)
                self.imu_history['roll'].append(roll)
                self.imu_history['pitch'].append(pitch)
                self.imu_history['yaw'].append(yaw)
                
                # orientation_reference больше не используется - перешли на ускорения
                
                # Ограничение размера истории
                for key in self.imu_history:
                    if len(self.imu_history[key]) > self.imu_history_size:
                        self.imu_history[key].pop(0)
                
                # Логирование первого получения IMU данных
                if not self.imu_data_received:
                    self.imu_data_received = True
                    rospy.loginfo("✅ IMU data received! Starting calibration...")
                    rospy.loginfo(f"   First IMU reading - Pitch: {math.degrees(pitch):.1f}°, Roll: {math.degrees(roll):.1f}°, Accel: X={ax:.2f}, Y={ay:.2f}, Z={az:.2f}")
                    rospy.loginfo(f"   Collecting {self.gravity_calibration_samples} samples for calibration (robot should be stable)...")
                
                # КАЛИБРОВКА при запуске: собираем данные когда робот стабилен
                if self.calibration_in_progress:
                    self._perform_startup_calibration(ax, ay)

            # Обработка падения (оригинальная логика) - используем ИСХОДНЫЕ значения БЕЗ преобразования
            current_time = rospy.get_time()
            
            # Нормальная проверка падения (НЕ в период cooldown)
            if current_time >= self.fall_check_cooldown:
                ACCEL_THRESH = 7.0
                ANGLE_THRESH = 30.0
                COUNT_INCREMENT = 1
                COUNT_DECREMENT = 2  # Увеличена скорость сброса

                if abs(az_original) > 1e-6:
                    angle_rad = math.atan2(abs(ay_original), abs(az_original))
                    angle_deg = math.degrees(angle_rad)
                else:
                    angle_deg = 90.0

                # ЛОГИКА ПАДЕНИЯ: использует ИСХОДНЫЕ ay и az (НЕ изменена, БЕЗ преобразования осей)
                # az > 7.0 = падение вперед (lie_to_stand)
                # az < -7.0 = падение назад (recline_to_stand)
                if angle_deg < ANGLE_THRESH:
                    if az_original > ACCEL_THRESH:
                        self.count_lie += COUNT_INCREMENT
                        self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
                    elif az_original < -ACCEL_THRESH:
                        self.count_recline += COUNT_INCREMENT
                        self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                    else:
                        # az в нормальном диапазоне - уменьшаем счетчики быстрее
                        self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                        self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
                else:
                    # Угол нормальный - уменьшаем счетчики быстрее
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
            else:
                # В период cooldown - АГРЕССИВНО сбрасываем счетчики, не увеличиваем
                self.count_lie = max(0, self.count_lie - 15)  # Очень быстро сбрасываем (увеличено)
                self.count_recline = max(0, self.count_recline - 15)

            old_state = self.robot_state

            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'lie_to_stand'
            elif self.count_recline > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'recline_to_stand'
            else:
                self.robot_state = 'stand'

            if old_state != self.robot_state:
                rospy.loginfo(f"🔄 IMU detected robot state change: '{old_state}' -> '{self.robot_state}' (lie_count:{self.count_lie}, recline_count:{self.count_recline})")
                # Отслеживаем время начала падения
                if self.robot_state != 'stand' and old_state == 'stand':
                    # Робот упал - устанавливаем время начала падения
                    if self.fall_start_time is None:  # Устанавливаем только если еще не установлено
                        self.fall_start_time = rospy.get_time()
                        rospy.logwarn(f"⚠️ Robot fell! State: {self.robot_state}, starting fall timer...")
                elif self.robot_state == 'stand' and old_state != 'stand':
                    # Робот встал - АГРЕССИВНО сбрасываем данные о падении
                    self.fall_start_time = None
                    self.last_auto_getup_time = 0  # Сбрасываем таймер подъема
                    self.getup_action_in_progress = False  # Сбрасываем флаг выполнения действия
                    # АГРЕССИВНЫЙ сброс счетчиков падения - ПРИНУДИТЕЛЬНО обнуляем
                    self.count_lie = 0
                    self.count_recline = 0
                    # Дополнительно: устанавливаем задержку перед следующей проверкой падения
                    self.fall_check_cooldown = rospy.get_time() + 10.0  # 10 секунд задержки (увеличено)
                    self.balance_step_count = 0  # Сбрасываем счетчик шагов балансировки
                    self.last_balance_direction = None
                    rospy.loginfo(f"✅ Robot recovered to stand position - all fall/balance data reset. Cooldown: 10.0s, counters forced to 0")

            # Новая логика: автоматический подъем при падении
            if self.auto_getup_enabled and self.robot_state != 'stand':
                self._handle_auto_getup()

            # Новая логика: автоматическая балансировка в покое (использует линейные ускорения)
            # Новая логика: автоматическая балансировка в покое (использует ИЗМЕНЕНИЕ линейных ускорений)
            # Работает только после завершения калибровки
            if self.auto_balance_enabled and self.robot_state == 'stand' and self.gravity_calibrated:
                self._handle_auto_balance_accel_change(ax, ay)

            # Новая логика: обнаружение резонанса и адаптация параметров
            if self.resonance_detection_enabled and self.status == 'move':
                self._handle_resonance_detection()
            
            # Логирование IMU данных для отладки (периодически)
            current_time = rospy.get_time()
            if current_time - self.last_log_time >= self.log_interval:
                self._log_imu_status(pitch, roll, yaw, ax, ay, az, gx, gy, gz)
                self.last_log_time = current_time

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data in imu_callback: {e}")

    def _perform_startup_calibration(self, ax, ay):
        """
        Выполняет калибровку базового значения гравитации и уровня шумов при запуске ноды.
        Робот должен быть неподвижен и стоять стабильно.
        """
        # Собираем данные для калибровки
        self.calibration_data['x'].append(ax)
        self.calibration_data['y'].append(ay)
        
        # Ограничиваем размер
        if len(self.calibration_data['y']) > self.gravity_calibration_samples:
            self.calibration_data['x'].pop(0)
            self.calibration_data['y'].pop(0)
        
        # Когда собрали достаточно данных, выполняем калибровку
        if len(self.calibration_data['y']) >= self.gravity_calibration_samples:
            # Вычисляем средние значения
            avg_x = sum(self.calibration_data['x']) / len(self.calibration_data['x'])
            avg_y = sum(self.calibration_data['y']) / len(self.calibration_data['y'])
            
            # Вычисляем уровень шума (стандартное отклонение)
            x_variance = sum((x - avg_x) ** 2 for x in self.calibration_data['x']) / len(self.calibration_data['x'])
            y_variance = sum((y - avg_y) ** 2 for y in self.calibration_data['y']) / len(self.calibration_data['y'])
            self.noise_level_x = math.sqrt(x_variance)
            self.noise_level_y = math.sqrt(y_variance)
            
            # Устанавливаем базовое значение гравитации
            self.gravity_base_y = avg_y
            self.gravity_calibrated = True
            self.calibration_in_progress = False
            
            # Обновляем минимальный порог на основе уровня шума
            # Минимальный порог должен быть больше уровня шума
            self.accel_min_threshold = max(0.1, self.noise_level_y * 1.5)
            
            rospy.loginfo("=" * 60)
            rospy.loginfo("📐 CALIBRATION COMPLETE")
            rospy.loginfo(f"   Gravity base (Y): {self.gravity_base_y:.3f} m/s²")
            rospy.loginfo(f"   Noise levels: X={self.noise_level_x:.3f}, Y={self.noise_level_y:.3f} m/s²")
            rospy.loginfo(f"   Adjusted min threshold: {self.accel_min_threshold:.3f} m/s²")
            rospy.loginfo("   Auto-balance system is now ACTIVE")
            rospy.loginfo("=" * 60)
            
            # Очищаем данные калибровки
            self.calibration_data = {'x': [], 'y': []}

    def _handle_auto_getup(self):
        """
        Автоматически вызывает подъем робота, если он упал и лежит дольше заданного времени.
        ЗАЩИТА ОТ ПОВТОРНЫХ ВЫЗОВОВ: использует флаг getup_action_in_progress.
        """
        if not self.can_move():
            return
        
        current_time = rospy.get_time()
        
        # КРИТИЧНО: не пытаемся подниматься в период cooldown после подъема
        if current_time < self.fall_check_cooldown:
            return
        
        # КРИТИЧНО: если действие подъема уже выполняется - не вызываем повторно
        if self.getup_action_in_progress:
            return
        
        # Проверяем, что робот действительно упал
        if self.robot_state == 'stand':
            self.fall_start_time = None
            self.getup_action_in_progress = False
            return
        
        # Если время начала падения не установлено, устанавливаем его
        if self.fall_start_time is None:
            self.fall_start_time = current_time
            return
        
        # Проверяем, прошло ли достаточно времени с последней попытки подъема
        if current_time - self.last_auto_getup_time < self.auto_getup_interval:
            return
        
        # Проверяем, лежит ли робот достаточно долго
        fall_duration = current_time - self.fall_start_time
        if fall_duration >= self.fall_time_threshold:
            rospy.logwarn(f"🚨 Auto-getup: Robot has been down for {fall_duration:.1f} seconds (threshold: {self.fall_time_threshold}s)")
            rospy.logwarn(f"   Attempting automatic get-up action: {self.robot_state}")
            
            try:
                action_to_run = None
                if self.robot_state == 'lie_to_stand':
                    action_to_run = self.lie_to_stand_action_name
                elif self.robot_state == 'recline_to_stand':
                    action_to_run = self.recline_to_stand_action_name
                
                if action_to_run and self.motion_manager is not None:
                    # УСТАНАВЛИВАЕМ ФЛАГ ПЕРЕД ВЫЗОВОМ - защита от повторных вызовов
                    self.getup_action_in_progress = True
                    rospy.loginfo(f"🤖 Executing auto-getup action: {action_to_run}")
                    self.motion_manager.run_action(action_to_run)
                    self.last_auto_getup_time = current_time
                    # НЕ сбрасываем fall_start_time сразу - ждем подтверждения что робот встал
                    # Сброс произойдет в imu_callback когда robot_state станет 'stand'
                    rospy.loginfo(f"✅ Auto-getup action '{action_to_run}' initiated (flag set to prevent duplicate calls)")
                else:
                    rospy.logwarn(f"⚠️ Cannot execute auto-getup: action={action_to_run}, motion_manager={self.motion_manager is not None}")
            except Exception as e:
                rospy.logerr(f"❌ Error in auto-getup: {e}")
                self.last_auto_getup_time = current_time  # Все равно обновляем время, чтобы не спамить
                self.getup_action_in_progress = False  # Сбрасываем флаг при ошибке

    def _handle_auto_balance_accel_change(self, ax, ay):
        """
        Обрабатывает автоматическую балансировку используя ИЗМЕНЕНИЕ линейных ускорений (производную).
        КРИТИЧНО: используем ИЗМЕНЕНИЕ ускорения, а не абсолютное значение.
        Это позволяет ловить толчки/качания, а не реагировать на постоянные смещения.
        
        Y- = падение назад → нужен шаг назад
        Y+ = падение вперед → нужен шаг вперед
        X+ = вправо → нужен шаг вправо
        X- = влево → нужен шаг влево
        
        КРИТИЧНО: Работает ТОЛЬКО когда робот в покое и НЕТ команд от джойстика.
        НЕ работает если робот упал (robot_state != 'stand').
        """
        if not self.can_move():
            return
        
        current_time = rospy.get_time()
        
        # КРИТИЧНО: БЛОКИРОВКА - не запускаем НОВУЮ балансировку если идет охлаждение
        # Но если balance_step_count > 0, это означает что мы уже в процессе выполнения шагов - продолжаем
        if self.balance_step_count == 0 and current_time < self.balance_cooldown_end_time:
            # Охлаждение после завершения предыдущей балансировки - не генерируем новые команды
            return
        
        # КРИТИЧНО: балансировка НЕ работает если робот упал или начинает падать
        # Проверяем не только robot_state, но и счетчики падения для раннего обнаружения
        if (self.robot_state != 'stand' or 
            self.count_lie > 10 or 
            self.count_recline > 10):
            if self.balance_step_count > 0:
                rospy.logdebug(f"🛑 Auto-balance disabled: robot state={self.robot_state}, lie_count={self.count_lie}, recline_count={self.count_recline}")
            self.balance_step_count = 0
            self.last_balance_direction = None
            self.accel_history = {'x': [], 'y': []}
            return
        
        # СТРОГАЯ ПРОВЕРКА: балансировка НЕ должна работать при управлении джойстиком
        if (self.status != 'stop' or 
            self.update_param or 
            abs(self.x_move_amplitude) > 0.001 or 
            abs(self.y_move_amplitude) > 0.001 or 
            abs(self.angle_move_amplitude) > 0.001):
            # Есть команды от джойстика - полностью отключаем балансировку
            if self.balance_step_count > 0:
                rospy.logdebug(f"🛑 Auto-balance disabled: joystick control active")
            self.balance_step_count = 0
            self.last_balance_direction = None
            self.accel_history = {'x': [], 'y': []}
            return
        
        # Проверяем таймаут после достижения максимума шагов
        if self.balance_step_count >= 2:
            if self.auto_balance_timeout_start == 0:
                self.auto_balance_timeout_start = current_time
                rospy.logwarn(f"⏸️ Auto-balance: Reached max steps ({self.balance_step_count})")
            
            # Если таймаут еще не истек, ждем
            if current_time - self.auto_balance_timeout_start < self.auto_balance_timeout:
                return
            
            # Таймаут истек - проверяем результат
            if self.robot_state != 'stand':
                rospy.logwarn(f"⚠️ Auto-balance: Robot fell after {self.balance_step_count} steps")
                self.balance_step_count = 0
                self.last_balance_direction = None
                self.accel_history = {'x': [], 'y': []}
                return
            else:
                rospy.loginfo(f"✅ Auto-balance: Robot stable after {self.balance_step_count} steps, resetting")
                self.balance_step_count = 0
                self.last_balance_direction = None
                self.accel_history = {'x': [], 'y': []}
        
        # Добавляем текущие ускорения в историю (относительно гравитации для Y)
        ay_deviation = ay - self.gravity_base_y  # Отклонение от гравитации
        self.accel_history['x'].append(ax)
        self.accel_history['y'].append(ay_deviation)
        
        # Ограничиваем размер истории
        for key in self.accel_history:
            if len(self.accel_history[key]) > self.accel_history_size:
                self.accel_history[key].pop(0)
        
        # МИНИМУМ для быстрой реакции: нужно всего 3 значения
        if len(self.accel_history['x']) < 3:
            return
        
        # Вычисляем ИЗМЕНЕНИЕ ускорения (производную) - МАКСИМАЛЬНО БЫСТРО
        # Берем разницу между последним и предыдущим значением (самая быстрая реакция)
        if len(self.accel_history['x']) >= 2:
            change_x = self.accel_history['x'][-1] - self.accel_history['x'][-2]
            change_y = self.accel_history['y'][-1] - self.accel_history['y'][-2]
        else:
            return
        
        # Простая проверка качания: если знак меняется - это качание (только для второго шага)
        if self.balance_step_count == 1 and len(self.accel_history['x']) >= 3:
            # Проверяем, не качается ли робот
            if (self.accel_history['x'][-1] >= 0) != (self.accel_history['x'][-2] >= 0) or \
               (self.accel_history['y'][-1] >= 0) != (self.accel_history['y'][-2] >= 0):
                rospy.logdebug(f"🔄 Auto-balance: Oscillating detected, skipping second step")
                self.balance_cooldown_end_time = current_time + self.balance_cooldown_duration
                self.balance_step_count = 0
                self.last_balance_direction = None
                return
        
        # Проверка: изменение должно быть значительным (только для первого шага)
        if self.balance_step_count == 0:
            if abs(change_x) < self.accel_change_threshold and abs(change_y) < self.accel_change_threshold:
                return  # Нет значительного изменения - не делаем шаг
        else:
            # Для второго шага проверяем, что изменение все еще есть (но меньше)
            if abs(change_x) < self.accel_change_min_threshold and abs(change_y) < self.accel_change_min_threshold:
                # Изменение слишком мало - сбрасываем счетчик и устанавливаем охлаждение
                self.balance_cooldown_end_time = current_time + self.balance_cooldown_duration
                self.balance_step_count = 0
                self.last_balance_direction = None
                return
        
        # Определяем направление по ИЗМЕНЕНИЮ ускорения
        # change_y < 0 = ускорение уменьшается (падает назад) → нужен шаг назад
        # change_y > 0 = ускорение увеличивается (падает вперед) → нужен шаг вперед
        # change_x > 0 = ускорение вправо увеличивается → нужен шаг вправо
        # change_x < 0 = ускорение влево увеличивается → нужен шаг влево
        
        direction = None
        step_x = 0
        step_y = 0
        change_magnitude = 0.0
        
        # Определяем направление по ИЗМЕНЕНИЮ ускорения
        # Приоритет: сначала вперед/назад, потом влево/вправо
        direction = None
        step_x = 0
        step_y = 0
        change_magnitude = 0.0
        
        if abs(change_y) > abs(change_x):
            # Доминирует изменение вперед/назад
            if change_y < -self.accel_change_threshold:
                direction = 'backward'
                change_magnitude = abs(change_y)
            elif change_y > self.accel_change_threshold:
                direction = 'forward'
                change_magnitude = abs(change_y)
        else:
            # Доминирует изменение влево/вправо
            if change_x > self.accel_change_threshold:
                direction = 'right'
                change_magnitude = abs(change_x)
            elif change_x < -self.accel_change_threshold:
                direction = 'left'
                change_magnitude = abs(change_x)
        
        # Если изменение слишком мало - сбрасываем счетчик (только для первого шага)
        if direction is None:
            if self.balance_step_count > 0:
                self.balance_cooldown_end_time = current_time + self.balance_cooldown_duration
                self.balance_step_count = 0
                self.last_balance_direction = None
            return
        
        # Если направление изменилось, сбрасываем счетчик (робот качается)
        if self.last_balance_direction is not None and self.last_balance_direction != direction:
            rospy.loginfo(f"🔄 Auto-balance: Direction changed from {self.last_balance_direction} to {direction}, resetting")
            self.balance_cooldown_end_time = current_time + self.balance_cooldown_duration
            self.balance_step_count = 0
            self.last_balance_direction = None
        
        # Определяем какой шаг делаем: первый (быстрый длинный) или второй (медленный короткий)
        is_first_step = (self.balance_step_count == 0)
        
        # Проверяем, прошло ли достаточно времени с последнего шага
        if is_first_step:
            step_interval = self.auto_step_interval_first
        else:
            step_interval = self.auto_step_interval_second
        
        if current_time - self.last_auto_step_time < step_interval:
            return
        
        # Вычисляем амплитуду шага
        if is_first_step:
            # Первый шаг: БЫСТРЫЙ и ДЛИННЫЙ (зависит от ускорения)
            step_amplitude = self._calculate_dynamic_amplitude(change_magnitude, self.auto_step_amplitude_base, self.auto_step_amplitude_max)
            fast = True
        else:
            # Второй шаг: МЕДЛЕННЫЙ и КОРОТКИЙ (для успокоения)
            step_amplitude = self.auto_step_amplitude_base * 0.5  # Половина базовой амплитуды
            fast = False
        
        # Определяем направление шага
        if direction == 'backward':
            step_x = -step_amplitude
        elif direction == 'forward':
            step_x = step_amplitude
        elif direction == 'right':
            step_y = step_amplitude
        elif direction == 'left':
            step_y = -step_amplitude
        
        # Выполняем шаг
        step_type = "FAST LONG" if is_first_step else "SLOW SHORT"
        rospy.loginfo(f"🤖 Auto-balance: Robot tilting {direction} (change_x={change_x:.3f}, change_y={change_y:.3f}, magnitude={change_magnitude:.3f} m/s²), making {step_type} step ({self.balance_step_count + 1}/2)")
        self._make_auto_step_2d(step_x, step_y, fast=fast)
        self.last_auto_step_time = current_time
        self.balance_step_count += 1
        self.last_balance_direction = direction
        
        # После второго шага сбрасываем счетчик и устанавливаем охлаждение
        if self.balance_step_count >= 2:
            self.balance_cooldown_end_time = current_time + self.balance_cooldown_duration
            self.balance_step_count = 0
            self.last_balance_direction = None
            rospy.loginfo(f"✅ Auto-balance: Completed 2 steps, cooldown for {self.balance_cooldown_duration}s")

    def _handle_auto_balance_accel(self, ax, ay, pitch, roll):
        """
        Обрабатывает автоматическую балансировку в состоянии покоя используя ЛИНЕЙНЫЕ УСКОРЕНИЯ.
        Y- = падение назад, Y+ = падение вперед
        X+ = вправо, X- = влево
        КРИТИЧНО: Работает ТОЛЬКО когда робот в покое и НЕТ команд от джойстика.
        НЕ работает если робот упал (robot_state != 'stand').
        """
        if not self.can_move():
            return
        
        current_time = rospy.get_time()
        
        # КРИТИЧНО: балансировка НЕ работает если робот упал или начинает падать
        # Проверяем не только robot_state, но и счетчики падения для раннего обнаружения
        if (self.robot_state != 'stand' or 
            self.count_lie > 10 or 
            self.count_recline > 10):
            if self.balance_step_count > 0:
                rospy.logdebug(f"🛑 Auto-balance disabled: robot state={self.robot_state}, lie_count={self.count_lie}, recline_count={self.count_recline}")
            self.balance_step_count = 0
            self.last_balance_direction = None
            self.accel_history = {'x': [], 'y': []}
            return
        
        # СТРОГАЯ ПРОВЕРКА: балансировка НЕ должна работать при управлении джойстиком
        if (self.status != 'stop' or 
            self.update_param or 
            abs(self.x_move_amplitude) > 0.001 or 
            abs(self.y_move_amplitude) > 0.001 or 
            abs(self.angle_move_amplitude) > 0.001):
            # Есть команды от джойстика - полностью отключаем балансировку
            if self.balance_step_count > 0:
                rospy.logdebug(f"🛑 Auto-balance disabled: joystick control active")
            self.balance_step_count = 0
            self.last_balance_direction = None
            self.accel_history = {'x': [], 'y': []}
            return
        
        # Проверяем таймаут после достижения максимума шагов
        if self.balance_step_count >= 2:
            if self.auto_balance_timeout_start == 0:
                self.auto_balance_timeout_start = current_time
                rospy.logwarn(f"⏸️ Auto-balance: Reached max steps ({self.balance_step_count})")
            
            # Если таймаут еще не истек, ждем
            if current_time - self.auto_balance_timeout_start < self.auto_balance_timeout:
                return
            
            # Таймаут истек - проверяем результат
            if self.robot_state != 'stand':
                rospy.logwarn(f"⚠️ Auto-balance: Robot fell after {self.balance_step_count} steps")
                self.balance_step_count = 0
                self.last_balance_direction = None
                self.accel_history = {'x': [], 'y': []}
                return
            else:
                rospy.loginfo(f"✅ Auto-balance: Robot stable after {self.balance_step_count} steps, resetting")
                self.balance_step_count = 0
                self.last_balance_direction = None
                self.accel_history = {'x': [], 'y': []}
        
        # Добавляем текущие ускорения в историю
        self.accel_history['x'].append(ax)
        self.accel_history['y'].append(ay)
        
        # Ограничиваем размер истории (старая функция, больше не используется)
        for key in self.accel_history:
            if len(self.accel_history[key]) > 10:  # Используем фиксированное значение для старой функции
                self.accel_history[key].pop(0)
        
        # Нужно достаточно данных для анализа вектора (уменьшено для быстрой реакции)
        if len(self.accel_history['x']) < 3:
            return
        
        # Анализируем вектор ускорения - смотрим на последние значения (меньше окно для быстрой реакции)
        # Берем среднее из последних значений для определения направления
        recent_x = self.accel_history['x'][-3:]
        recent_y = self.accel_history['y'][-3:]
        
        avg_x = sum(recent_x) / len(recent_x)
        avg_y_raw = sum(recent_y) / len(recent_y)
        
        # Используем калиброванное базовое значение гравитации (определено при запуске)
        avg_y = avg_y_raw - self.gravity_base_y  # Отклонение от калиброванного базового значения
        
        # КРИТИЧНО: проверяем ИЗМЕНЕНИЕ отклонения, а не абсолютное значение
        # Если отклонение постоянно в одном направлении без изменений - это не падение
        # Используем уровень шума, определенный при калибровке (уменьшено окно для быстрой реакции)
        if len(self.accel_history['y']) >= 6:
            # Вычисляем изменение отклонения (производную)
            recent_y_deviations = [y - self.gravity_base_y for y in recent_y]
            if len(recent_y_deviations) >= 2:
                # Изменение отклонения (разница между последними значениями)
                deviation_change = abs(recent_y_deviations[-1] - recent_y_deviations[0])
                # Порог изменения основан на уровне шума (минимум снижен для чувствительности)
                change_threshold = max(0.08, self.noise_level_y * 1.5)
                # Если отклонение не меняется (стабильно в пределах шума), не балансируем
                if deviation_change < change_threshold:
                    if self.balance_step_count > 0:
                        rospy.logdebug(f"✅ Auto-balance: Deviation stable (change={deviation_change:.3f} < {change_threshold:.3f}), resetting")
                    self.balance_step_count = 0
                    self.last_balance_direction = None
                    return
        
        # КРИТИЧНО: проверяем ИЗМЕНЕНИЕ ускорения, а не абсолютное значение
        # Вычисляем изменение ускорения (производную) - разницу между последними значениями
        # Это позволяет ловить толчки/качания, а не реагировать на постоянные смещения
        
        if len(self.accel_history['x']) < 5 or len(self.accel_history['y']) < 5:
            return  # Нужно минимум данных для анализа изменений (уменьшено для быстрой реакции)
        
        # Берем более короткую историю для анализа изменений (быстрее реакция)
        extended_x = self.accel_history['x'][-5:]
        extended_y = self.accel_history['y'][-5:]
        
        # Вычисляем изменение ускорения (разница между последними и предыдущими значениями)
        # Используем разницу между средними из последних 2 и предыдущих 2 значений (быстрее)
        recent_2_x = extended_x[-2:]
        recent_2_y = extended_y[-2:]
        prev_2_x = extended_x[:2] if len(extended_x) >= 4 else extended_x[:1]
        prev_2_y = extended_y[:2] if len(extended_y) >= 4 else extended_y[:1]
        
        avg_recent_x = sum(recent_2_x) / len(recent_2_x)
        avg_recent_y = sum(recent_2_y) / len(recent_2_y)
        avg_prev_x = sum(prev_2_x) / len(prev_2_x) if prev_2_x else avg_recent_x
        avg_prev_y = sum(prev_2_y) / len(prev_2_y) if prev_2_y else avg_recent_y
        
        # Изменение ускорения (производная)
        change_x = avg_recent_x - avg_prev_x
        change_y = (avg_recent_y - self.gravity_base_y) - (avg_prev_y - self.gravity_base_y)
        
        # Находим максимальное изменение для определения силы шага
        max_change_x = max(abs(x - extended_x[0]) for x in extended_x) if extended_x else 0.0
        max_change_y = max(abs((y - self.gravity_base_y) - (extended_y[0] - self.gravity_base_y)) for y in extended_y) if extended_y else 0.0
        
        # Проверяем, что изменение значительное (не шум)
        # Используем порог, основанный на уровне шума (сильно снижен для чувствительности)
        # Используем единый порог для изменения ускорения
        
        # УЛУЧШЕННАЯ проверка качания: анализируем частоту изменения знака
        # Если знак меняется часто - это качание, не делаем шаг
        if len(extended_x) >= 4 and len(extended_y) >= 4:
            x_sign_changes = 0
            y_sign_changes = 0
            for i in range(len(extended_x) - 1):
                if (extended_x[i] >= 0) != (extended_x[i+1] >= 0):
                    x_sign_changes += 1
            for i in range(len(extended_y) - 1):
                if (extended_y[i] >= self.gravity_base_y) != (extended_y[i+1] >= self.gravity_base_y):
                    y_sign_changes += 1
            
            # Если знак меняется более 2 раз - это качание, не балансируем
            if x_sign_changes >= 2 or y_sign_changes >= 2:
                if self.balance_step_count > 0:
                    rospy.logdebug(f"🔄 Auto-balance: Oscillating detected (x_changes={x_sign_changes}, y_changes={y_sign_changes}), resetting")
                self.balance_step_count = 0
                self.last_balance_direction = None
                return
        
        # Если изменение слишком мало - это не падение, а постоянное смещение или шум
        if abs(change_x) < self.accel_change_threshold and abs(change_y) < self.accel_change_threshold:
            if self.balance_step_count > 0:
                rospy.logdebug(f"✅ Auto-balance: No significant change (dx={change_x:.3f}, dy={change_y:.3f}), resetting")
            self.balance_step_count = 0
            self.last_balance_direction = None
            return
        
        # Определяем направление по ИЗМЕНЕНИЮ ускорения (не по абсолютному значению)
        # change_y < 0 = ускорение уменьшается (падает назад) → нужен шаг назад
        # change_y > 0 = ускорение увеличивается (падает вперед) → нужен шаг вперед
        # change_x > 0 = ускорение вправо увеличивается → нужен шаг вправо
        # change_x < 0 = ускорение влево увеличивается → нужен шаг влево
        
        direction = None
        step_x = 0
        step_y = 0
        change_magnitude = 0.0  # Величина изменения для расчета амплитуды
        
        # Приоритет: сначала вперед/назад, потом влево/вправо
        if abs(change_y) > abs(change_x):
            # Доминирует изменение вперед/назад
            if change_y < -self.accel_change_threshold:
                direction = 'backward'
                change_magnitude = abs(change_y)
                step_amplitude = self._calculate_dynamic_amplitude(change_magnitude, self.auto_step_amplitude_base, self.auto_step_amplitude_max)
                step_x = -step_amplitude  # x_move_amplitude: отрицательное = назад
            elif change_y > self.accel_change_threshold:
                direction = 'forward'
                change_magnitude = abs(change_y)
                step_amplitude = self._calculate_dynamic_amplitude(change_magnitude, self.auto_step_amplitude_base, self.auto_step_amplitude_max)
                step_x = step_amplitude  # x_move_amplitude: положительное = вперед
        else:
            # Доминирует изменение влево/вправо
            if change_x > self.accel_change_threshold:
                direction = 'right'
                change_magnitude = abs(change_x)
                step_amplitude = self._calculate_dynamic_amplitude(change_magnitude, self.auto_step_amplitude_lateral_base, self.auto_step_amplitude_max)
                step_y = step_amplitude  # y_move_amplitude: положительное = вправо
            elif change_x < -self.accel_change_threshold:
                direction = 'left'
                change_magnitude = abs(change_x)
                step_amplitude = self._calculate_dynamic_amplitude(change_magnitude, self.auto_step_amplitude_lateral_base, self.auto_step_amplitude_max)
                step_y = -step_amplitude  # y_move_amplitude: отрицательное = влево
        
        # Если ускорение слишком мало - сбрасываем счетчик
        if direction is None:
            if self.balance_step_count > 0:
                rospy.logdebug(f"✅ Auto-balance: Stable (ax={avg_x:.3f}, ay_dev={avg_y:.3f}, base={self.gravity_base_y:.3f}), resetting")
            self.balance_step_count = 0
            self.last_balance_direction = None
            self.last_stable_accel_y = avg_y_raw  # Сохраняем стабильное значение
            return
        
        # Проверяем, прошло ли достаточно времени с последнего шага
        # Первый шаг делаем быстрее
        step_interval = self.auto_step_interval_first if self.balance_step_count == 0 else self.auto_step_interval_second
        if current_time - self.last_auto_step_time < step_interval:
            return
        
        # Если направление изменилось, сбрасываем счетчик (робот качается)
        if self.last_balance_direction is not None and self.last_balance_direction != direction:
            rospy.loginfo(f"🔄 Auto-balance: Direction changed from {self.last_balance_direction} to {direction}, resetting counter")
            self.balance_step_count = 0
        
        # Выполняем шаг
        is_first_step = (self.balance_step_count == 0)
        step_type = "FAST LONG" if is_first_step else "SLOW SHORT"
        rospy.loginfo(f"🤖 Auto-balance: Robot tilting {direction} (change_x={change_x:.3f}, change_y={change_y:.3f}, magnitude={change_magnitude:.3f} m/s²), making {step_type} step ({self.balance_step_count + 1}/2)")
        self._make_auto_step_2d(step_x, step_y, fast=is_first_step)
        self.last_auto_step_time = current_time
        self.balance_step_count += 1
        self.last_balance_direction = direction

    def _calculate_dynamic_amplitude(self, change_magnitude, base_amplitude, max_amplitude):
        """
        Вычисляет динамическую амплитуду шага на основе величины ИЗМЕНЕНИЯ ускорения.
        Чем больше изменение, тем сильнее шаг.
        """
        if change_magnitude <= self.accel_change_threshold:
            return base_amplitude
        
        # Линейная интерполяция от базовой до максимальной амплитуды
        # От accel_change_threshold до максимального изменения (примерно 1.0 м/с²)
        max_change = 1.0  # Максимальное изменение для максимального шага
        if change_magnitude >= max_change:
            return max_amplitude
        
        ratio = (change_magnitude - self.accel_change_threshold) / (max_change - self.accel_change_threshold)
        return base_amplitude + (max_amplitude - base_amplitude) * ratio

    def _make_auto_step(self, backward=False, fast=False):
        """
        Выполняет автоматический шаг для балансировки (legacy, для совместимости).
        """
        step_x = -self.auto_step_amplitude_base if backward else self.auto_step_amplitude_base
        self._make_auto_step_2d(step_x, 0, fast)

    def _make_auto_step_2d(self, step_x, step_y, fast=False):
        """
        Выполняет автоматический шаг для балансировки в двух направлениях.
        step_x: шаг вперед/назад (положительный = вперед)
        step_y: шаг влево/вправо (положительный = вправо)
        fast=True делает шаг быстрее (уменьшает период времени).
        """
        try:
            gait_param = self.gait_manager.get_gait_param()
            params = self.speed_params[self.speed_mode]
            period_time = list(params['period_time'])
            
            # Ускоряем шаг для дополнительных шагов
            if fast:
                # Уменьшаем период времени на 30% для быстрых шагов
                period_time[0] = int(period_time[0] * 0.7)
                rospy.logdebug(f"   Fast step: period_time reduced to {period_time[0]}ms")
            
            if self.speed_mode > 1:
                gait_param.update(params['gait_base'])
            
            # Выполняем один шаг
            gait_param['init_z_offset'] = self.init_z_offset
            self.gait_manager.set_step(
                period_time, 
                step_x,  # x_move_amplitude (вперед/назад)
                step_y,  # y_move_amplitude (влево/вправо)
                0,  # angle_move_amplitude
                gait_param, 
                step_num=1  # Один шаг
            )
            
            step_type = "fast" if fast else "normal"
            direction_str = ""
            if abs(step_x) > 0.001:
                direction_str += f"{'forward' if step_x > 0 else 'backward'}"
            if abs(step_y) > 0.001:
                if direction_str:
                    direction_str += "+"
                direction_str += f"{'right' if step_y > 0 else 'left'}"
            
            rospy.loginfo(f"✅ Auto-balance step executed: {direction_str} ({step_type}, x={step_x:.4f}, y={step_y:.4f}, period: {period_time[0]}ms)")
        except Exception as e:
            rospy.logwarn(f"❌ Error making auto-balance step: {e}")

    def _handle_resonance_detection(self):
        """
        Обнаруживает резонанс на основе амплитуды качания и адаптирует параметры gait_manager.
        """
        try:
            with self.imu_lock:
                if len(self.imu_history['ax']) < 10:  # Нужно минимум данных для анализа
                    return
                
                # Расчет амплитуды качания по каждой оси
                # Используем стандартное отклонение как меру амплитуды
                def calculate_amplitude(data_list):
                    if len(data_list) < 2:
                        return 0.0
                    mean = sum(data_list) / len(data_list)
                    variance = sum((x - mean) ** 2 for x in data_list) / len(data_list)
                    return math.sqrt(variance)
                
                amplitude_x = calculate_amplitude(self.imu_history['ax'])
                amplitude_y = calculate_amplitude(self.imu_history['ay'])
                amplitude_z = calculate_amplitude(self.imu_history['az'])
                
                # Используем максимальную амплитуду из всех осей
                max_amplitude = max(amplitude_x, amplitude_y, amplitude_z)
                
                # Также учитываем амплитуду по pitch (наклон вперед/назад)
                amplitude_pitch = calculate_amplitude(self.imu_history['pitch'])
                
                # Комбинированная метрика амплитуды
                combined_amplitude = max(max_amplitude, abs(amplitude_pitch) * 2.0)
            
            # Адаптация параметров на основе амплитуды
            old_factor = self.current_adaptation_factor
            
            if combined_amplitude > self.critical_amplitude:
                # Критическая ситуация - резко уменьшаем шаг
                self.current_adaptation_factor = 0.5
                rospy.logwarn(f"⚠️ CRITICAL resonance detected! (amplitude={combined_amplitude:.3f} m/s²), reducing step by 50% (factor: {self.current_adaptation_factor:.2f})")
                rospy.loginfo(f"   Amplitudes - X: {amplitude_x:.3f}, Y: {amplitude_y:.3f}, Z: {amplitude_z:.3f}, Pitch: {amplitude_pitch:.3f} rad")
            elif combined_amplitude > self.max_safe_amplitude:
                # Высокая амплитуда - постепенно уменьшаем шаг
                # Линейная интерполяция от max_safe_amplitude до critical_amplitude
                ratio = (combined_amplitude - self.max_safe_amplitude) / (self.critical_amplitude - self.max_safe_amplitude)
                self.current_adaptation_factor = 1.0 - ratio * 0.4  # От 1.0 до 0.6
                rospy.loginfo(f"📊 High amplitude detected (amplitude={combined_amplitude:.3f} m/s²), adaptation factor: {self.current_adaptation_factor:.2f}")
                rospy.loginfo(f"   Amplitudes - X: {amplitude_x:.3f}, Y: {amplitude_y:.3f}, Z: {amplitude_z:.3f}, Pitch: {amplitude_pitch:.3f} rad")
            else:
                # Нормальная амплитуда - постепенно возвращаем к нормальным параметрам
                if self.current_adaptation_factor < 1.0:
                    self.current_adaptation_factor = min(1.0, self.current_adaptation_factor + 0.05)
                    if self.current_adaptation_factor != old_factor:
                        rospy.loginfo(f"✅ Recovering from resonance (amplitude={combined_amplitude:.3f} m/s²), adaptation factor: {self.current_adaptation_factor:.2f}")
            
            # Применяем адаптацию к параметрам gait_manager
            if abs(self.current_adaptation_factor - 1.0) > 0.01:  # Только если есть значительное изменение
                self._apply_gait_adaptation()
                
        except Exception as e:
            rospy.logwarn(f"❌ Error in resonance detection: {e}")

    def _log_imu_status(self, pitch, roll, yaw, ax, ay, az, gx, gy, gz):
        """
        Логирует текущее состояние IMU для отладки.
        """
        try:
            with self.imu_lock:
                history_size = len(self.imu_history['pitch'])
            
            # Вычисляем сглаженные значения
            with self.imu_lock:
                if history_size >= 5:
                    recent_pitch = self.imu_history['pitch'][-5:]
                    smoothed_pitch = sum(recent_pitch) / len(recent_pitch)
                else:
                    smoothed_pitch = pitch
            
            rospy.loginfo("=" * 60)
            rospy.loginfo("📡 IMU Status Report")
            rospy.loginfo(f"   Orientation:")
            rospy.loginfo(f"      Pitch: {pitch:.3f} rad ({math.degrees(pitch):.1f}°)")
            rospy.loginfo(f"      Roll:  {roll:.3f} rad ({math.degrees(roll):.1f}°)")
            rospy.loginfo(f"      Yaw:   {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
            rospy.loginfo(f"   Acceleration: X={ax:.2f}, Y={ay:.2f}, Z={az:.2f} m/s²")
            rospy.loginfo(f"   Angular Velocity (Gyro): X={gx:.4f}, Y={gy:.4f}, Z={gz:.4f} rad/s")
            rospy.loginfo(f"   Smoothed Pitch: {smoothed_pitch:.3f} rad ({math.degrees(smoothed_pitch):.1f}°)")
            
            # Средние ускорения из истории
            avg_x = sum(self.accel_history['x']) / len(self.accel_history['x']) if len(self.accel_history['x']) > 0 else 0.0
            avg_y_raw = sum(self.accel_history['y']) / len(self.accel_history['y']) if len(self.accel_history['y']) > 0 else 0.0
            avg_y_deviation = avg_y_raw - 9.8  # Отклонение от гравитации
            
            rospy.loginfo(f"   Average Acceleration: X={avg_x:.3f} (left/right), Y_raw={avg_y_raw:.3f}, Y_deviation={avg_y_deviation:.3f} (forward/back) m/s²")
            rospy.loginfo(f"   Gravity base (calibrated): {self.gravity_base_y:.3f} m/s², calibrated={self.gravity_calibrated}")
            rospy.loginfo(f"   Noise levels: X={self.noise_level_x:.3f}, Y={self.noise_level_y:.3f} m/s²")
            rospy.loginfo(f"   Calibration status: {'COMPLETE' if self.gravity_calibrated else 'IN PROGRESS'}")
            rospy.loginfo(f"   Robot State: {self.robot_state}, Movement Status: {self.status}")
            rospy.loginfo(f"   Update Param: {self.update_param}, Move Amplitudes: X={self.x_move_amplitude:.4f}, Y={self.y_move_amplitude:.4f}")
            
            rospy.loginfo(f"   Auto-balance: enabled={self.auto_balance_enabled}, step_count={self.balance_step_count}/2")
            rospy.loginfo(f"   Last direction: {self.last_balance_direction}")
            rospy.loginfo(f"   Change threshold: {self.accel_change_threshold:.3f} m/s², min: {self.accel_change_min_threshold:.3f} m/s²")
            fall_duration = rospy.get_time() - self.fall_start_time if self.fall_start_time else 0
            rospy.loginfo(f"   Auto-getup: enabled={self.auto_getup_enabled}, fall_duration={fall_duration:.1f}s (threshold: {self.fall_time_threshold}s)")
            rospy.loginfo(f"   Resonance: enabled={self.resonance_detection_enabled}, adaptation_factor={self.current_adaptation_factor:.2f}")
            rospy.loginfo(f"   IMU History Size: {history_size}/{self.imu_history_size}, Accel History: X={len(self.accel_history['x'])}, Y={len(self.accel_history['y'])}")
            rospy.loginfo("=" * 60)
        except Exception as e:
            rospy.logwarn(f"Error logging IMU status: {e}")

    def _apply_gait_adaptation(self):
        """
        Применяет адаптацию параметров gait_manager на основе current_adaptation_factor.
        Вызывается из _handle_resonance_detection для динамической адаптации во время движения.
        """
        try:
            if not self.update_param or self.status != 'move':  # Только если робот движется
                return
            
            gait_param = self.gait_manager.get_gait_param()
            params = self.speed_params[self.speed_mode]
            period_time = list(params['period_time'])
            
            if self.speed_mode > 1:
                gait_param.update(params['gait_base'])
            
            # Применяем адаптацию к амплитудам движения
            adapted_x_amp = self.x_move_amplitude * self.current_adaptation_factor
            adapted_y_amp = self.y_move_amplitude * self.current_adaptation_factor
            adapted_angle_amp = self.angle_move_amplitude * self.current_adaptation_factor
            
            # Также адаптируем период времени (увеличить для большей стабильности)
            if self.current_adaptation_factor < 0.8:
                # Увеличиваем период времени шага для большей стабильности
                period_time[0] = int(period_time[0] * (1.0 + (1.0 - self.current_adaptation_factor) * 0.3))
            
            # Применяем адаптированные параметры
            gait_param['init_z_offset'] = self.init_z_offset
            self.gait_manager.set_step(
                period_time,
                adapted_x_amp,
                adapted_y_amp,
                adapted_angle_amp,
                gait_param,
                step_num=0
            )
            
        except Exception as e:
            rospy.logwarn(f"Error applying gait adaptation: {e}")

    def axes_callback(self, axes):
        """Calculates and sets walking parameters based on speed mode and joystick input."""
        # Проверка разрешения на движение
        if not self.can_move():
            if self.status == 'move':
                self.status = 'stop'
                self.gait_manager.stop()
                rospy.logwarn("Movement command blocked - no permission")
            return
        
        self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude = 0, 0, 0
        self.update_param = False
        
        gait_param = self.gait_manager.get_gait_param()
        params = self.speed_params[self.speed_mode]
        period_time = list(params['period_time'])
        
        if self.speed_mode > 1:
            gait_param.update(params['gait_base'])

        # Conditional logic from kinematics, now simplified
        if self.speed_mode == 1 and abs(axes['lx']) > 0.3: period_time[2] = 0.025
        elif self.speed_mode == 2:
            if abs(axes['rx']) > 0.3 and abs(axes['lx']) < 0.3 and abs(axes['ly']) < 0.3: gait_param.update({'init_roll_offset': 0, 'init_y_offset': -0.005, 'y_swap_amplitude': 0.022})
            if abs(axes['lx']) > 0.3 and abs(axes['ly']) < 0.3 and abs(axes['rx']) < 0.3: gait_param.update({'init_y_offset': -0.005, 'y_swap_amplitude': 0.028, 'init_roll_offset': 3})
        elif self.speed_mode == 3:
            if abs(axes['rx']) > 0.3 and abs(axes['lx']) < 0.3 and abs(axes['ly']) < 0.3: gait_param['y_swap_amplitude'] = 0.022
            if abs(axes['lx']) > 0.3 and abs(axes['ly']) < 0.3 and abs(axes['rx']) < 0.3: gait_param.update({'init_y_offset': 0, 'init_roll_offset': 3, 'y_swap_amplitude': 0.025})
        elif self.speed_mode == 4:
            if abs(axes['ly']) > 0.3: gait_param['init_roll_offset'] = -3
            if abs(axes['lx']) > 0.3 and abs(axes['ly']) < 0.3 and abs(axes['rx']) < 0.3: gait_param['init_roll_offset'] = -1.0
            if abs(axes['rx']) > 0.3 and abs(axes['lx']) < 0.3 and abs(axes['ly']) < 0.3: gait_param['init_roll_offset'] = -3

        if abs(axes['ly']) > 0.3: self.x_move_amplitude = math.copysign(params['x_amp'], axes['ly'])
        if abs(axes['lx']) > 0.3: self.y_move_amplitude = math.copysign(params['y_amp'], axes['lx'])
        if abs(axes['rx']) > 0.3: self.angle_move_amplitude = math.copysign(params['angle_amp'], axes['rx'])
        
        self.update_param = any(abs(amp) > 0 for amp in [self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude])

        if self.update_param:
            gait_param['init_z_offset'] = self.init_z_offset
            
            # Применяем адаптацию на основе резонанса, если она активна
            if self.resonance_detection_enabled and abs(self.current_adaptation_factor - 1.0) > 0.01:
                adapted_x_amp = self.x_move_amplitude * self.current_adaptation_factor
                adapted_y_amp = self.y_move_amplitude * self.current_adaptation_factor
                adapted_angle_amp = self.angle_move_amplitude * self.current_adaptation_factor
                
                # Также адаптируем период времени для большей стабильности при резонансе
                adapted_period_time = list(period_time)
                if self.current_adaptation_factor < 0.8:
                    adapted_period_time[0] = int(period_time[0] * (1.0 + (1.0 - self.current_adaptation_factor) * 0.3))
                
                self.gait_manager.set_step(
                    adapted_period_time,
                    adapted_x_amp,
                    adapted_y_amp,
                    adapted_angle_amp,
                    gait_param,
                    step_num=0
                )
            else:
                # Нормальная установка параметров без адаптации
                self.gait_manager.set_step(period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, gait_param, step_num=0)
        
        if self.status == 'stop' and self.update_param: self.status = 'move'
        elif self.status == 'move' and not self.update_param:
            self.status = 'stop'
            self.gait_manager.stop()
            # Сбрасываем адаптацию при остановке
            self.current_adaptation_factor = 1.0

    def height_callback(self, axes):
        """Handles height adjustment."""
        if rospy.get_time() > self.time_stamp_ry:
            if abs(axes['ry']) > 0.5:
                self.init_z_offset -= 0.005 * math.copysign(1, axes['ry'])
                self.init_z_offset = max(0.025, min(0.06, self.init_z_offset))
                self.update_height = True
            
            if self.update_height and not self.update_param:
                gait_param = self.gait_manager.get_gait_param()
                gait_param['body_height'] = self.init_z_offset
                self.gait_manager.update_param(self.speed_params[1]['period_time'], 0, 0, 0, gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05
                self.update_height = False

    # --- BUTTON CALLBACKS ---
    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.speed_mode < 4:
            self.speed_mode += 1
            rospy.loginfo(f"Speed mode: {self.speed_mode}")
            self.board.set_buzzer(2000, 0.1, 0.05, 1)

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.speed_mode > 1:
            self.speed_mode -= 1
            rospy.loginfo(f"Speed mode: {self.speed_mode}")
            self.board.set_buzzer(1500, 0.1, 0.05, 1)

    def cross_callback(self, new_state):
        # Проверка разрешения на стрельбу
        if not self.can_fire():
            if new_state == ButtonState.Pressed:
                rospy.logwarn("Fire command blocked - no permission")
            return
        
        ser = self.get_serial_port()
        if ser is None or not ser.is_open:
            rospy.logwarn("Cannot send firing command: Arduino not connected.")
            return
        
        try:
            if new_state == ButtonState.Pressed:
                ser.write(b"FIRE\n")
                self.is_firing = True
                self.firing_state_pub.publish(True)
                rospy.loginfo("Firing started")
            elif new_state == ButtonState.Released:
                ser.write(b"STOP\n")
                self.is_firing = False
                self.firing_state_pub.publish(False)
                rospy.loginfo("Firing stopped")
        except serial.SerialException as e:
            rospy.logwarn(f"Failed to send firing command to Arduino: {e}")
            # Mark serial as disconnected, reconnection timer will handle it
            with self.serial_lock:
                if self.serial_port is not None:
                    try:
                        if self.serial_port.is_open:
                            self.serial_port.close()
                    except:
                        pass
                    self.serial_port = None

    def x_button_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.sound: self.sound.play()

    def square_callback(self, new_state):
        duration = 0.5
        if new_state == ButtonState.Pressed:
            self.board.bus_servo_set_position(duration, [[13, 253], [15, 880], [17, 480], [19, 520]])
        elif new_state == ButtonState.Released:
            self.board.bus_servo_set_position(duration, [[13, 707], [15, 840], [17, 480], [19, 133]])
    
    # NEW: Circle Callback from joystick_control 1.py
    def circle_callback(self, new_state):
        """
        Executes "Get Up" action.
        Automatically selects between lie_to_stand and recline_to_stand
        based on state determined via IMU.
        """
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle (B) button pressed. Current IMU robot state: '{self.robot_state}'.")
            
            # Added new check
            if self.robot_state == 'stand':
                rospy.loginfo("Robot is already in a 'stand' state. No get-up action will be performed.")
                self.board.set_buzzer(1500, 0.1, 0.05, 1)
                return

            action_to_run = None
            log_msg = ""
            
            if self.robot_state == 'lie_to_stand':
                action_to_run = self.lie_to_stand_action_name
                log_msg = "Robot state indicates lying on FRONT. Initiating lie_to_stand."
            elif self.robot_state == 'recline_to_stand':
                action_to_run = self.recline_to_stand_action_name
                log_msg = "Robot state indicates lying on BACK. Initiating recline_to_stand."

            rospy.loginfo(log_msg)
            self.board.set_buzzer(1500, 0.1, 0.05, 1)

            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated successfully.")
                    
                    self.robot_state = 'stand'
                    self.count_lie = 0
                    self.count_recline = 0
                    rospy.logdebug("Robot state set to 'stand' and IMU counters reset after Get Up command.")
                    
                else:
                    rospy.logwarn("MotionManager not initialized or no action specified.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action('{action_to_run}'): {e}")

    def start_callback(self, new_state):
        """Resets the robot's height to default. (Code from your request)."""
        if new_state == ButtonState.Pressed:
            rospy.loginfo("Start button pressed. Resetting body height.")
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            gait_param = self.gait_manager.get_gait_param()
            t = int(abs(0.025 - self.init_z_offset) / 0.005)
            if t != 0:
                direction = math.copysign(1, 0.025 - self.init_z_offset)
                for _ in range(t):
                    self.init_z_offset += 0.005 * direction
                    gait_param['body_height'] = self.init_z_offset
                    # Use the refactored speed_params dictionary
                    params = self.speed_params[self.speed_mode]
                    gait_param['z_move_amplitude'] = params['z_move_amplitude']
                    self.gait_manager.update_param(params['period_time'], 0.0, 0.0, 0.0, gait_param, step_num=1)
                    time.sleep(0.05)

    # --- Unused Button Callbacks ---
    def select_callback(self, new_state): pass
    def triangle_callback(self, new_state): pass
    def l2_callback(self, new_state): pass
    def r2_callback(self, new_state): pass
    def hat_xl_callback(self, new_state): pass
    def hat_xr_callback(self, new_state): pass
    def hat_yd_callback(self, new_state): pass
    def hat_yu_callback(self, new_state): pass

    # --- Main Joy Message Processor ---
    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        axes_changed = any(self.last_axes[key] != value for key, value in axes.items())
        
        if axes_changed:
            try:
                self.axes_callback(axes)
                self.height_callback(axes)
            except Exception as e:
                rospy.logerr(f"Error in axes processing: {str(e)}")

        for key, value in buttons.items():
            if not key: continue
            
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
                callback_name = f"{key}_callback"
                if hasattr(self, callback_name):
                    try:
                        getattr(self, callback_name)(new_state)
                    except Exception as e:
                        rospy.logerr(f"Error in button callback '{callback_name}': {str(e)}")

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    try:
        node = JoystickController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"An error occurred in the main execution block: {str(e)}")