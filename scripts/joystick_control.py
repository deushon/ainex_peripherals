#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import serial
import threading
import os
import math
import json
# pygame импортируется внутри функции для обработки ошибок
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import Trigger, TriggerResponse

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

        # --- NEW: Refactored speed parameters into a dictionary ---
        self.setup_speed_parameters()

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
        Processes IMU data to determine fall state.
        Logic adapted from mobile app example, using ay and az.
        """
        try:
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            ACCEL_THRESH = 7.0
            ANGLE_THRESH = 30.0
            COUNT_INCREMENT = 1
            COUNT_DECREMENT = 1

            if abs(az) > 1e-6:
                angle_rad = math.atan2(abs(ay), abs(az))
                angle_deg = math.degrees(angle_rad)
            else:
                angle_deg = 90.0

            if angle_deg < ANGLE_THRESH:
                if az > ACCEL_THRESH:
                    self.count_lie += COUNT_INCREMENT
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
                elif az < -ACCEL_THRESH:
                    self.count_recline += COUNT_INCREMENT
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                else:
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
            else:
                self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)

            old_state = self.robot_state

            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'lie_to_stand'
            elif self.count_recline > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'recline_to_stand'
            else:
                self.robot_state = 'stand'

            if old_state != self.robot_state:
                rospy.loginfo(f"IMU detected robot state change: '{old_state}' -> '{self.robot_state}' (lie_count:{self.count_lie}, recline_count:{self.count_recline})")

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data in imu_callback: {e}")

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
            self.gait_manager.set_step(period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, gait_param, step_num=0)
        
        if self.status == 'stop' and self.update_param: self.status = 'move'
        elif self.status == 'move' and not self.update_param:
            self.status = 'stop'
            self.gait_manager.stop()

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