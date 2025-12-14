#!/usr/bin/env python3
# encoding: utf-8
"""
Рефакторенная версия joystick_control.py с модульной архитектурой.
Все конфиги вынесены в начало модулей, дублирование кода удалено.
"""

import sys
import os

# Добавляем путь к модулям в sys.path для корректного импорта
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import time
import rospy
import serial
import threading
import json
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import Trigger, TriggerResponse

# Импорт модулей
from auto_stabilization import AutoStabilization
from game_services import GameServices
from speed_control import SpeedControl
from button_actions import ButtonActions
from imu_handler import IMUHandler

# ========== КОНФИГУРАЦИЯ ==========
# Константы джойстика
AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', 'x_button', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

# Serial порт
SERIAL_RECONNECT_INTERVAL = 5.0  # Интервал проверки переподключения (сек)
SERIAL_READ_SLEEP = 0.01  # Задержка чтения serial (сек)
SERIAL_ERROR_SLEEP = 1.0  # Задержка при ошибке serial (сек)
SERIAL_MAX_CONSECUTIVE_ERRORS = 10  # Максимальное количество последовательных ошибок

# Инициализация
INIT_Z_OFFSET = 0.025  # Начальное смещение по Z
INIT_DELAY = 0.2  # Задержка после инициализации (сек)

# Путь к action groups (относительно корня пакета)
ACTION_GROUPS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'action_groups')
# ===================================

class ButtonState:
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

# --- Serial Reader Thread ---
class SerialReader(threading.Thread):
    def __init__(self, ser_getter, hit_detection_pub_instance, robot_id):
        super().__init__()
        self.ser_getter = ser_getter
        self.hit_detection_pub = hit_detection_pub_instance
        self.robot_id = robot_id
        self.running = True
        self.consecutive_errors = 0
        self.max_consecutive_errors = SERIAL_MAX_CONSECUTIVE_ERRORS

    def run(self):
        rospy.loginfo("Serial port reading thread started.")
        while self.running and not rospy.is_shutdown():
            try:
                ser = self.ser_getter()
                if ser is None or not ser.is_open:
                    time.sleep(SERIAL_ERROR_SLEEP)
                    continue
                
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        rospy.loginfo(f"Received from Arduino: '{line}'")
                        if line == "HIT":
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
                    self.consecutive_errors = 0
            except serial.SerialException as e:
                self.consecutive_errors += 1
                if self.consecutive_errors >= self.max_consecutive_errors:
                    rospy.logwarn(f"Serial port error in reading thread (attempt {self.consecutive_errors}): {e}")
                    rospy.logwarn("Serial port appears disconnected. Will retry when connection is restored.")
                    self.consecutive_errors = 0
                time.sleep(SERIAL_ERROR_SLEEP)
            except AttributeError:
                time.sleep(SERIAL_ERROR_SLEEP)
            except Exception as e:
                rospy.logwarn(f"Unexpected error in serial reading thread: {e}")
                time.sleep(SERIAL_ERROR_SLEEP / 2)
            else:
                time.sleep(SERIAL_READ_SLEEP)
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
        
        # Инициализация MotionManager с путем к action groups из ноды
        self.action_groups_dir = ACTION_GROUPS_DIR
        if os.path.exists(self.action_groups_dir):
            rospy.loginfo(f"Action groups directory: {self.action_groups_dir}")
        else:
            rospy.logwarn(f"Action groups directory not found: {self.action_groups_dir}. Creating it...")
            os.makedirs(self.action_groups_dir, exist_ok=True)
        
        # Устанавливаем ROS параметр для MotionManager, если он его использует
        rospy.set_param('~action_groups_path', self.action_groups_dir)
        
        self.motion_manager = MotionManager()
        
        # Обертка для run_action с поддержкой путей из нашей ноды
        self._original_run_action = self.motion_manager.run_action
        self.motion_manager.run_action = self._run_action_with_local_path
        
        # --- State Variables ---
        self.status = 'stop'
        self.time_stamp_ry = 0
        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP)))
        
        # --- Robot ID ---
        self.robot_id = rospy.get_param('~robot_id', 'robot_1')
        
        # --- Serial Port Configuration ---
        self.serial_port = None
        self.serial_port_name = rospy.get_param('~port', '/dev/ttyUSB0')
        self.serial_baudrate = rospy.get_param('~baudrate', 9600)
        self.serial_lock = threading.Lock()
        
        # --- Initialize Modules ---
        self.game_services = GameServices(self.robot_id)
        self.speed_control = SpeedControl(self.gait_manager)
        self.imu_handler = IMUHandler()
        
        # Инициализируем высоту через единый метод (после создания speed_control)
        self.speed_control.set_body_height(INIT_Z_OFFSET)
        
        # Инициализируем переменные для движения (нужны для автостабилизации)
        self.x_move_amplitude = 0
        self.y_move_amplitude = 0
        self.angle_move_amplitude = 0
        
        # Получаем начальную высоту из gait_manager (единый источник истины)
        initial_height = self.speed_control.get_body_height()
        self.auto_stabilization = AutoStabilization(
            self.gait_manager,
            self.speed_control.get_speed_params(),
            self.speed_control.get_speed_mode(),
            initial_height
        )
        # Устанавливаем ссылку на auto_stabilization в speed_control для применения корректировок
        self.speed_control.set_auto_stabilization(self.auto_stabilization)
        self.button_actions = ButtonActions(
            self.board,
            self.gait_manager,
            self.speed_control,
            self.motion_manager,
            self.game_services,
            self.get_serial_port,
            lambda: self.imu_handler.get_robot_state()
        )
        
        # --- Setup Serial and Game Features ---
        self.setup_serial_and_game()
        
        # --- Start ROS Subscribers ---
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        rospy.loginfo("Joystick Controller Initialized (Refactored)")
        time.sleep(INIT_DELAY)

    def setup_serial_and_game(self):
        """Инициализация Serial порта и игровых функций."""
        # Подключение к serial порту
        self.reconnect_serial()
        
        # Публикаторы для игры
        self.hit_detection_pub = rospy.Publisher('/game/hit_detection', String, queue_size=10)
        self.firing_state_pub = rospy.Publisher('/game/firing_state', Bool, queue_size=10)
        
        # Инициализация потока чтения serial
        self.serial_reader_thread = SerialReader(self.get_serial_port, self.hit_detection_pub, self.robot_id)
        self.serial_reader_thread.daemon = True
        self.serial_reader_thread.start()
        rospy.on_shutdown(self.serial_reader_thread.stop)
        # Гарантируем остановку робота при завершении
        rospy.on_shutdown(self._shutdown_handler)
        
        # Сервис проверки здоровья
        self.health_service = rospy.Service('/game/robot_health', Trigger, self.health_check_service)
        
        # Таймер для проверки переподключения serial
        self.serial_reconnect_timer = rospy.Timer(rospy.Duration(SERIAL_RECONNECT_INTERVAL), self.check_and_reconnect_serial)

    def get_serial_port(self):
        """Thread-safe метод для получения serial порта."""
        with self.serial_lock:
            return self.serial_port

    def reconnect_serial(self):
        """Попытка переподключения к Arduino serial порту."""
        with self.serial_lock:
            if self.serial_port is not None and self.serial_port.is_open:
                try:
                    self.serial_port.in_waiting
                    return True
                except:
                    try:
                        self.serial_port.close()
                    except:
                        pass
                    self.serial_port = None
            
            if self.serial_port is not None:
                try:
                    if self.serial_port.is_open:
                        self.serial_port.close()
                except Exception as e:
                    rospy.logdebug(f"Error closing serial port: {e}")
                self.serial_port = None
            
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
        """Периодически проверяет serial соединение и пытается переподключиться."""
        ser = self.get_serial_port()
        if ser is None or not ser.is_open:
            self.reconnect_serial()

    def health_check_service(self, req):
        """Обработчик сервиса проверки здоровья робота."""
        response = TriggerResponse()
        try:
            ser = self.get_serial_port()
            arduino_connected = ser is not None and ser.is_open if ser else False
            
            if self.game_services.use_detailed_permissions:
                locked = not (self.game_services.permissions['movement'] or self.game_services.permissions['firing'])
            else:
                locked = self.game_services.is_locked
            
            health_data = {
                'available': True,
                'hp': self.game_services.current_hp,
                'locked': locked,
                'connected': arduino_connected,
                'robot_id': self.robot_id
            }
            response.success = True
            response.message = json.dumps(health_data)
            rospy.logdebug(f"Health check requested. Response: {response.message}")
        except Exception as e:
            response.success = False
            response.message = f"Error during health check: {str(e)}"
            rospy.logerr(response.message)
        return response

    def imu_callback(self, msg: Imu):
        """Обработчик данных IMU."""
        # Обработка IMU данных
        imu_data = self.imu_handler.process_imu(msg)
        if imu_data is None:
            return
        
        # Обновление автостабилизации
        if self.auto_stabilization.enabled and self.imu_handler.get_robot_state() == 'stand':
            # Обрабатываем стабилизацию с новыми данными IMU
            self.auto_stabilization.process(
                imu_data,
                self.imu_handler.get_robot_state(),
                self.status,
                self.x_move_amplitude,
                self.y_move_amplitude,
                self.angle_move_amplitude
            )
        
        # Проверка автоматического подъема
        if self.imu_handler.auto_getup_enabled:
            self.imu_handler.check_auto_getup(
                self.motion_manager,
                self.game_services.can_move,
                'lie_to_stand',
                'BACK_UP',
                'LEFT_UP',
                'RIGHT_UP'
            )

    def axes_callback(self, axes):
        """Обработчик осей джойстика."""
        if not self.game_services.can_move():
            if self.status == 'move':
                self.status = 'stop'
                self.gait_manager.stop()
                rospy.logwarn("Movement command blocked - no permission")
            return
        
        # Обновляем speed_mode в auto_stabilization при изменении
        current_speed_mode = self.speed_control.get_speed_mode()
        if self.auto_stabilization.speed_mode != current_speed_mode:
            self.auto_stabilization.speed_mode = current_speed_mode
            self.auto_stabilization.speed_params = self.speed_control.get_speed_params()
        
        # Синхронизируем высоту из gait_manager в auto_stabilization
        current_height = self.speed_control.get_body_height()
        if abs(self.auto_stabilization.init_z_offset - current_height) > 0.001:
            self.auto_stabilization.init_z_offset = current_height
        
        # Обрабатываем оси через speed_control (высота берется из gait_manager)
        # Фактор адаптации резонанса удален - больше не используется
        x_move_amp, y_move_amp, angle_move_amp, status = self.speed_control.process_axes(
            axes,
            1.0  # Фактор адаптации всегда 1.0 (резонанс больше не обрабатывается)
        )
        
        self.x_move_amplitude = x_move_amp
        self.y_move_amplitude = y_move_amp
        self.angle_move_amplitude = angle_move_amp
        self.status = status
    
    def _update_height_in_all_modules(self, height):
        """
        Обновляет высоту во всех модулях через единый источник истины.
        
        Args:
            height: Новая высота
        """
        # Устанавливаем через speed_control (единый источник)
        self.speed_control.set_body_height(height)
        # Синхронизируем с auto_stabilization
        self.auto_stabilization.init_z_offset = self.speed_control.get_body_height()

    def height_callback(self, axes):
        """Обработчик изменения высоты."""
        new_height, new_timestamp, updated = self.speed_control.process_height(
            axes,
            self.time_stamp_ry
        )
        if updated and new_height is not None:
            self.time_stamp_ry = new_timestamp
            # Обновляем высоту в auto_stabilization (синхронизация через gait_manager)
            self.auto_stabilization.init_z_offset = new_height

    def joy_callback(self, joy_msg):
        """Главный обработчик сообщений джойстика."""
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
            if not key:
                continue
            
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
                callback_name = f"{key}_callback"
                if hasattr(self.button_actions, callback_name):
                    try:
                        if callback_name == 'start_callback':
                            # Специальная обработка для start_callback
                            # Высота теперь управляется через speed_control (единый источник истины)
                            self.button_actions.start_callback(
                                new_state,
                                lambda: self.speed_control.get_body_height(),
                                lambda h: self._update_height_in_all_modules(h)
                            )
                        else:
                            getattr(self.button_actions, callback_name)(new_state)
                    except Exception as e:
                        rospy.logerr(f"Error in button callback '{callback_name}': {str(e)}")

        self.last_buttons = buttons
        self.last_axes = axes
    
    def _run_action_with_local_path(self, action_name):
        """
        Обертка для run_action, которая ищет action groups в локальной директории ноды.
        
        Args:
            action_name: Имя action group
        """
        # Сначала пробуем найти в локальной директории action_groups
        # Проверяем разные возможные расширения файлов (включая .d6a для action groups)
        possible_extensions = ['.d6a', '', '.json', '.yaml', '.yml', '.txt']
        local_action_path = None
        
        for ext in possible_extensions:
            test_path = os.path.join(self.action_groups_dir, action_name + ext)
            if os.path.exists(test_path):
                local_action_path = test_path
                break
        
        # Если файл существует в локальной директории, используем его
        if local_action_path:
            rospy.loginfo(f"Using local action group: {local_action_path}")
            # Сохраняем текущую рабочую директорию
            old_cwd = os.getcwd()
            try:
                # Переходим в директорию action_groups для загрузки
                os.chdir(self.action_groups_dir)
                # Вызываем оригинальный метод с именем (MotionManager будет искать в текущей директории)
                return self._original_run_action(action_name)
            finally:
                # Восстанавливаем рабочую директорию
                os.chdir(old_cwd)
        else:
            # Если файла нет в локальной директории, используем оригинальный метод
            # MotionManager будет искать в стандартных местах
            rospy.logdebug(f"Action group '{action_name}' not found in local directory ({self.action_groups_dir}), using default search")
            return self._original_run_action(action_name)
    
    def _shutdown_handler(self):
        """Обработчик завершения работы - гарантирует остановку робота."""
        try:
            rospy.loginfo("Shutting down joystick controller...")
            # Останавливаем робота
            self.gait_manager.stop()
            # Сбрасываем корректировки стабилизации
            if hasattr(self, 'auto_stabilization') and self.auto_stabilization is not None:
                self.auto_stabilization.reset_walking_corrections()
            rospy.loginfo("Robot stopped on shutdown")
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")

if __name__ == "__main__":
    node = None
    try:
        node = JoystickController()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    except Exception as e:
        rospy.logerr(f"An error occurred in the main execution block: {str(e)}")
    finally:
        # Гарантируем остановку робота при завершении
        if node is not None:
            try:
                node.gait_manager.stop()
                rospy.loginfo("Robot stopped on shutdown")
            except:
                pass

