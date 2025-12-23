#!/usr/bin/env python3
# encoding: utf-8
"""
Главный контроллер джойстика.
Объединяет все модули управления роботом.
"""

import os
import time
import json
import rospy
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import Trigger, TriggerResponse

from config import ConfigLoader
from game import GameServices
from imu import IMUDataHandler
from robot_state import RobotStateManager, FallDetector, AutoGetup
from walking import SpeedControl, StabilizationModule, IMUData
from control.button_handler import ButtonHandler
from control.serial_handler import SerialHandler


class JoystickController:
    """
    Главный контроллер джойстика.
    """
    
    def __init__(self):
        """Инициализация контроллера."""
        rospy.init_node('joystick_control', anonymous=True)
        
        self.config = ConfigLoader()
        joystick_config = self.config.load('joystick')
        init_config = joystick_config.get('initialization', {})
        
        self.axes_map = tuple(joystick_config.get('joystick', {}).get('axes_map', []))
        self.button_map = tuple(joystick_config.get('joystick', {}).get('button_map', []))
        
        self.action_groups_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'action_groups'
        )
        if not os.path.exists(self.action_groups_dir):
            os.makedirs(self.action_groups_dir, exist_ok=True)
        
        rospy.set_param('~action_groups_path', self.action_groups_dir)
        
        self.board = Board()
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager()
        
        self._original_run_action = self.motion_manager.run_action
        self.motion_manager.run_action = self._run_action_with_local_path
        
        self.status = 'stop'
        self.time_stamp_ry = 0.0
        self.last_axes = dict(zip(self.axes_map, [0.0] * len(self.axes_map)))
        self.last_buttons = dict(zip(self.button_map, [0.0] * len(self.button_map)))
        
        game_config = self.config.load('game')
        game_params = game_config.get('game', {})
        serial_config = self.config.load('serial')
        serial_params = serial_config.get('serial', {})
        
        self.robot_id = game_params.get('robot_id', 'robot_1')
        serial_port_name = serial_params.get('port', '/dev/ttyUSB0')
        serial_baudrate = serial_params.get('baudrate', 9600)
        
        self.hit_detection_pub = rospy.Publisher('/game/hit_detection', String, queue_size=10)
        self.firing_state_pub = rospy.Publisher('/game/firing_state', Bool, queue_size=10)
        
        self.serial_handler = SerialHandler(
            serial_port_name, serial_baudrate, self.robot_id,
            self.hit_detection_pub, self.config
        )
        
        self.game_services = GameServices(self.robot_id, self.config)
        
        stabilization_module = StabilizationModule(self.config)
        rospy.loginfo(f"StabilizationModule initialized: enabled={stabilization_module.is_enabled()}")
        self.speed_control = SpeedControl(
            self.gait_manager, self.config, stabilization_module
        )
        rospy.loginfo(f"SpeedControl initialized with stabilization: enabled={self.speed_control.get_stabilization_module().is_enabled()}")
        
        init_z_offset = init_config.get('z_offset', 0.025)
        self.speed_control.set_body_height(init_z_offset)
        
        self.imu_data_handler = IMUDataHandler()
        self.robot_state_manager = RobotStateManager()
        self.fall_detector = FallDetector(self.config)
        self.auto_getup = AutoGetup(self.config)
        
        self.fall_check_cooldown = 0.0
        imu_config = self.config.load('imu')
        auto_getup_config = imu_config.get('imu', {}).get('auto_getup', {})
        self.fall_check_cooldown_duration = auto_getup_config.get('cooldown_duration', 5.0)
        
        self.last_imu_data = None
        self.x_move_amplitude = 0.0
        self.y_move_amplitude = 0.0
        self.angle_move_amplitude = 0.0
        
        # Watchdog для автоматической остановки при отсутствии команд
        self.last_command_time = rospy.get_time()
        self.command_timeout = 0.3  # Таймаут в секундах (300 мс)
        
        # Кэш больше не нужен - стабилизация применяется только в process_axes()
        
        self.button_handler = ButtonHandler(
            self.board,
            self.gait_manager,
            self.speed_control,
            self.motion_manager,
            self.game_services,
            self.serial_handler.get_port,
            lambda: self.robot_state_manager.get_state(),
            self.firing_state_pub,
            self.config
        )
        
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        # queue_size=1 для минимальной задержки (обрабатываем только последние данные)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        
        self.health_service = rospy.Service('/game/robot_health', Trigger, self.health_check_service)
        
        rospy.Timer(
            rospy.Duration(self.serial_handler.reconnect_interval),
            self._check_and_reconnect_serial
        )
        
        # Watchdog таймер для проверки команд движения (проверка каждые 0.1 секунды)
        rospy.Timer(rospy.Duration(0.1), self._watchdog_check)
        
        rospy.on_shutdown(self._shutdown_handler)
        
        init_delay = init_config.get('delay', 0.2)
        rospy.loginfo("Joystick Controller Initialized")
        time.sleep(init_delay)
    
    def _run_action_with_local_path(self, action_name):
        """
        Обертка для run_action, которая ищет action groups в локальной директории.
        
        Args:
            action_name: Имя action group
        """
        possible_extensions = ['.d6a', '', '.json', '.yaml', '.yml', '.txt']
        local_action_path = None
        
        for ext in possible_extensions:
            test_path = os.path.join(self.action_groups_dir, action_name + ext)
            if os.path.exists(test_path):
                local_action_path = test_path
                break
        
        if local_action_path:
            rospy.loginfo(f"Using local action group: {local_action_path}")
            old_cwd = os.getcwd()
            try:
                os.chdir(self.action_groups_dir)
                return self._original_run_action(action_name)
            finally:
                os.chdir(old_cwd)
        else:
            rospy.logdebug(f"Action group '{action_name}' not found in local directory, using default search")
            return self._original_run_action(action_name)
    
    def _check_and_reconnect_serial(self, event):
        """Периодически проверяет serial соединение и пытается переподключиться."""
        if not self.serial_handler.is_connected():
            self.serial_handler.reconnect()
    
    def _watchdog_check(self, event):
        """
        Watchdog проверка: останавливает робота, если команды не приходят слишком долго.
        Это предотвращает ситуацию, когда робот продолжает двигаться после
        прекращения команд (например, если джойстик отключился или нода зависла).
        """
        current_time = rospy.get_time()
        time_since_last_command = current_time - self.last_command_time
        
        # Если прошло слишком много времени с последней команды и робот движется - останавливаем
        if time_since_last_command > self.command_timeout and self.status == 'move':
            rospy.logwarn(f"Watchdog: No movement commands for {time_since_last_command:.3f}s, stopping robot")
            self.status = 'stop'
            self.x_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            try:
                self.gait_manager.stop()
            except Exception as e:
                rospy.logerr(f"Watchdog: Error stopping gait_manager: {e}")
    
    def health_check_service(self, req):
        """Обработчик сервиса проверки здоровья робота."""
        response = TriggerResponse()
        try:
            arduino_connected = self.serial_handler.is_connected()
            
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
        imu_data_dict = self.imu_data_handler.process(msg)
        if imu_data_dict is None:
            return
        
        current_time = rospy.get_time()
        # Прямой доступ к словарям для уменьшения задержки
        orientation = imu_data_dict['orientation']
        angular_velocity = imu_data_dict['angular_velocity']
        linear_acceleration = imu_data_dict['linear_acceleration']
        
        roll_deg = orientation['roll_deg']
        pitch_deg = orientation['pitch_deg']
        ax = linear_acceleration['x']
        ay = linear_acceleration['y']
        az = linear_acceleration['z']
        
        detected_state = self.fall_detector.detect(
            roll_deg, pitch_deg, ax, ay, az, current_time, self.fall_check_cooldown
        )
        
        old_state = self.robot_state_manager.get_state()
        self.robot_state_manager.set_state(detected_state)
        
        if old_state != detected_state:
            if detected_state != 'stand' and old_state == 'stand':
                self.fall_check_cooldown = 0.0
                rospy.logwarn(f"Robot fell! State: {detected_state}")
            elif detected_state == 'stand' and old_state != 'stand':
                self.fall_check_cooldown = current_time + self.fall_check_cooldown_duration
                self.auto_getup.reset()
                rospy.loginfo("Robot recovered to stand position")
        
        # Прямой доступ для уменьшения задержки
        imu_data = IMUData(
            orientation={
                'roll': roll_deg,
                'pitch': pitch_deg,
                'yaw': orientation['yaw_deg']
            },
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration
        )
        
        self.last_imu_data = imu_data_dict
        
        # ВАЖНО: Стабилизация НЕ вызывается здесь!
        # Стабилизация применяется только в axes_callback() вместе с командами движения
        # Это предотвращает конфликты - все команды движения в одном месте
        
        self.auto_getup.check(
            detected_state,
            self.motion_manager,
            self.game_services.can_move,
            current_time,
            self.fall_check_cooldown
        )
    
    def axes_callback(self, axes):
        """
        Обработчик осей джойстика.
        ЕДИНСТВЕННОЕ место установки команд движения - как в старой рабочей версии.
        """
        if not self.game_services.can_move():
            if self.status == 'move':
                self.status = 'stop'
                self.gait_manager.stop()
                rospy.logwarn("Movement command blocked - no permission")
            return
        
        # Подготавливаем данные IMU для стабилизации (если есть)
        imu_data = None
        if self.last_imu_data:
            orientation = self.last_imu_data.get('orientation', {})
            angular_velocity = self.last_imu_data.get('angular_velocity', {})
            linear_acceleration = self.last_imu_data.get('linear_acceleration', {})
            
            imu_data = IMUData(
                orientation={
                    'roll': orientation.get('roll_deg', 0.0),
                    'pitch': orientation.get('pitch_deg', 0.0),
                    'yaw': orientation.get('yaw_deg', 0.0)
                },
                angular_velocity=angular_velocity,
                linear_acceleration=linear_acceleration
            )
        
        # ЕДИНСТВЕННОЕ место установки команд движения
        # Стабилизация применяется ВНУТРИ process_axes()
        # Передаем предыдущий статус для определения переходов
        x_move_amp, y_move_amp, angle_move_amp, status = self.speed_control.process_axes(
            axes, imu_data, previous_status=self.status
        )
        
        self.x_move_amplitude = x_move_amp
        self.y_move_amplitude = y_move_amp
        self.angle_move_amplitude = angle_move_amp
        self.status = status
    
    def height_callback(self, axes):
        """Обработчик изменения высоты."""
        new_height, new_timestamp, updated = self.speed_control.process_height(
            axes, self.time_stamp_ry
        )
        if updated and new_height is not None:
            self.time_stamp_ry = new_timestamp
    
    def joy_callback(self, joy_msg):
        """Главный обработчик сообщений джойстика."""
        # Обновляем время последней команды при каждом сообщении от джойстика
        self.last_command_time = rospy.get_time()
        
        axes = dict(zip(self.axes_map, joy_msg.axes))
        buttons = dict(zip(self.button_map, joy_msg.buttons))
        
        axes_changed = any(self.last_axes.get(key, 0.0) != value for key, value in axes.items())
        
        # ВСЕГДА вызываем axes_callback для постоянного обновления параметров и стабилизации
        # Это нужно для работы стабилизации во время покоя
        try:
            self.axes_callback(axes)
            self.height_callback(axes)
        except Exception as e:
            rospy.logerr(f"Error in axes processing: {str(e)}")
        
        for key, value in buttons.items():
            if not key:
                continue
            
            if value != self.last_buttons.get(key, 0.0):
                new_state = 1 if value > 0 else 3
                callback_name = f"{key}_callback"
                
                if hasattr(self.button_handler, callback_name):
                    try:
                        if callback_name == 'start_callback':
                            self.button_handler.start_callback(
                                new_state,
                                lambda: self.speed_control.get_body_height(),
                                lambda h: self.speed_control.set_body_height(h)
                            )
                        else:
                            getattr(self.button_handler, callback_name)(new_state)
                    except Exception as e:
                        rospy.logerr(f"Error in button callback '{callback_name}': {str(e)}")
        
        self.last_buttons = buttons.copy()
        self.last_axes = axes.copy()
    
    def _shutdown_handler(self):
        """Обработчик завершения работы - гарантирует остановку робота."""
        try:
            rospy.loginfo("Shutting down joystick controller...")
            # Принудительно обнуляем амплитуды
            self.x_move_amplitude = 0.0
            self.y_move_amplitude = 0.0
            self.angle_move_amplitude = 0.0
            self.status = 'stop'
            # Останавливаем робота несколько раз для надежности
            try:
                self.gait_manager.stop()
            except Exception as e:
                rospy.logerr(f"Error stopping gait_manager in shutdown: {e}")
            # Даем время на остановку
            rospy.sleep(0.1)
            try:
                self.gait_manager.stop()
            except Exception:
                pass
            self.serial_handler.stop()
            rospy.loginfo("Robot stopped on shutdown")
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())


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
        if node is not None:
            try:
                node.gait_manager.stop()
                node.serial_handler.stop()
                rospy.loginfo("Robot stopped on shutdown")
            except Exception:
                pass

