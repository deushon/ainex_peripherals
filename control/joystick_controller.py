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
from walking import SpeedControl, StabilizationModule, IMUData, ThrottleData, converter
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
        
        stabilization_module = StabilizationModule()
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
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        self.health_service = rospy.Service('/game/robot_health', Trigger, self.health_check_service)
        
        rospy.Timer(
            rospy.Duration(self.serial_handler.reconnect_interval),
            self._check_and_reconnect_serial
        )
        
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
        orientation = imu_data_dict.get('orientation', {})
        angular_velocity = imu_data_dict.get('angular_velocity', {})
        linear_acceleration = imu_data_dict.get('linear_acceleration', {})
        
        roll_deg = orientation.get('roll_deg', 0.0)
        pitch_deg = orientation.get('pitch_deg', 0.0)
        ax = linear_acceleration.get('x', 0.0)
        ay = linear_acceleration.get('y', 0.0)
        az = linear_acceleration.get('z', 0.0)
        
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
        
        imu_data = IMUData(
            orientation={
                'roll': roll_deg,
                'pitch': pitch_deg,
                'yaw': orientation.get('yaw_deg', 0.0)
            },
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration
        )
        
        self.last_imu_data = imu_data_dict
        
        # Вызываем стабилизацию прямо здесь, чтобы она работала постоянно
        # даже когда робот стоит и джойстик не двигается
        self._apply_stabilization(imu_data)
        
        self.auto_getup.check(
            detected_state,
            self.motion_manager,
            self.game_services.can_move,
            current_time,
            self.fall_check_cooldown
        )
    
    def _apply_stabilization(self, imu_data: IMUData):
        """
        Применяет стабилизацию на основе данных IMU.
        Вызывается постоянно из imu_callback, даже когда робот стоит.
        """
        try:
            # Получаем текущие параметры из gait_manager
            gait_param = self.gait_manager.get_gait_param()
            
            # Получаем текущие параметры скорости
            speed_mode = self.speed_control.speed_mode
            if speed_mode in self.speed_control.speed_params:
                params = self.speed_control.speed_params[speed_mode]
                period_time = list(params['period_time'])
                move_amplitudes = {
                    'x': self.x_move_amplitude,
                    'y': self.y_move_amplitude,
                    'angle': self.angle_move_amplitude
                }
            else:
                rospy.logwarn("_apply_stabilization: speed_mode not found, using defaults")
                period_time = [400, 0.2, 0.02]
                move_amplitudes = {'x': 0.0, 'y': 0.0, 'angle': 0.0}
            
            # Конвертируем в WalkingParams
            current_walking_params = converter.gait_param_to_walking_params(
                gait_param, period_time, move_amplitudes
            )
            
            # Создаем ThrottleData (может быть нулевым, если робот стоит)
            throttle_data = ThrottleData(
                x=self.x_move_amplitude,
                y=self.y_move_amplitude,
                angle=self.angle_move_amplitude
            )
            
            # Вызываем стабилизацию
            stabilization_result = self.speed_control.get_stabilization_module().process(
                imu_data, throttle_data, current_walking_params
            )
            
            # Применяем результат, если были изменения
            if stabilization_result.modified:
                current_walking_params = converter.apply_stabilization_result(
                    current_walking_params, stabilization_result
                )
                
                # Конвертируем обратно в gait_param
                gait_param = converter.walking_params_to_gait_param(current_walking_params)
                period_time = converter.walking_params_to_period_time(current_walking_params)
                
                # Применяем изменения через update_param
                self.gait_manager.update_param(
                    period_time,
                    self.x_move_amplitude,
                    self.y_move_amplitude,
                    self.angle_move_amplitude,
                    gait_param
                )
        except Exception as e:
            rospy.logerr(f"Error in _apply_stabilization: {e}")
            import traceback
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    def axes_callback(self, axes):
        """Обработчик осей джойстика."""
        if not self.game_services.can_move():
            if self.status == 'move':
                self.status = 'stop'
                self.gait_manager.stop()
                rospy.logwarn("Movement command blocked - no permission")
            return
        
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
            rospy.logdebug(f"JoystickController.axes_callback: IMU data prepared, "
                          f"pitch={imu_data.orientation.get('pitch', 0.0):.2f}°, "
                          f"roll={imu_data.orientation.get('roll', 0.0):.2f}°")
        else:
            rospy.logwarn("JoystickController.axes_callback: No IMU data available (last_imu_data is None)")
        
        x_move_amp, y_move_amp, angle_move_amp, status = self.speed_control.process_axes(
            axes, imu_data
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
        axes = dict(zip(self.axes_map, joy_msg.axes))
        buttons = dict(zip(self.button_map, joy_msg.buttons))
        
        axes_changed = any(self.last_axes.get(key, 0.0) != value for key, value in axes.items())
        
        if axes_changed:
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
            self.gait_manager.stop()
            self.serial_handler.stop()
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
        if node is not None:
            try:
                node.gait_manager.stop()
                node.serial_handler.stop()
                rospy.loginfo("Robot stopped on shutdown")
            except Exception:
                pass

