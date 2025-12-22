#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль управления скоростью и движением робота.
Интегрирован с модулем стабилизации.
"""

import rospy
import math
from config import ConfigLoader
from walking import (
    StabilizationModule,
    IMUData,
    ThrottleData,
    converter
)


class SpeedControl:
    """
    Класс для управления скоростью и движением робота.
    """
    
    def __init__(self, gait_manager, config_loader=None, stabilization_module=None):
        """
        Инициализация модуля управления скоростью.
        
        Args:
            gait_manager: Экземпляр GaitManager
            config_loader: Экземпляр ConfigLoader (опционально)
            stabilization_module: Экземпляр StabilizationModule (опционально)
        """
        self.gait_manager = gait_manager
        self.config = config_loader if config_loader else ConfigLoader()
        self.stabilization = stabilization_module if stabilization_module else StabilizationModule()
        
        walking_config = self.config.load('walking')
        walking_params = walking_config.get('walking', {})
        
        self.axis_threshold = walking_params.get('axis_threshold', 0.3)
        height_params = walking_params.get('height', {})
        self.height_axis_threshold = height_params.get('axis_threshold', 0.5)
        self.height_update_interval = height_params.get('update_interval', 0.05)
        self.height_step = height_params.get('step', 0.005)
        self.height_min = height_params.get('min', 0.025)
        self.height_max = height_params.get('max', 0.06)
        self.height_default = height_params.get('default', 0.025)
        
        self.speed_mode = 1
        speed_modes_config = walking_params.get('speed_modes', {})
        rospy.logdebug(f"Loading speed modes from config. Config keys: {list(speed_modes_config.keys()) if speed_modes_config else 'empty'}, types: {[type(k).__name__ for k in speed_modes_config.keys()] if speed_modes_config else 'empty'}")
        self.speed_params = self._load_speed_parameters(speed_modes_config)
        
        if not self.speed_params:
            rospy.logerr("Failed to load speed parameters from config!")
        else:
            rospy.loginfo(f"SpeedControl module initialized with {len(self.speed_params)} speed modes: {list(self.speed_params.keys())}")
    
    def _load_speed_parameters(self, speed_modes_config):
        """
        Загружает параметры режимов скорости из конфига.
        
        Args:
            speed_modes_config: Словарь с конфигурацией режимов скорости
        
        Returns:
            dict: Словарь с параметрами для каждого режима скорости
        """
        speed_params = {}
        
        for mode_num in range(1, 5):
            mode_config = speed_modes_config.get(mode_num) or speed_modes_config.get(str(mode_num), {})
            if not mode_config:
                rospy.logwarn(f"Speed mode {mode_num} not found in config, using defaults")
                mode_config = {}
            else:
                rospy.logdebug(f"Loaded speed mode {mode_num} from config")
            
            period_time_list = mode_config.get('period_time', [400, 0.2, 0.02])
            move_amps = mode_config.get('move_amplitudes', {})
            gait_base = mode_config.get('gait_base', {})
            
            speed_params[mode_num] = {
                'period_time': period_time_list,
                'x_amp': move_amps.get('x', 0.01),
                'y_amp': move_amps.get('y', 0.01),
                'angle_amp': move_amps.get('angle', 10.0),
                'z_move_amplitude': mode_config.get('z_move_amplitude', 0.02),
                'arm_swap': mode_config.get('arm_swap', 30.0),
                'gait_base': gait_base
            }
        
        if not speed_params:
            rospy.logerr("No speed modes loaded from config! Using default mode 1")
            speed_params[1] = {
                'period_time': [400, 0.2, 0.02],
                'x_amp': 0.01,
                'y_amp': 0.01,
                'angle_amp': 10.0,
                'z_move_amplitude': 0.02,
                'arm_swap': 30.0,
                'gait_base': {
                    'step_fb_ratio': 0.028,
                    'z_swap_amplitude': 0.006,
                    'hip_pitch_offset': 15.0,
                    'pelvis_offset': 5.0
                }
            }
        
        if not speed_params:
            rospy.logerr("No speed modes loaded from config! Using default mode 1")
            speed_params[1] = {
                'period_time': [400, 0.2, 0.02],
                'x_amp': 0.01,
                'y_amp': 0.01,
                'angle_amp': 10.0,
                'z_move_amplitude': 0.02,
                'arm_swap': 30.0,
                'gait_base': {
                    'step_fb_ratio': 0.028,
                    'z_swap_amplitude': 0.006,
                    'hip_pitch_offset': 15.0,
                    'pelvis_offset': 5.0
                }
            }
        
        return speed_params
    
    def get_body_height(self):
        """
        Получает текущую высоту корпуса из gait_manager.
        
        Returns:
            float: Текущая высота корпуса
        """
        gait_param = self.gait_manager.get_gait_param()
        return gait_param.get('body_height', self.height_default)
    
    def set_body_height(self, height):
        """
        Устанавливает высоту корпуса через gait_manager.
        
        Args:
            height: Новая высота корпуса (будет ограничена диапазоном)
        
        Returns:
            float: Установленная высота
        """
        height = max(self.height_min, min(self.height_max, height))
        gait_param = self.gait_manager.get_gait_param()
        gait_param['body_height'] = height
        
        if self.speed_mode not in self.speed_params:
            rospy.logerr(f"Speed mode {self.speed_mode} not found in speed_params! Available: {list(self.speed_params.keys())}")
            if self.speed_params:
                self.speed_mode = list(self.speed_params.keys())[0]
                rospy.logwarn(f"Switching to speed mode {self.speed_mode}")
            else:
                rospy.logerr("No speed params available! Cannot set body height")
                return height
        
        params = self.speed_params[self.speed_mode]
        self.gait_manager.update_param(
            params['period_time'],
            0, 0, 0,
            gait_param,
            step_num=0
        )
        return height
    
    def process_axes(self, axes, imu_data=None):
        """
        Обрабатывает данные осей джойстика и устанавливает параметры движения.
        Интегрирован с модулем стабилизации.
        
        Args:
            axes: Словарь с данными осей джойстика
            imu_data: Данные IMU для стабилизации (опционально)
        
        Returns:
            tuple: (x_move_amplitude, y_move_amplitude, angle_move_amplitude, status)
        """
        has_movement = any(abs(axes.get(k, 0.0)) > 0.1 for k in ['lx', 'ly', 'rx'])
        rospy.loginfo(f"SpeedControl.process_axes: called, imu_data={'present' if imu_data else 'None'}, "
                     f"has_movement={has_movement}, "
                     f"stabilization.enabled={self.stabilization.is_enabled() if self.stabilization else 'N/A'}")
        
        x_move_amplitude = 0.0
        y_move_amplitude = 0.0
        angle_move_amplitude = 0.0
        
        gait_param = self.gait_manager.get_gait_param()
        
        if self.speed_mode not in self.speed_params:
            rospy.logerr(f"Speed mode {self.speed_mode} not found in speed_params! Available: {list(self.speed_params.keys())}")
            if self.speed_params:
                self.speed_mode = list(self.speed_params.keys())[0]
                rospy.logwarn(f"Switching to speed mode {self.speed_mode}")
            else:
                rospy.logerr("No speed params available! Cannot process axes")
                return 0.0, 0.0, 0.0, 'stop'
        
        params = self.speed_params[self.speed_mode]
        period_time = list(params['period_time'])
        
        offset_keys = ['init_x_offset', 'init_y_offset', 'init_roll_offset', 'init_pitch_offset']
        saved_offsets = {key: gait_param.get(key, 0.0) for key in offset_keys}
        
        gait_base = params.get('gait_base', {})
        for key, value in gait_base.items():
            if key not in offset_keys:
                gait_param[key] = value
        
        for key in offset_keys:
            gait_param[key] = saved_offsets[key]
        
        if abs(axes.get('ly', 0.0)) > self.axis_threshold:
            x_move_amplitude = math.copysign(params['x_amp'], axes['ly'])
        if abs(axes.get('lx', 0.0)) > self.axis_threshold:
            y_move_amplitude = math.copysign(params['y_amp'], axes['lx'])
        if abs(axes.get('rx', 0.0)) > self.axis_threshold:
            angle_move_amplitude = math.copysign(params['angle_amp'], axes['rx'])
        
        move_amplitudes = {
            'x': x_move_amplitude,
            'y': y_move_amplitude,
            'angle': angle_move_amplitude
        }
        
        update_param = any(abs(amp) > 0.0 for amp in move_amplitudes.values())
        stabilization_result = None
        
        # Стабилизация должна работать всегда, когда есть данные IMU (даже если робот стоит)
        if imu_data:
            rospy.logdebug(f"SpeedControl: processing stabilization, update_param={update_param}, "
                          f"imu_data.orientation={imu_data.orientation if imu_data else None}")
        else:
            rospy.logwarn("SpeedControl: imu_data is None, stabilization will not be called!")
            current_walking_params = converter.gait_param_to_walking_params(
                gait_param, period_time, move_amplitudes
            )
            
            throttle_data = ThrottleData(
                x=x_move_amplitude,
                y=y_move_amplitude,
                angle=angle_move_amplitude
            )
            
            stabilization_result = self.stabilization.process(
                imu_data, throttle_data, current_walking_params
            )
            
            rospy.logdebug(f"SpeedControl: stabilization_result.modified={stabilization_result.modified if stabilization_result else False}")
            
            if stabilization_result.modified:
                current_walking_params = converter.apply_stabilization_result(
                    current_walking_params, stabilization_result
                )
                
                gait_param = converter.walking_params_to_gait_param(current_walking_params)
                period_time = converter.walking_params_to_period_time(current_walking_params)
                
                # Обновляем move_amplitudes только если они были изменены стабилизацией
                # (обычно они не меняются, но на всякий случай)
                move_amplitudes = current_walking_params.move_amplitudes
                x_move_amplitude = move_amplitudes['x']
                y_move_amplitude = move_amplitudes['y']
                angle_move_amplitude = move_amplitudes['angle']
        
        if update_param:
            self.gait_manager.set_step(
                period_time,
                x_move_amplitude,
                y_move_amplitude,
                angle_move_amplitude,
                gait_param,
                step_num=0
            )
        elif stabilization_result and stabilization_result.modified:
            # Если робот стоит, но стабилизация изменила позу - применяем через update_param
            self.gait_manager.update_param(
                period_time,
                0, 0, 0,
                gait_param
            )
        
        status = 'move' if update_param else 'stop'
        if status == 'stop':
            gait_param.update(params.get('gait_base', {}))
            self.gait_manager.update_param(
                params['period_time'],
                0, 0, 0,
                gait_param,
                step_num=0
            )
            self.gait_manager.stop()
        
        return x_move_amplitude, y_move_amplitude, angle_move_amplitude, status
    
    def process_height(self, axes, time_stamp_ry):
        """
        Обрабатывает изменение высоты робота.
        
        Args:
            axes: Словарь с данными осей джойстика
            time_stamp_ry: Временная метка для ограничения частоты обновления
        
        Returns:
            tuple: (new_body_height, new_time_stamp_ry, updated)
        """
        current_time = rospy.get_time()
        updated = False
        
        if current_time > time_stamp_ry:
            if abs(axes.get('ry', 0.0)) > self.height_axis_threshold:
                current_height = self.get_body_height()
                new_height = current_height - self.height_step * math.copysign(1.0, axes['ry'])
                self.set_body_height(new_height)
                updated = True
                time_stamp_ry = current_time + self.height_update_interval
        
        new_body_height = self.get_body_height() if updated else None
        return new_body_height, time_stamp_ry, updated
    
    def set_speed_mode(self, mode):
        """
        Устанавливает режим скорости.
        
        Args:
            mode: Режим скорости (1-4)
        
        Returns:
            bool: True если режим установлен успешно
        """
        if 1 <= mode <= 4:
            self.speed_mode = mode
            speed_names = self._get_speed_names()
            rospy.loginfo(f"Speed mode set to: {self.speed_mode} ({speed_names.get(mode, 'Unknown')} Speed)")
            return True
        return False
    
    def _get_speed_names(self):
        """Возвращает словарь имен режимов скорости."""
        return {
            1: "Very Low",
            2: "Low",
            3: "Medium",
            4: "High"
        }
    
    def get_speed_mode(self):
        """Возвращает текущий режим скорости."""
        return self.speed_mode
    
    def get_speed_params(self):
        """Возвращает параметры скорости."""
        return self.speed_params
    
    def set_stabilization_module(self, stabilization_module):
        """
        Устанавливает модуль стабилизации.
        
        Args:
            stabilization_module: Экземпляр StabilizationModule
        """
        self.stabilization = stabilization_module
    
    def get_stabilization_module(self):
        """Возвращает модуль стабилизации."""
        return self.stabilization

