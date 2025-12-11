#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль управления скоростью и движением робота.
"""

# ========== КОНФИГУРАЦИЯ ==========
# Пороги для обработки осей джойстика
AXIS_THRESHOLD = 0.3  # Порог для определения движения по осям
HEIGHT_AXIS_THRESHOLD = 0.5  # Порог для изменения высоты
HEIGHT_UPDATE_INTERVAL = 0.05  # Интервал обновления высоты (сек)
HEIGHT_STEP = 0.005  # Шаг изменения высоты
HEIGHT_MIN = 0.025  # Минимальная высота
HEIGHT_MAX = 0.06  # Максимальная высота
HEIGHT_DEFAULT = 0.025  # Высота по умолчанию

# Адаптация резонанса
RESONANCE_ADAPTATION_THRESHOLD = 0.01  # Порог для применения адаптации
RESONANCE_PERIOD_MULTIPLIER = 0.3  # Множитель для увеличения периода при резонансе
RESONANCE_FACTOR_THRESHOLD = 0.8  # Порог фактора адаптации для изменения периода
# ===================================

import rospy
import math


class SpeedControl:
    """
    Класс для управления скоростью и движением робота.
    """
    
    def __init__(self, gait_manager):
        """
        Инициализация модуля управления скоростью.
        
        Args:
            gait_manager: Экземпляр GaitManager
        """
        self.gait_manager = gait_manager
        self.speed_mode = 1
        self.speed_params = self._setup_speed_parameters()
        
        rospy.loginfo("SpeedControl module initialized")
    
    def _setup_speed_parameters(self):
        """Инициализирует словарь с параметрами для каждого режима скорости."""
        return {
            1: {
                'period_time': [400, 0.2, 0.022],
                'x_amp': 0.01,
                'y_amp': 0.015,
                'angle_amp': 8,
                'z_move_amplitude': 0.025
            },
            2: {
                'period_time': [500, 0.2, 0.028],
                'x_amp': 0.015,
                'y_amp': 0.015,
                'angle_amp': 10,
                'z_move_amplitude': 0.02,
                'gait_base': {
                    'dsp_ratio': 0.2,
                    'step_fb_ratio': 0.028,
                    'y_swap_amplitude': 0.02,
                    'z_swap_amplitude': 0.006,
                    'init_y_offset': -0.008
                }
            },
            3: {
                'period_time': [400, 0.2, 0.028],
                'x_amp': 0.01,
                'y_amp': 0.015,
                'angle_amp': 10,
                'z_move_amplitude': 0.02,
                'gait_base': {
                    'dsp_ratio': 0.2,
                    'init_y_offset': -0.005,
                    'step_fb_ratio': 0.028,
                    'y_swap_amplitude': 0.02,
                    'z_swap_amplitude': 0.006
                }
            },
            4: {
                'period_time': [300, 0.2, 0.028],
                'x_amp': 0.01,
                'y_amp': 0.015,
                'angle_amp': 8,
                'z_move_amplitude': 0.015,
                'gait_base': {
                    'dsp_ratio': 0.2,
                    'init_y_offset': -0.008,
                    'step_fb_ratio': 0.028,
                    'y_swap_amplitude': 0.021,
                    'z_swap_amplitude': 0.006,
                    'pelvis_offset': 5,
                    'arm_swing_gain': 0.5
                }
            }
        }
    
    def process_axes(self, axes, init_z_offset, adaptation_factor=1.0):
        """
        Обрабатывает данные осей джойстика и устанавливает параметры движения.
        
        Args:
            axes: Словарь с данными осей джойстика
            init_z_offset: Начальное смещение по Z
            adaptation_factor: Фактор адаптации для резонанса (по умолчанию 1.0)
        
        Returns:
            tuple: (x_move_amplitude, y_move_amplitude, angle_move_amplitude, status)
                status: 'move' или 'stop'
        """
        x_move_amplitude = 0
        y_move_amplitude = 0
        angle_move_amplitude = 0
        
        gait_param = self.gait_manager.get_gait_param()
        params = self.speed_params[self.speed_mode]
        period_time = list(params['period_time'])
        
        if self.speed_mode > 1:
            gait_param.update(params.get('gait_base', {}))
        
        # Условная логика для разных режимов скорости
        if self.speed_mode == 1 and abs(axes['lx']) > AXIS_THRESHOLD:
            period_time[2] = 0.025
        elif self.speed_mode == 2:
            if abs(axes['rx']) > AXIS_THRESHOLD and abs(axes['lx']) < AXIS_THRESHOLD and abs(axes['ly']) < AXIS_THRESHOLD:
                gait_param.update({'init_roll_offset': 0, 'init_y_offset': -0.005, 'y_swap_amplitude': 0.022})
            if abs(axes['lx']) > AXIS_THRESHOLD and abs(axes['ly']) < AXIS_THRESHOLD and abs(axes['rx']) < AXIS_THRESHOLD:
                gait_param.update({'init_y_offset': -0.005, 'y_swap_amplitude': 0.028, 'init_roll_offset': 3})
        elif self.speed_mode == 3:
            if abs(axes['rx']) > AXIS_THRESHOLD and abs(axes['lx']) < AXIS_THRESHOLD and abs(axes['ly']) < AXIS_THRESHOLD:
                gait_param['y_swap_amplitude'] = 0.022
            if abs(axes['lx']) > AXIS_THRESHOLD and abs(axes['ly']) < AXIS_THRESHOLD and abs(axes['rx']) < AXIS_THRESHOLD:
                gait_param.update({'init_y_offset': 0, 'init_roll_offset': 3, 'y_swap_amplitude': 0.025})
        elif self.speed_mode == 4:
            if abs(axes['ly']) > AXIS_THRESHOLD:
                gait_param['init_roll_offset'] = -3
            if abs(axes['lx']) > AXIS_THRESHOLD and abs(axes['ly']) < AXIS_THRESHOLD and abs(axes['rx']) < AXIS_THRESHOLD:
                gait_param['init_roll_offset'] = -1.0
            if abs(axes['rx']) > AXIS_THRESHOLD and abs(axes['lx']) < AXIS_THRESHOLD and abs(axes['ly']) < AXIS_THRESHOLD:
                gait_param['init_roll_offset'] = -3
        
        # Вычисляем амплитуды движения
        if abs(axes['ly']) > AXIS_THRESHOLD:
            x_move_amplitude = math.copysign(params['x_amp'], axes['ly'])
        if abs(axes['lx']) > AXIS_THRESHOLD:
            y_move_amplitude = math.copysign(params['y_amp'], axes['lx'])
        if abs(axes['rx']) > AXIS_THRESHOLD:
            angle_move_amplitude = math.copysign(params['angle_amp'], axes['rx'])
        
        update_param = any(abs(amp) > 0 for amp in [x_move_amplitude, y_move_amplitude, angle_move_amplitude])
        
        if update_param:
            gait_param['init_z_offset'] = init_z_offset
            
            # Применяем адаптацию на основе резонанса, если она активна
            if abs(adaptation_factor - 1.0) > RESONANCE_ADAPTATION_THRESHOLD:
                adapted_x_amp = x_move_amplitude * adaptation_factor
                adapted_y_amp = y_move_amplitude * adaptation_factor
                adapted_angle_amp = angle_move_amplitude * adaptation_factor
                
                # Также адаптируем период времени для большей стабильности при резонансе
                adapted_period_time = list(period_time)
                if adaptation_factor < RESONANCE_FACTOR_THRESHOLD:
                    period_increase = (1.0 - adaptation_factor) * RESONANCE_PERIOD_MULTIPLIER
                    adapted_period_time[0] = int(period_time[0] * (1.0 + period_increase))
                
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
                self.gait_manager.set_step(
                    period_time,
                    x_move_amplitude,
                    y_move_amplitude,
                    angle_move_amplitude,
                    gait_param,
                    step_num=0
                )
        
        status = 'move' if update_param else 'stop'
        if status == 'stop':
            self.gait_manager.stop()
        
        return x_move_amplitude, y_move_amplitude, angle_move_amplitude, status
    
    def process_height(self, axes, init_z_offset, time_stamp_ry):
        """
        Обрабатывает изменение высоты робота.
        
        Args:
            axes: Словарь с данными осей джойстика
            init_z_offset: Текущее смещение по Z
            time_stamp_ry: Временная метка для ограничения частоты обновления
        
        Returns:
            tuple: (new_init_z_offset, new_time_stamp_ry, updated)
        """
        current_time = rospy.get_time()
        new_init_z_offset = init_z_offset
        updated = False
        
        if current_time > time_stamp_ry:
            if abs(axes['ry']) > HEIGHT_AXIS_THRESHOLD:
                new_init_z_offset -= HEIGHT_STEP * math.copysign(1, axes['ry'])
                new_init_z_offset = max(HEIGHT_MIN, min(HEIGHT_MAX, new_init_z_offset))
                updated = True
            
            if updated:
                gait_param = self.gait_manager.get_gait_param()
                gait_param['body_height'] = new_init_z_offset
                params = self.speed_params[1]  # Используем параметры режима 1 для высоты
                self.gait_manager.update_param(
                    params['period_time'],
                    0, 0, 0,
                    gait_param,
                    step_num=0
                )
                time_stamp_ry = current_time + HEIGHT_UPDATE_INTERVAL
        
        return new_init_z_offset, time_stamp_ry, updated
    
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
            rospy.loginfo(f"Speed mode set to: {self.speed_mode}")
            return True
        return False
    
    def get_speed_mode(self):
        """Возвращает текущий режим скорости."""
        return self.speed_mode
    
    def get_speed_params(self):
        """Возвращает параметры скорости."""
        return self.speed_params

