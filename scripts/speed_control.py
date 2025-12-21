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
        self.auto_stabilization = None  # Будет установлено из joystick_control
        
        rospy.loginfo("SpeedControl module initialized")
    
    def set_auto_stabilization(self, auto_stabilization):
        """
        Устанавливает ссылку на модуль автостабилизации для применения корректировок при ходьбе.
        
        Args:
            auto_stabilization: Экземпляр AutoStabilization
        """
        self.auto_stabilization = auto_stabilization
    
    def _setup_speed_parameters(self):
        """
        Инициализирует словарь с параметрами для каждого режима скорости.
        Режимы основаны на пресетах App Speed (1-4):
        - 1: Very Low Speed (самая медленная, period_time=600ms)
        - 2: Low Speed (period_time=500ms)
        - 3: Medium Speed (period_time=400ms)
        - 4: High Speed (самая быстрая, period_time=300ms)
        
        Параметры соответствуют базовым пресетам из ainex_controller.py (set_app_walking_param_callback).
        ВАЖНО: dsp_ratio и y_swap_amplitude находятся в period_time[1] и period_time[2],
        они передаются напрямую через period_time в gait_manager.set_step().
        """
        return {
            1: {
                # Very Low Speed - самая медленная и стабильная
                'period_time': [400, 0.5, 0.015],  # [period_ms, dsp_ratio, y_swap_amplitude]
                'x_amp': 0.01,   # Ограничивается до ±0.01 или ±0.015 в зависимости от направления
                'y_amp': 0.015,   # Ограничивается до ±0.01 или ±0.012 в зависимости от направления
                'angle_amp': 2.9,  # Ограничивается до ±10 градусов
                'z_move_amplitude': 0.025,
                'arm_swap': 30.0,  # arm_swing_gain = 0.5 радиан ≈ 30 градусов
                'gait_base': {
                    #'dsp_ratio': 0.5,  # доля времени в цикле, когда обе ноги на земле
                    'step_fb_ratio': 0.070,  # дистанция шага вперед и назад
                    #'y_swap_amplitude': 0.015,  # амплитуда качания ноги влево и вправо
                    'z_swap_amplitude': 0.0015,  # амплитуда подъема ноги
                    'init_y_offset': 0.03,  # РАЗДВИЖЕНИЕ НОГ
                    'init_z_offset': 0.04,  # Может меняться до 0 в зависимости от движения
                    'init_x_offset': 0.0,   # смещение корпуса влево и вправо
                    'init_roll_offset': 0.0,  # Может меняться до 3-5 в зависимости от движения
                    'init_pitch_offset': -8.0,  # угол наклона корпуса вперед и назад
                    'hip_pitch_offset': 1.0, # угол наклона бедра вперед и назад
                    'pelvis_offset': 5.0  #расстояние между поворотами тазобедренных суставов влево и вправо
                }
            },
            2: {
                # Low Speed
                'period_time': [500, 0.2, 0.02],  # [period_ms, dsp_ratio, y_swap_amplitude]
                'x_amp': 0.01,   # Ограничивается до ±0.01 или ±0.015
                'y_amp': 0.01,   # Ограничивается до ±0.01 или ±0.015
                'angle_amp': 10,  # Ограничивается до ±10 градусов
                'z_move_amplitude': 0.02,
                'arm_swap': 30.0,
                'gait_base': {
                    'step_fb_ratio': 0.028,
                    'z_swap_amplitude': 0.006,
                    'init_y_offset': -0.008,  # Может меняться до -0.005
                    'init_x_offset': 0.0,
                    'init_roll_offset': 0.0,  # Может меняться до 3
                    'init_pitch_offset': 0.0,
                    'hip_pitch_offset': 15.0,
                    'pelvis_offset': 5.0
                }
            },
            3: {
                # Medium Speed
                'period_time': [400, 0.2, 0.02],  # [period_ms, dsp_ratio, y_swap_amplitude]
                'x_amp': 0.01,   # Ограничивается до ±0.01
                'y_amp': 0.01,   # Ограничивается до ±0.01 или ±0.015
                'angle_amp': 8,   # Ограничивается до ±8 или ±10 градусов
                'z_move_amplitude': 0.02,
                'arm_swap': 30.0,
                'gait_base': {
                    'step_fb_ratio': 0.028,
                    'z_swap_amplitude': 0.006,
                    'init_y_offset': -0.005,  # Может меняться до 0
                    'init_x_offset': 0.0,
                    'init_roll_offset': 0.0,  # Может меняться до 3
                    'init_pitch_offset': 0.0,
                    'hip_pitch_offset': 15.0,
                    'pelvis_offset': 5.0
                }
            },
            4: {
                # High Speed - самая быстрая
                'period_time': [300, 0.2, 0.021],  # [period_ms, dsp_ratio, y_swap_amplitude]
                'x_amp': 0.01,   # Ограничивается до ±0.01
                'y_amp': 0.01,   # Ограничивается до ±0.01 или ±0.015
                'angle_amp': 8,   # Ограничивается до ±8 градусов
                'z_move_amplitude': 0.015,
                'arm_swap': 30.0,
                'gait_base': {
                    'step_fb_ratio': 0.028,
                    'z_swap_amplitude': 0.006,
                    'init_y_offset': -0.008,
                    'init_x_offset': 0.0,
                    'init_roll_offset': 0.0,  # Может меняться до -3 или -1.0
                    'init_pitch_offset': 0.0,
                    'hip_pitch_offset': 15.0,
                    'pelvis_offset': 5.0
                }
            }
        }
    
    def get_body_height(self):
        """
        Получает текущую высоту корпуса из gait_manager (единый источник истины).
        
        Returns:
            float: Текущая высота корпуса
        """
        gait_param = self.gait_manager.get_gait_param()
        return gait_param.get('body_height', HEIGHT_DEFAULT)
    
    def set_body_height(self, height):
        """
        Устанавливает высоту корпуса через gait_manager (единый источник истины).
        
        Args:
            height: Новая высота корпуса (будет ограничена диапазоном)
        """
        height = max(HEIGHT_MIN, min(HEIGHT_MAX, height))
        gait_param = self.gait_manager.get_gait_param()
        gait_param['body_height'] = height
        # Обновляем параметры без движения, используем текущий режим скорости
        params = self.speed_params[self.speed_mode]
        self.gait_manager.update_param(
            params['period_time'],
            0, 0, 0,
            gait_param,
            step_num=0
        )
        return height
    
    def process_axes(self, axes, adaptation_factor=1.0):
        """
        Обрабатывает данные осей джойстика и устанавливает параметры движения.
        
        Args:
            axes: Словарь с данными осей джойстика
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
        
        # Применяем базовые параметры режима скорости
        # Стабилизация при ходьбе может переопределить их только в меньшую сторону
        gait_param.update(params.get('gait_base', {}))
        
        # Вычисляем амплитуды движения
        if abs(axes['ly']) > AXIS_THRESHOLD:
            x_move_amplitude = math.copysign(params['x_amp'], axes['ly'])
        if abs(axes['lx']) > AXIS_THRESHOLD:
            y_move_amplitude = math.copysign(params['y_amp'], axes['lx'])
        if abs(axes['rx']) > AXIS_THRESHOLD:
            angle_move_amplitude = math.copysign(params['angle_amp'], axes['rx'])
        
        update_param = any(abs(amp) > 0 for amp in [x_move_amplitude, y_move_amplitude, angle_move_amplitude])
        
        if update_param:
            # Применяем корректировки стабилизации при ходьбе ТОЛЬКО когда есть движение
            # Это должно быть ПОСЛЕ установки базовых параметров, но ДО set_step
            if self.auto_stabilization is not None:
                self.auto_stabilization.apply_walking_corrections(gait_param, period_time)
            # Используем body_height из gait_manager (единый источник истины)
            # Не устанавливаем init_z_offset, так как body_height уже актуален
            
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
            # При остановке сбрасываем корректировки стабилизации при ходьбе
            if self.auto_stabilization is not None:
                self.auto_stabilization.reset_walking_corrections()
            # Возвращаем базовые параметры режима скорости (сбрасываем корректировки)
            gait_param.update(params.get('gait_base', {}))
            # Обновляем параметры без движения для сброса корректировок
            self.gait_manager.update_param(
                params['period_time'],
                0, 0, 0,  # Без движения
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
            if abs(axes['ry']) > HEIGHT_AXIS_THRESHOLD:
                # Получаем текущую высоту из gait_manager
                current_height = self.get_body_height()
                # Вычисляем новую высоту
                new_height = current_height - HEIGHT_STEP * math.copysign(1, axes['ry'])
                # Устанавливаем через единый метод
                self.set_body_height(new_height)
                updated = True
                time_stamp_ry = current_time + HEIGHT_UPDATE_INTERVAL
        
        new_body_height = self.get_body_height() if updated else None
        return new_body_height, time_stamp_ry, updated
    
    def set_speed_mode(self, mode):
        """
        Устанавливает режим скорости.
        
        Args:
            mode: Режим скорости (1-4): 
                  1=Very Low Speed (самая медленная, period_time=600ms)
                  2=Low Speed (period_time=500ms)
                  3=Medium Speed (period_time=400ms)
                  4=High Speed (самая быстрая, period_time=300ms)
        
        Returns:
            bool: True если режим установлен успешно
        """
        if 1 <= mode <= 4:
            self.speed_mode = mode
            speed_names = {1: "Very Low", 2: "Low", 3: "Medium", 4: "High"}
            rospy.loginfo(f"Speed mode set to: {self.speed_mode} ({speed_names[mode]} Speed)")
            return True
        return False
    
    def get_speed_mode(self):
        """Возвращает текущий режим скорости."""
        return self.speed_mode
    
    def get_speed_params(self):
        """Возвращает параметры скорости."""
        return self.speed_params

