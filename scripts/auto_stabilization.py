#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль автостабилизации робота в покое.
Упрощенная логика: 1 шаг с длиной и скоростью зависящей от ускорения, затем охлаждение.
"""

# ========== КОНФИГУРАЦИЯ ==========
# Параметры автостабилизации
ACCEL_THRESHOLD = 0.3  # Порог ускорения для срабатывания (м/с²)
STEP_AMPLITUDE_BASE = 0.008  # Базовая амплитуда шага
STEP_AMPLITUDE_MAX = 0.020  # Максимальная амплитуда шага
ACCEL_MAX_FOR_MAX_STEP = 2.0  # Максимальное ускорение для максимального шага
COOLDOWN_DURATION = 2.2  # Длительность охлаждения после шага (сек)

# Калибровка гравитации
GRAVITY_BASE_Y_DEFAULT = 9.8  # Начальное значение гравитации
CALIBRATION_SAMPLES = 30  # Количество образцов для калибровки

# Фильтрация ускорений
FILTER_ALPHA = 0.8  # Коэффициент экспоненциального фильтра (0.0-1.0)

# Пороги для проверки движения от джойстика
JOYSTICK_MOVE_THRESHOLD = 0.001  # Минимальная амплитуда для определения движения

# Параметры скорости шага
SPEED_FACTOR_MIN = 0.7  # Минимальный множитель скорости (быстрый шаг)
SPEED_FACTOR_NORMAL = 1.0  # Нормальный множитель скорости
# ===================================

import rospy
import math


class AutoStabilization:
    """
    Класс для автоматической стабилизации робота в покое.
    Упрощенная логика без вычисления колебаний.
    """
    
    def __init__(self, gait_manager, speed_params, speed_mode, init_z_offset):
        """
        Инициализация модуля автостабилизации.
        
        Args:
            gait_manager: Экземпляр GaitManager для управления шагами
            speed_params: Параметры скорости
            speed_mode: Текущий режим скорости
            init_z_offset: Начальное смещение по Z
        """
        self.gait_manager = gait_manager
        self.speed_params = speed_params
        self.speed_mode = speed_mode
        self.init_z_offset = init_z_offset
        
        # Параметры автостабилизации (из конфига)
        self.enabled = True
        self.accel_threshold = ACCEL_THRESHOLD
        self.step_amplitude_base = STEP_AMPLITUDE_BASE
        self.step_amplitude_max = STEP_AMPLITUDE_MAX
        self.accel_max_for_max_step = ACCEL_MAX_FOR_MAX_STEP
        
        # Состояние
        self.last_step_time = 0
        self.cooldown_end_time = 0
        self.cooldown_duration = COOLDOWN_DURATION
        
        # Калибровка гравитации
        self.gravity_base_y = GRAVITY_BASE_Y_DEFAULT
        self.gravity_calibrated = False
        self.calibration_samples = CALIBRATION_SAMPLES
        self.calibration_data = []
        self.calibration_in_progress = True
        
        # Фильтрация ускорений
        self.filter_alpha = FILTER_ALPHA
        self.filtered_accel = {'x': 0.0, 'y': 0.0}
        
        rospy.loginfo("AutoStabilization module initialized")
    
    def update_calibration(self, ax, ay):
        """
        Обновляет калибровку базового значения гравитации.
        
        Args:
            ax: Ускорение по оси X
            ay: Ускорение по оси Y
        """
        if not self.calibration_in_progress:
            return
        
        self.calibration_data.append({'x': ax, 'y': ay})
        
        if len(self.calibration_data) > self.calibration_samples:
            self.calibration_data.pop(0)
        
        if len(self.calibration_data) >= self.calibration_samples:
            # Вычисляем среднее значение гравитации
            avg_y = sum(d['y'] for d in self.calibration_data) / len(self.calibration_data)
            self.gravity_base_y = avg_y
            self.gravity_calibrated = True
            self.calibration_in_progress = False
            self.calibration_data = []
            
            rospy.loginfo(f"AutoStabilization calibration complete. Gravity base Y: {self.gravity_base_y:.3f} m/s²")
    
    def process(self, ax, ay, robot_state, status, x_move_amp, y_move_amp, angle_move_amp):
        """
        Обрабатывает данные ускорения и выполняет стабилизацию при необходимости.
        
        Args:
            ax: Ускорение по оси X (влево/вправо)
            ay: Ускорение по оси Y (вперед/назад)
            robot_state: Состояние робота ('stand', 'lie_to_stand', 'recline_to_stand')
            status: Статус движения ('stop', 'move')
            x_move_amp: Амплитуда движения по X от джойстика
            y_move_amp: Амплитуда движения по Y от джойстика
            angle_move_amp: Амплитуда поворота от джойстика
        
        Returns:
            bool: True если был выполнен шаг стабилизации
        """
        if not self.enabled:
            return False
        
        current_time = rospy.get_time()
        
        # Калибровка
        if self.calibration_in_progress:
            self.update_calibration(ax, ay)
            return False
        
        if not self.gravity_calibrated:
            return False
        
        # Проверка охлаждения
        if current_time < self.cooldown_end_time:
            return False
        
        # Проверка условий для стабилизации
        # Работает только когда робот стоит и нет команд от джойстика
        if (robot_state != 'stand' or 
            status != 'stop' or
            abs(x_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
            abs(y_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
            abs(angle_move_amp) > JOYSTICK_MOVE_THRESHOLD):
            return False
        
        # Фильтрация ускорений
        ay_deviation = ay - self.gravity_base_y
        self.filtered_accel['x'] = self.filter_alpha * ax + (1 - self.filter_alpha) * self.filtered_accel['x']
        self.filtered_accel['y'] = self.filter_alpha * ay_deviation + (1 - self.filter_alpha) * self.filtered_accel['y']
        
        # Определение направления и величины отклонения
        fx = self.filtered_accel['x']
        fy = self.filtered_accel['y']
        
        # Вычисляем величину отклонения
        magnitude = math.sqrt(fx * fx + fy * fy)
        
        # Проверка порога
        if magnitude < self.accel_threshold:
            return False
        
        # Определяем направление (приоритет: вперед/назад, затем влево/вправо)
        step_x = 0.0
        step_y = 0.0
        
        if abs(fy) > abs(fx):
            # Доминирует отклонение вперед/назад
            if fy < -self.accel_threshold:
                # Падение назад - шаг назад
                step_x = -self._calculate_step_amplitude(abs(fy))
            elif fy > self.accel_threshold:
                # Падение вперед - шаг вперед
                step_x = self._calculate_step_amplitude(abs(fy))
        else:
            # Доминирует отклонение влево/вправо
            if fx > self.accel_threshold:
                # Падение вправо - шаг вправо
                step_y = self._calculate_step_amplitude(abs(fx))
            elif fx < -self.accel_threshold:
                # Падение влево - шаг влево
                step_y = -self._calculate_step_amplitude(abs(fx))
        
        if abs(step_x) < JOYSTICK_MOVE_THRESHOLD and abs(step_y) < JOYSTICK_MOVE_THRESHOLD:
            return False
        
        # Выполняем шаг
        self._make_step(step_x, step_y, magnitude)
        
        # Устанавливаем охлаждение
        self.cooldown_end_time = current_time + self.cooldown_duration
        self.last_step_time = current_time
        
        rospy.loginfo(f"AutoStabilization: Step executed (x={step_x:.4f}, y={step_y:.4f}, magnitude={magnitude:.3f} m/s²), cooldown={self.cooldown_duration}s")
        
        return True
    
    def _calculate_step_amplitude(self, accel_magnitude):
        """
        Вычисляет амплитуду шага на основе величины ускорения.
        
        Args:
            accel_magnitude: Величина ускорения
        
        Returns:
            float: Амплитуда шага
        """
        if accel_magnitude <= self.accel_threshold:
            return self.step_amplitude_base
        
        if accel_magnitude >= self.accel_max_for_max_step:
            return self.step_amplitude_max
        
        # Линейная интерполяция
        ratio = (accel_magnitude - self.accel_threshold) / (self.accel_max_for_max_step - self.accel_threshold)
        return self.step_amplitude_base + (self.step_amplitude_max - self.step_amplitude_base) * ratio
    
    def _calculate_step_speed(self, accel_magnitude):
        """
        Вычисляет скорость шага на основе величины ускорения.
        Больше ускорение = быстрее шаг.
        
        Args:
            accel_magnitude: Величина ускорения
        
        Returns:
            float: Множитель скорости (1.0 = нормальная, <1.0 = медленнее, >1.0 = быстрее)
        """
        if accel_magnitude <= self.accel_threshold:
            return SPEED_FACTOR_NORMAL
        
        if accel_magnitude >= self.accel_max_for_max_step:
            return SPEED_FACTOR_MIN  # Быстрый шаг (уменьшаем период на 30%)
        
        # Линейная интерполяция от нормальной до минимальной скорости
        ratio = (accel_magnitude - self.accel_threshold) / (self.accel_max_for_max_step - self.accel_threshold)
        speed_reduction = SPEED_FACTOR_NORMAL - SPEED_FACTOR_MIN
        return SPEED_FACTOR_NORMAL - ratio * speed_reduction
    
    def _make_step(self, step_x, step_y, accel_magnitude):
        """
        Выполняет шаг стабилизации.
        
        Args:
            step_x: Шаг вперед/назад (положительный = вперед)
            step_y: Шаг влево/вправо (положительный = вправо)
            accel_magnitude: Величина ускорения для расчета скорости
        """
        try:
            gait_param = self.gait_manager.get_gait_param()
            params = self.speed_params[self.speed_mode]
            period_time = list(params['period_time'])
            
            # Вычисляем скорость шага на основе ускорения
            speed_factor = self._calculate_step_speed(accel_magnitude)
            period_time[0] = int(period_time[0] * speed_factor)
            
            if self.speed_mode > 1:
                gait_param.update(params.get('gait_base', {}))
            
            gait_param['init_z_offset'] = self.init_z_offset
            
            # Выполняем один шаг
            self.gait_manager.set_step(
                period_time,
                step_x,
                step_y,
                0,  # angle_move_amplitude = 0
                gait_param,
                step_num=1  # Один шаг
            )
        except Exception as e:
            rospy.logwarn(f"AutoStabilization: Error making step: {e}")
    
    def reset(self):
        """Сбрасывает состояние стабилизации."""
        self.cooldown_end_time = 0
        self.last_step_time = 0
        self.filtered_accel = {'x': 0.0, 'y': 0.0}

