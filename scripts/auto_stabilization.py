#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль автостабилизации робота в покое.
Упрощенная логика: 1 шаг с длиной и скоростью зависящей от ускорения, затем охлаждение.
"""

# ========== КОНФИГУРАЦИЯ ==========
# Параметры автостабилизации
ACCEL_CHANGE_THRESHOLD = 0.03  # Порог ИЗМЕНЕНИЯ ускорения для срабатывания (м/с²) - увеличен для уменьшения ложных срабатываний
STEP_AMPLITUDE_BASE = 0.020  # Базовая амплитуда шага (минимальная)
STEP_AMPLITUDE_MAX = 0.020  # Максимальная амплитуда шага
ACCEL_MAX_FOR_MAX_STEP = 1.0  # Максимальное изменение ускорения для максимального шага
COOLDOWN_DURATION = 2.5 # Длительность охлаждения после шага (сек)

# Маппинг изменения ускорения на направление шага
# Определяет, в какую сторону делать шаг при изменении ускорения по каждой оси
# Формат: (ось, знак_изменения) -> (направление_шага_x, направление_шага_y)
# где:
#   ось: 'x' (влево/вправо) или 'y' (вперед/назад)
#   знак_изменения: '+' (положительное изменение) или '-' (отрицательное изменение)
#   направление_шага_x: 'forward' (вперед, +), 'backward' (назад, -), или None
#   направление_шага_y: 'right' (вправо, +), 'left' (влево, -), или None
STEP_DIRECTION_MAP = {
    # Изменение по оси X (влево/вправо)
    ('x', '+'): (None, 'left'),   # Ускорение увеличивается вправо -> шаг вправо
    ('x', '-'): (None, 'right'),    # Ускорение увеличивается влево -> шаг влево
    # Изменение по оси Y (вперед/назад)
    ('y', '+'): ('forward', None), # Ускорение увеличивается вперед -> шаг вперед
    ('y', '-'): ('backward', None), # Ускорение уменьшается (падаем назад) -> шаг назад
}

# Приоритет осей при одновременном изменении (какая ось важнее)
# Если обе оси превышают порог, используется ось с более высоким приоритетом
AXIS_PRIORITY = {
    'y': 1,  # Ось Y (вперед/назад) имеет приоритет 1 (высший)
    'x': 2,  # Ось X (влево/вправо) имеет приоритет 2
}

# Калибровка гравитации
CALIBRATION_SAMPLES = 30  # Количество образцов для калибровки
CALIBRATION_STABILITY_THRESHOLD = 0.1  # Порог стабильности для калибровки (м/с²)

# Фильтрация ускорений
FILTER_ALPHA = 0.8  # Коэффициент экспоненциального фильтра (0.0-1.0, меньше = больше сглаживание)

# Пороги для проверки движения от джойстика
JOYSTICK_MOVE_THRESHOLD = 0.001  # Минимальная амплитуда для определения движения

# Параметры скорости шага
SPEED_FACTOR_MIN = 0.2  # Минимальный множитель скорости (быстрый шаг)
SPEED_FACTOR_NORMAL = 0.4 # Нормальный множитель скорости

# История для анализа изменения ускорения
ACCEL_HISTORY_SIZE = 5  # Размер истории для анализа изменения ускорения
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
        self.accel_change_threshold = ACCEL_CHANGE_THRESHOLD
        self.step_amplitude_base = STEP_AMPLITUDE_BASE
        self.step_amplitude_max = STEP_AMPLITUDE_MAX
        self.accel_max_for_max_step = ACCEL_MAX_FOR_MAX_STEP
        
        # Состояние
        self.last_step_time = 0
        self.cooldown_end_time = 0
        self.cooldown_duration = COOLDOWN_DURATION
        
        # Калибровка гравитации (используем преобразованные значения)
        self.gravity_base_y = None  # Будет установлено при калибровке
        self.gravity_calibrated = False
        self.calibration_samples = CALIBRATION_SAMPLES
        self.calibration_data = []
        self.calibration_in_progress = True
        
        # Фильтрация ускорений
        self.filter_alpha = FILTER_ALPHA
        self.filtered_accel = {'x': 0.0, 'y': 0.0}
        
        # История ускорений для анализа изменения
        self.accel_history = {'x': [], 'y': []}
        self.accel_history_size = ACCEL_HISTORY_SIZE
        
        rospy.loginfo("AutoStabilization module initialized")
    
    def update_calibration(self, ax, ay):
        """
        Обновляет калибровку базового значения гравитации.
        Проверяет стабильность данных перед калибровкой.
        
        Args:
            ax: Ускорение по оси X (преобразованное)
            ay: Ускорение по оси Y (преобразованное)
        """
        if not self.calibration_in_progress:
            return
        
        self.calibration_data.append({'x': ax, 'y': ay})
        
        if len(self.calibration_data) > self.calibration_samples:
            self.calibration_data.pop(0)
        
        if len(self.calibration_data) >= self.calibration_samples:
            # Проверяем стабильность данных
            y_values = [d['y'] for d in self.calibration_data]
            y_mean = sum(y_values) / len(y_values)
            y_variance = sum((y - y_mean) ** 2 for y in y_values) / len(y_values)
            y_std = math.sqrt(y_variance)
            
            # Если данные стабильны (низкое стандартное отклонение), калибруем
            if y_std < CALIBRATION_STABILITY_THRESHOLD:
                self.gravity_base_y = y_mean
                self.gravity_calibrated = True
                self.calibration_in_progress = False
                self.calibration_data = []
                
                rospy.loginfo(f"AutoStabilization calibration complete. Gravity base Y: {self.gravity_base_y:.3f} m/s² (std: {y_std:.3f})")
            else:
                # Данные нестабильны, продолжаем сбор
                rospy.logdebug(f"Calibration: data not stable (std: {y_std:.3f} > {CALIBRATION_STABILITY_THRESHOLD}), continuing...")
    
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
        if self.gravity_base_y is not None:
            ay_deviation = ay - self.gravity_base_y
        else:
            ay_deviation = ay  # Если еще не откалибровано, используем как есть
        
        self.filtered_accel['x'] = self.filter_alpha * ax + (1 - self.filter_alpha) * self.filtered_accel['x']
        self.filtered_accel['y'] = self.filter_alpha * ay_deviation + (1 - self.filter_alpha) * self.filtered_accel['y']
        
        # Добавляем в историю для анализа изменения
        self.accel_history['x'].append(self.filtered_accel['x'])
        self.accel_history['y'].append(self.filtered_accel['y'])
        
        # Ограничиваем размер истории
        if len(self.accel_history['x']) > self.accel_history_size:
            self.accel_history['x'].pop(0)
            self.accel_history['y'].pop(0)
        
        # Нужно минимум данных для анализа изменения
        if len(self.accel_history['x']) < 3:
            return False
        
        # Вычисляем ИЗМЕНЕНИЕ ускорения (производную)
        # Берем разницу между последним и предыдущим значением
        change_x = self.accel_history['x'][-1] - self.accel_history['x'][-2]
        change_y = self.accel_history['y'][-1] - self.accel_history['y'][-2]
        
        # Вычисляем величину изменения
        change_magnitude = math.sqrt(change_x * change_x + change_y * change_y)
        
        # Проверка порога ИЗМЕНЕНИЯ (не абсолютного значения!)
        if change_magnitude < self.accel_change_threshold:
            return False
        
        # Проверка на стабильность: если ускорение не меняется (постоянное смещение), не делаем шаг
        # Проверяем, что изменение действительно значительное, а не просто шум
        if len(self.accel_history['x']) >= 3:
            # Проверяем, что знак изменения стабилен (не качание)
            # Берем последние 3 изменения
            changes_x = []
            changes_y = []
            for i in range(len(self.accel_history['x'])-1, max(0, len(self.accel_history['x'])-4), -1):
                if i > 0:
                    changes_x.append(self.accel_history['x'][i] - self.accel_history['x'][i-1])
            for i in range(len(self.accel_history['y'])-1, max(0, len(self.accel_history['y'])-4), -1):
                if i > 0:
                    changes_y.append(self.accel_history['y'][i] - self.accel_history['y'][i-1])
            
            # Если знаки меняются - это качание, не делаем шаг
            if changes_x and len(set(1 if c >= 0 else -1 for c in changes_x)) > 1:
                return False
            if changes_y and len(set(1 if c >= 0 else -1 for c in changes_y)) > 1:
                return False
        
        # Определяем направление шага на основе конфигурации STEP_DIRECTION_MAP
        step_x = 0.0
        step_y = 0.0
        
        # Проверяем изменения по каждой оси
        axes_changes = []
        if abs(change_x) >= self.accel_change_threshold:
            sign = '+' if change_x > 0 else '-'
            axes_changes.append(('x', sign, abs(change_x), AXIS_PRIORITY['x']))
        if abs(change_y) >= self.accel_change_threshold:
            sign = '+' if change_y > 0 else '-'
            axes_changes.append(('y', sign, abs(change_y), AXIS_PRIORITY['y']))
        
        if not axes_changes:
            return False
        
        # Сортируем по приоритету (меньший номер = выше приоритет)
        axes_changes.sort(key=lambda x: x[3])
        
        # Берем ось с наивысшим приоритетом
        selected_axis, selected_sign, selected_magnitude, _ = axes_changes[0]
        
        # Получаем направление шага из конфигурации
        direction_key = (selected_axis, selected_sign)
        if direction_key not in STEP_DIRECTION_MAP:
            rospy.logwarn(f"AutoStabilization: No direction mapping for {direction_key}")
            return False
        
        step_dir_x, step_dir_y = STEP_DIRECTION_MAP[direction_key]
        
        # Вычисляем амплитуду шага в диапазоне [STEP_AMPLITUDE_BASE, STEP_AMPLITUDE_MAX]
        step_amplitude = self._calculate_step_amplitude(change_magnitude)
        
        # Применяем направление
        if step_dir_x == 'forward':
            step_x = step_amplitude
        elif step_dir_x == 'backward':
            step_x = -step_amplitude
        
        if step_dir_y == 'right':
            step_y = step_amplitude
        elif step_dir_y == 'left':
            step_y = -step_amplitude
        
        if abs(step_x) < JOYSTICK_MOVE_THRESHOLD and abs(step_y) < JOYSTICK_MOVE_THRESHOLD:
            return False
        
        # Выполняем шаг
        self._make_step(step_x, step_y, change_magnitude)
        
        # Устанавливаем охлаждение
        self.cooldown_end_time = current_time + self.cooldown_duration
        self.last_step_time = current_time
        
        # Очищаем историю после шага для предотвращения накопления ошибок
        self.accel_history = {'x': [], 'y': []}
        
        rospy.loginfo(f"AutoStabilization: Step executed (x={step_x:.4f}, y={step_y:.4f}, change={change_magnitude:.3f} m/s²), cooldown={self.cooldown_duration}s")
        
        return True
    
    def _calculate_step_amplitude(self, change_magnitude):
        """
        Вычисляет амплитуду шага на основе величины ИЗМЕНЕНИЯ ускорения.
        Гарантирует, что результат находится в диапазоне [STEP_AMPLITUDE_BASE, STEP_AMPLITUDE_MAX].
        
        Args:
            change_magnitude: Величина изменения ускорения
        
        Returns:
            float: Амплитуда шага в диапазоне [STEP_AMPLITUDE_BASE, STEP_AMPLITUDE_MAX]
        """
        if change_magnitude <= self.accel_change_threshold:
            amplitude = self.step_amplitude_base
        elif change_magnitude >= self.accel_max_for_max_step:
            amplitude = self.step_amplitude_max
        else:
            # Линейная интерполяция между базовой и максимальной амплитудой
            ratio = (change_magnitude - self.accel_change_threshold) / (self.accel_max_for_max_step - self.accel_change_threshold)
            amplitude = self.step_amplitude_base + (self.step_amplitude_max - self.step_amplitude_base) * ratio
        
        # Гарантируем, что амплитуда в заданном диапазоне
        amplitude = max(self.step_amplitude_base, min(self.step_amplitude_max, amplitude))
        
        return amplitude
    
    def _calculate_step_speed(self, change_magnitude):
        """
        Вычисляет скорость шага на основе величины ИЗМЕНЕНИЯ ускорения.
        Больше изменение = быстрее шаг.
        
        Args:
            change_magnitude: Величина изменения ускорения
        
        Returns:
            float: Множитель скорости (1.0 = нормальная, <1.0 = медленнее, >1.0 = быстрее)
        """
        if change_magnitude <= self.accel_change_threshold:
            return SPEED_FACTOR_NORMAL
        
        if change_magnitude >= self.accel_max_for_max_step:
            return SPEED_FACTOR_MIN  # Быстрый шаг (уменьшаем период на 30%)
        
        # Линейная интерполяция от нормальной до минимальной скорости
        ratio = (change_magnitude - self.accel_change_threshold) / (self.accel_max_for_max_step - self.accel_change_threshold)
        speed_reduction = SPEED_FACTOR_NORMAL - SPEED_FACTOR_MIN
        return SPEED_FACTOR_NORMAL - ratio * speed_reduction
    
    def _make_step(self, step_x, step_y, change_magnitude):
        """
        Выполняет шаг стабилизации.
        
        Args:
            step_x: Шаг вперед/назад (положительный = вперед)
            step_y: Шаг влево/вправо (положительный = вправо)
            change_magnitude: Величина изменения ускорения для расчета скорости
        """
        try:
            gait_param = self.gait_manager.get_gait_param()
            params = self.speed_params[self.speed_mode]
            period_time = list(params['period_time'])
            
            # Вычисляем скорость шага на основе изменения ускорения
            speed_factor = self._calculate_step_speed(change_magnitude)
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
        self.accel_history = {'x': [], 'y': []}

