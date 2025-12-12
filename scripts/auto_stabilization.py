#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль автостабилизации робота в покое и при ходьбе.

СТАБИЛИЗАЦИЯ В ПОКОЕ:
- Выполняет корректирующие шаги при обнаружении нестабильности
- Динамически изменяемые параметры:
  * period_time[0] - скорость шага (через speed_factor, зависит от величины изменения ускорения)
  * step_amplitude - амплитуда шага (зависит от величины изменения ускорения)
  * arm_swap - размах рук для балансировки (зависит от величины изменения ускорения)

СТАБИЛИЗАЦИЯ ПРИ ХОДЬБЕ:
- Динамически корректирует параметры походки на основе IMU данных
- Не работает во время шагов стабилизации покоя (защита REST_STEP_PROTECTION_DURATION)
- Динамически изменяемые параметры:
  * init_roll_offset - крен (компенсация наклона влево/вправо, зависит от change_x)
  * init_pitch_offset - наклон вперед/назад (компенсация наклона, зависит от change_y)
  * init_y_offset - смещение по Y (компенсация смещения, зависит от change_y)
  * y_swap_amplitude - амплитуда обмена по Y (увеличивается при нестабильности)
  * z_swap_amplitude - амплитуда обмена по Z (увеличивается при нестабильности)
  * dsp_ratio - соотношение двойной опоры (увеличивается для большей стабильности)
  * step_fb_ratio - соотношение шага вперед/назад (корректируется при нестабильности)
- Все параметры фильтруются для плавности изменений (filter_alpha)
"""

# ========== КОНФИГУРАЦИЯ: СТАБИЛИЗАЦИЯ В ПОКОЕ ==========
REST_STABILIZATION_CONFIG = {
    # Пороги и параметры шага
    'accel_change_threshold': 0.03,  # Порог ИЗМЕНЕНИЯ ускорения для срабатывания (м/с²)
    'step_amplitude_base': 0.020,  # Базовая амплитуда шага (минимальная)
    'step_amplitude_max': 0.020,  # Максимальная амплитуда шага
    'accel_max_for_max_step': 1.0,  # Максимальное изменение ускорения для максимального шага
    'cooldown_duration': 2.5,  # Длительность охлаждения после шага (сек)
    
    # Маппинг изменения ускорения на направление шага
    # Формат: (ось, знак_изменения) -> (направление_шага_x, направление_шага_y)
    'step_direction_map': {
        # Изменение по оси X (влево/вправо)
        ('x', '+'): (None, 'left'),   # Ускорение увеличивается вправо -> шаг вправо
        ('x', '-'): (None, 'right'),    # Ускорение увеличивается влево -> шаг влево
        # Изменение по оси Y (вперед/назад)
        ('y', '+'): ('forward', None), # Ускорение увеличивается вперед -> шаг вперед
        ('y', '-'): ('backward', None), # Ускорение уменьшается (падаем назад) -> шаг назад
    },
    
    # Приоритет осей при одновременном изменении
    'axis_priority': {
        'y': 1,  # Ось Y (вперед/назад) имеет приоритет 1 (высший)
        'x': 2,  # Ось X (влево/вправо) имеет приоритет 2
    },
    
    # Параметры скорости шага
    'speed_factor_min': 0.2,  # Минимальный множитель скорости (быстрый шаг)
    'speed_factor_normal': 0.7,  # Нормальный множитель скорости
    
    # Динамический размах рук для балансировки
    'arm_swing_base': 20,  # Базовый размах рук (град) при малом изменении ускорения
    'arm_swing_max': 60,  # Максимальный размах рук (град) при большом изменении ускорения
    'arm_swing_threshold_low': 0.02,  # Порог изменения ускорения для базового размаха
    'arm_swing_threshold_high': 0.3,  # Порог изменения ускорения для максимального размаха
}

# ========== КОНФИГУРАЦИЯ: СТАБИЛИЗАЦИЯ ПРИ ХОДЬБЕ ==========
WALKING_STABILIZATION_CONFIG = {
    # Пороги для активации стабилизации при ходьбе
    'accel_change_threshold': 0.05,  # Порог изменения ускорения для корректировки (м/с²)
    'update_interval': 0.1,  # Интервал обновления параметров при ходьбе (сек)
    
    # Параметры динамической корректировки
    # init_roll_offset - крен (для компенсации наклона влево/вправо)
    'roll_offset_base': 0.0,  # Базовое значение крена (град)
    'roll_offset_max': 5.0,  # Максимальная корректировка крена (град)
    'roll_sensitivity': 2.0,  # Чувствительность к изменению ускорения по X
    
    # init_pitch_offset - наклон вперед/назад
    'pitch_offset_base': 0.0,  # Базовое значение наклона (град)
    'pitch_offset_max': 3.0,  # Максимальная корректировка наклона (град)
    'pitch_sensitivity': 1.5,  # Чувствительность к изменению ускорения по Y
    
    # init_y_offset - смещение по Y
    'y_offset_base': 0.0,  # Базовое смещение по Y (м)
    'y_offset_max': 0.01,  # Максимальная корректировка смещения (м)
    'y_offset_sensitivity': 0.5,  # Чувствительность к изменению ускорения
    
    # y_swap_amplitude - амплитуда обмена по Y
    'y_swap_base': 0.02,  # Базовая амплитуда обмена по Y (м)
    'y_swap_max': 0.03,  # Максимальная амплитуда обмена (м)
    'y_swap_sensitivity': 0.3,  # Чувствительность к изменению ускорения
    
    # z_swap_amplitude - амплитуда обмена по Z
    'z_swap_base': 0.006,  # Базовая амплитуда обмена по Z (м)
    'z_swap_max': 0.010,  # Максимальная амплитуда обмена (м)
    'z_swap_sensitivity': 0.2,  # Чувствительность к изменению ускорения
    
    # dsp_ratio - соотношение двойной опоры
    'dsp_ratio_base': 0.2,  # Базовое соотношение двойной опоры
    'dsp_ratio_max': 0.3,  # Максимальное соотношение (больше стабильности)
    'dsp_ratio_sensitivity': 0.1,  # Чувствительность к изменению ускорения
    
    # step_fb_ratio - соотношение шага вперед/назад
    'step_fb_ratio_base': 0.028,  # Базовое соотношение
    'step_fb_ratio_max': 0.035,  # Максимальное соотношение
    'step_fb_ratio_sensitivity': 0.15,  # Чувствительность к изменению ускорения
    
    # Фильтрация для плавности изменений
    'filter_alpha': 0.7,  # Коэффициент экспоненциального фильтра (0.0-1.0)
}

# ========== ОБЩИЕ ПАРАМЕТРЫ ==========
# Калибровка гравитации
CALIBRATION_SAMPLES = 30  # Количество образцов для калибровки
CALIBRATION_STABILITY_THRESHOLD = 0.1  # Порог стабильности для калибровки (м/с²)

# Фильтрация ускорений (для анализа)
FILTER_ALPHA = 0.8  # Коэффициент экспоненциального фильтра (0.0-1.0, меньше = больше сглаживание)

# Пороги для проверки движения от джойстика
JOYSTICK_MOVE_THRESHOLD = 0.001  # Минимальная амплитуда для определения движения

# История для анализа изменения ускорения
ACCEL_HISTORY_SIZE = 5  # Размер истории для анализа изменения ускорения

# Защита от конфликтов: время после шага стабилизации покоя, когда стабилизация при ходьбе не работает
REST_STEP_PROTECTION_DURATION = 3.0  # Длительность защиты (сек)
# ===================================

import rospy
import math


class AutoStabilization:
    """
    Класс для автоматической стабилизации робота в покое и при ходьбе.
    - Стабилизация в покое: выполняет корректирующие шаги при обнаружении нестабильности
    - Стабилизация при ходьбе: динамически корректирует параметры походки на основе IMU данных
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
        
        # Параметры стабилизации в покое (из конфига)
        self.rest_config = REST_STABILIZATION_CONFIG
        self.enabled = True
        self.rest_enabled = True  # Стабилизация в покое
        self.walking_enabled = True  # Стабилизация при ходьбе
        
        # Состояние стабилизации в покое
        self.last_rest_step_time = 0
        self.rest_cooldown_end_time = 0
        
        # Состояние стабилизации при ходьбе
        self.last_walking_update_time = 0
        self.walking_filtered_params = {}  # Отфильтрованные параметры для плавности
        
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
        
        rospy.loginfo("AutoStabilization module initialized (Rest + Walking)")
    
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
        Поддерживает два режима: стабилизация в покое и стабилизация при ходьбе.
        
        Args:
            ax: Ускорение по оси X (влево/вправо)
            ay: Ускорение по оси Y (вперед/назад)
            robot_state: Состояние робота ('stand', 'lie_to_stand', 'recline_to_stand')
            status: Статус движения ('stop', 'move')
            x_move_amp: Амплитуда движения по X от джойстика
            y_move_amp: Амплитуда движения по Y от джойстика
            angle_move_amp: Амплитуда поворота от джойстика
        
        Returns:
            bool: True если был выполнен шаг стабилизации покоя
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
        
        # Фильтрация ускорений (общая для обоих режимов)
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
        
        # Определяем режим стабилизации на основе статуса
        rest_step_executed = False
        if status == 'stop' and self.rest_enabled:
            # СТАБИЛИЗАЦИЯ В ПОКОЕ
            rest_step_executed = self._process_rest_stabilization(
                current_time, robot_state, x_move_amp, y_move_amp, angle_move_amp
            )
        elif status == 'move' and self.walking_enabled:
            # СТАБИЛИЗАЦИЯ ПРИ ХОДЬБЕ
            # Не работает во время шагов стабилизации покоя
            if current_time - self.last_rest_step_time > REST_STEP_PROTECTION_DURATION:
                self._process_walking_stabilization(current_time)
        
        return rest_step_executed
    
    def _process_rest_stabilization(self, current_time, robot_state, x_move_amp, y_move_amp, angle_move_amp):
        """
        Обрабатывает стабилизацию в покое.
        
        Returns:
            bool: True если был выполнен шаг стабилизации
        """
        config = self.rest_config
        
        # Проверка охлаждения
        if current_time < self.rest_cooldown_end_time:
            return False
        
        # Проверка условий для стабилизации
        # Работает только когда робот стоит и нет команд от джойстика
        if (robot_state != 'stand' or
            abs(x_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
            abs(y_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
            abs(angle_move_amp) > JOYSTICK_MOVE_THRESHOLD):
            return False
        
        # Нужно минимум данных для анализа изменения
        if len(self.accel_history['x']) < 3:
            return False
        
        # Вычисляем ИЗМЕНЕНИЕ ускорения (производную)
        change_x = self.accel_history['x'][-1] - self.accel_history['x'][-2]
        change_y = self.accel_history['y'][-1] - self.accel_history['y'][-2]
        
        # Вычисляем величину изменения
        change_magnitude = math.sqrt(change_x * change_x + change_y * change_y)
        
        # Проверка порога ИЗМЕНЕНИЯ
        if change_magnitude < config['accel_change_threshold']:
            return False
        
        # Проверка на стабильность: если ускорение не меняется (постоянное смещение), не делаем шаг
        if len(self.accel_history['x']) >= 3:
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
        
        # Определяем направление шага
        step_x = 0.0
        step_y = 0.0
        
        # Проверяем изменения по каждой оси
        axes_changes = []
        axis_priority = config['axis_priority']
        if abs(change_x) >= config['accel_change_threshold']:
            sign = '+' if change_x > 0 else '-'
            axes_changes.append(('x', sign, abs(change_x), axis_priority['x']))
        if abs(change_y) >= config['accel_change_threshold']:
            sign = '+' if change_y > 0 else '-'
            axes_changes.append(('y', sign, abs(change_y), axis_priority['y']))
        
        if not axes_changes:
            return False
        
        # Сортируем по приоритету (меньший номер = выше приоритет)
        axes_changes.sort(key=lambda x: x[3])
        
        # Берем ось с наивысшим приоритетом
        selected_axis, selected_sign, selected_magnitude, _ = axes_changes[0]
        
        # Получаем направление шага из конфигурации
        step_direction_map = config['step_direction_map']
        direction_key = (selected_axis, selected_sign)
        if direction_key not in step_direction_map:
            rospy.logwarn(f"AutoStabilization: No direction mapping for {direction_key}")
            return False
        
        step_dir_x, step_dir_y = step_direction_map[direction_key]
        
        # Вычисляем амплитуду шага
        step_amplitude = self._calculate_rest_step_amplitude(change_magnitude, config)
        
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
        self._make_rest_step(step_x, step_y, change_magnitude, config)
        
        # Устанавливаем охлаждение
        self.rest_cooldown_end_time = current_time + config['cooldown_duration']
        self.last_rest_step_time = current_time
        
        # Очищаем историю после шага для предотвращения накопления ошибок
        self.accel_history = {'x': [], 'y': []}
        
        rospy.loginfo(f"AutoStabilization [REST]: Step executed (x={step_x:.4f}, y={step_y:.4f}, change={change_magnitude:.3f} m/s²), cooldown={config['cooldown_duration']}s")
        
        return True
    
    def _process_walking_stabilization(self, current_time):
        """
        Обрабатывает стабилизацию при ходьбе.
        Динамически корректирует параметры походки на основе IMU данных.
        """
        config = WALKING_STABILIZATION_CONFIG
        
        # Проверка интервала обновления
        if current_time - self.last_walking_update_time < config['update_interval']:
            return
        
        # Нужно минимум данных для анализа
        if len(self.accel_history['x']) < 2:
            return
        
        # Вычисляем изменение ускорения
        change_x = self.accel_history['x'][-1] - self.accel_history['x'][-2]
        change_y = self.accel_history['y'][-1] - self.accel_history['y'][-2]
        change_magnitude = math.sqrt(change_x * change_x + change_y * change_y)
        
        # Проверка порога
        if change_magnitude < config['accel_change_threshold']:
            return
        
        # Получаем текущие параметры походки
        gait_param = self.gait_manager.get_gait_param()
        params = self.speed_params[self.speed_mode]
        
        # Вычисляем корректировки параметров
        corrections = self._calculate_walking_corrections(change_x, change_y, change_magnitude, config)
        
        # Применяем фильтрацию для плавности
        filter_alpha = config['filter_alpha']
        for key, value in corrections.items():
            if key not in self.walking_filtered_params:
                self.walking_filtered_params[key] = value
            else:
                self.walking_filtered_params[key] = (
                    filter_alpha * value + (1 - filter_alpha) * self.walking_filtered_params[key]
                )
        
        # Применяем корректировки к параметрам походки
        if self.speed_mode > 1:
            gait_param.update(params.get('gait_base', {}))
        
        # Применяем отфильтрованные корректировки
        if 'init_roll_offset' in self.walking_filtered_params:
            gait_param['init_roll_offset'] = self.walking_filtered_params['init_roll_offset']
        if 'init_pitch_offset' in self.walking_filtered_params:
            gait_param['init_pitch_offset'] = self.walking_filtered_params['init_pitch_offset']
        if 'init_y_offset' in self.walking_filtered_params:
            gait_param['init_y_offset'] = self.walking_filtered_params['init_y_offset']
        if 'y_swap_amplitude' in self.walking_filtered_params:
            gait_param['y_swap_amplitude'] = self.walking_filtered_params['y_swap_amplitude']
        if 'z_swap_amplitude' in self.walking_filtered_params:
            gait_param['z_swap_amplitude'] = self.walking_filtered_params['z_swap_amplitude']
        if 'dsp_ratio' in self.walking_filtered_params:
            gait_param['dsp_ratio'] = self.walking_filtered_params['dsp_ratio']
        if 'step_fb_ratio' in self.walking_filtered_params:
            gait_param['step_fb_ratio'] = self.walking_filtered_params['step_fb_ratio']
        
        # Обновляем параметры без движения (только корректировка походки)
        period_time = list(params['period_time'])
        self.gait_manager.update_param(
            period_time,
            0, 0, 0,  # Без движения
            gait_param,
            step_num=0
        )
        
        self.last_walking_update_time = current_time
        
        rospy.logdebug(f"AutoStabilization [WALKING]: Corrections applied (change={change_magnitude:.3f} m/s²)")
    
    def _calculate_rest_step_amplitude(self, change_magnitude, config):
        """
        Вычисляет амплитуду шага стабилизации в покое на основе величины ИЗМЕНЕНИЯ ускорения.
        
        Args:
            change_magnitude: Величина изменения ускорения
            config: Конфигурация стабилизации в покое
        
        Returns:
            float: Амплитуда шага
        """
        threshold = config['accel_change_threshold']
        base = config['step_amplitude_base']
        max_amp = config['step_amplitude_max']
        max_for_max = config['accel_max_for_max_step']
        
        if change_magnitude <= threshold:
            amplitude = base
        elif change_magnitude >= max_for_max:
            amplitude = max_amp
        else:
            # Линейная интерполяция между базовой и максимальной амплитудой
            ratio = (change_magnitude - threshold) / (max_for_max - threshold)
            amplitude = base + (max_amp - base) * ratio
        
        # Гарантируем, что амплитуда в заданном диапазоне
        amplitude = max(base, min(max_amp, amplitude))
        return amplitude
    
    def _calculate_rest_step_speed(self, change_magnitude, config):
        """
        Вычисляет скорость шага стабилизации в покое на основе величины ИЗМЕНЕНИЯ ускорения.
        Больше изменение = быстрее шаг.
        
        Args:
            change_magnitude: Величина изменения ускорения
            config: Конфигурация стабилизации в покое
        
        Returns:
            float: Множитель скорости
        """
        threshold = config['accel_change_threshold']
        normal = config['speed_factor_normal']
        min_factor = config['speed_factor_min']
        max_for_max = config['accel_max_for_max_step']
        
        if change_magnitude <= threshold:
            return normal
        
        if change_magnitude >= max_for_max:
            return min_factor  # Быстрый шаг
        
        # Линейная интерполяция от нормальной до минимальной скорости
        ratio = (change_magnitude - threshold) / (max_for_max - threshold)
        speed_reduction = normal - min_factor
        return normal - ratio * speed_reduction
    
    def _calculate_rest_arm_swing(self, change_magnitude, config):
        """
        Вычисляет размах рук для стабилизации в покое на основе величины изменения ускорения.
        
        Args:
            change_magnitude: Величина изменения ускорения
            config: Конфигурация стабилизации в покое
        
        Returns:
            float: Размах рук в градусах
        """
        base = config['arm_swing_base']
        max_swing = config['arm_swing_max']
        threshold_low = config['arm_swing_threshold_low']
        threshold_high = config['arm_swing_threshold_high']
        
        if change_magnitude <= threshold_low:
            return base
        elif change_magnitude >= threshold_high:
            return max_swing
        else:
            # Линейная интерполяция между базовым и максимальным размахом
            ratio = (change_magnitude - threshold_low) / (threshold_high - threshold_low)
            return base + (max_swing - base) * ratio
    
    def _make_rest_step(self, step_x, step_y, change_magnitude, config):
        """
        Выполняет шаг стабилизации в покое.
        
        Args:
            step_x: Шаг вперед/назад (положительный = вперед)
            step_y: Шаг влево/вправо (положительный = вправо)
            change_magnitude: Величина изменения ускорения для расчета скорости
            config: Конфигурация стабилизации в покое
        """
        try:
            gait_param = self.gait_manager.get_gait_param()
            params = self.speed_params[self.speed_mode]
            period_time = list(params['period_time'])
            
            # Вычисляем скорость шага на основе изменения ускорения
            speed_factor = self._calculate_rest_step_speed(change_magnitude, config)
            period_time[0] = int(period_time[0] * speed_factor)
            
            if self.speed_mode > 1:
                gait_param.update(params.get('gait_base', {}))
            
            # Используем body_height вместо init_z_offset (единый источник истины)
            gait_param['body_height'] = self.init_z_offset
            
            # Вычисляем динамический размах рук для балансировки
            arm_swing = self._calculate_rest_arm_swing(change_magnitude, config)
            
            # Выполняем один шаг с динамическим размахом рук
            self.gait_manager.set_step(
                period_time,
                step_x,
                step_y,
                0,  # angle_move_amplitude = 0
                gait_param,
                arm_swap=arm_swing,  # Динамический размах рук
                step_num=1  # Один шаг
            )
        except Exception as e:
            rospy.logwarn(f"AutoStabilization [REST]: Error making step: {e}")
    
    def _calculate_walking_corrections(self, change_x, change_y, change_magnitude, config):
        """
        Вычисляет корректировки параметров походки для стабилизации при ходьбе.
        
        Args:
            change_x: Изменение ускорения по оси X
            change_y: Изменение ускорения по оси Y
            change_magnitude: Величина изменения ускорения
            config: Конфигурация стабилизации при ходьбе
        
        Returns:
            dict: Словарь с корректировками параметров
        """
        corrections = {}
        
        # Нормализуем изменение для расчета корректировок (0.0 - 1.0)
        threshold = config['accel_change_threshold']
        normalized_change = min(1.0, change_magnitude / (threshold * 5))  # Масштабируем
        
        # init_roll_offset - крен (компенсация наклона влево/вправо)
        roll_base = config['roll_offset_base']
        roll_max = config['roll_offset_max']
        roll_sens = config['roll_sensitivity']
        roll_correction = roll_base + math.copysign(
            normalized_change * roll_max * roll_sens * abs(change_x) / change_magnitude if change_magnitude > 0 else 0,
            change_x
        )
        corrections['init_roll_offset'] = max(-roll_max, min(roll_max, roll_correction))
        
        # init_pitch_offset - наклон вперед/назад
        pitch_base = config['pitch_offset_base']
        pitch_max = config['pitch_offset_max']
        pitch_sens = config['pitch_sensitivity']
        pitch_correction = pitch_base + math.copysign(
            normalized_change * pitch_max * pitch_sens * abs(change_y) / change_magnitude if change_magnitude > 0 else 0,
            change_y
        )
        corrections['init_pitch_offset'] = max(-pitch_max, min(pitch_max, pitch_correction))
        
        # init_y_offset - смещение по Y
        y_offset_base = config['y_offset_base']
        y_offset_max = config['y_offset_max']
        y_offset_sens = config['y_offset_sensitivity']
        y_offset_correction = y_offset_base + math.copysign(
            normalized_change * y_offset_max * y_offset_sens,
            change_y
        )
        corrections['init_y_offset'] = max(-y_offset_max, min(y_offset_max, y_offset_correction))
        
        # y_swap_amplitude - амплитуда обмена по Y
        y_swap_base = config['y_swap_base']
        y_swap_max = config['y_swap_max']
        y_swap_sens = config['y_swap_sensitivity']
        y_swap_correction = y_swap_base + normalized_change * (y_swap_max - y_swap_base) * y_swap_sens
        corrections['y_swap_amplitude'] = max(y_swap_base, min(y_swap_max, y_swap_correction))
        
        # z_swap_amplitude - амплитуда обмена по Z
        z_swap_base = config['z_swap_base']
        z_swap_max = config['z_swap_max']
        z_swap_sens = config['z_swap_sensitivity']
        z_swap_correction = z_swap_base + normalized_change * (z_swap_max - z_swap_base) * z_swap_sens
        corrections['z_swap_amplitude'] = max(z_swap_base, min(z_swap_max, z_swap_correction))
        
        # dsp_ratio - соотношение двойной опоры (больше = стабильнее)
        dsp_base = config['dsp_ratio_base']
        dsp_max = config['dsp_ratio_max']
        dsp_sens = config['dsp_ratio_sensitivity']
        dsp_correction = dsp_base + normalized_change * (dsp_max - dsp_base) * dsp_sens
        corrections['dsp_ratio'] = max(dsp_base, min(dsp_max, dsp_correction))
        
        # step_fb_ratio - соотношение шага вперед/назад
        step_fb_base = config['step_fb_ratio_base']
        step_fb_max = config['step_fb_ratio_max']
        step_fb_sens = config['step_fb_ratio_sensitivity']
        step_fb_correction = step_fb_base + normalized_change * (step_fb_max - step_fb_base) * step_fb_sens
        corrections['step_fb_ratio'] = max(step_fb_base, min(step_fb_max, step_fb_correction))
        
        return corrections
    
    def reset(self):
        """Сбрасывает состояние стабилизации."""
        self.rest_cooldown_end_time = 0
        self.last_rest_step_time = 0
        self.last_walking_update_time = 0
        self.filtered_accel = {'x': 0.0, 'y': 0.0}
        self.accel_history = {'x': [], 'y': []}
        self.walking_filtered_params = {}

