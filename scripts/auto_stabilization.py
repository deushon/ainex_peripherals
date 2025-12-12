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
- Работает независимо от стабилизации покоя
- Динамически изменяемые параметры:
  * period_time[0] - скорость шага (уменьшается для большей стабильности)
  * step_fb_ratio - дистанция шага (уменьшается для большей стабильности)
  * init_roll_offset, init_pitch_offset, init_y_offset, y_swap_amplitude, z_swap_amplitude, dsp_ratio - опционально
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
    'step_direction_map': {
        ('x', '+'): (None, 'left'),
        ('x', '-'): (None, 'right'),
        ('y', '+'): ('forward', None),
        ('y', '-'): ('backward', None),
    },
    
    # Приоритет осей при одновременном изменении
    'axis_priority': {
        'y': 1,
        'x': 2,
    },
    
    # Параметры скорости шага
    'speed_factor_min': 0.2,
    'speed_factor_normal': 0.7,
    
    # Динамический размах рук для балансировки
    'arm_swing_base': 20,
    'arm_swing_max': 60,
    'arm_swing_threshold_low': 0.02,
    'arm_swing_threshold_high': 0.3,
}

# ========== КОНФИГУРАЦИЯ: СТАБИЛИЗАЦИЯ ПРИ ХОДЬБЕ ==========
WALKING_STABILIZATION_CONFIG = {
    # Пороги для активации стабилизации при ходьбе
    'accel_change_threshold': 0.05,
    'update_interval': 0.1,
    
    # Флаги включения/выключения параметров корректировки
    'enable_period_time': False, #ТРайминги выполненения шага, снижение в большинстве своем ухудшает динамику
    'enable_step_fb_ratio': False,
    'enable_roll_offset': False,
    'enable_pitch_offset': True,
    'enable_y_offset': False,
    'enable_y_swap': False,
    'enable_z_swap': False,
    'enable_dsp_ratio': False,
    
    # Параметры динамической корректировки
    'period_time_reduction_max': 0.3,
    'period_time_sensitivity': 1.0,
    'step_fb_ratio_reduction_max': 0.2,
    'step_fb_ratio_sensitivity': 0.15,
    'roll_offset_base': 0.0,
    'roll_offset_max': 5.0,
    'roll_sensitivity': 2.0,
    'pitch_offset_base': 0.0,
    'pitch_offset_max': 3.0,
    'pitch_sensitivity': 1.5,
    'y_offset_base': 0.0,
    'y_offset_max': 0.01,
    'y_offset_sensitivity': 0.5,
    'y_swap_base': 0.02,
    'y_swap_max': 0.03,
    'y_swap_sensitivity': 0.3,
    'z_swap_base': 0.006,
    'z_swap_max': 0.010,
    'z_swap_sensitivity': 0.2,
    'dsp_ratio_increase_max': 0.1,
    'dsp_ratio_sensitivity': 0.1,
    'filter_alpha': 0.7,
}

# ========== ОБЩИЕ ПАРАМЕТРЫ ==========
CALIBRATION_SAMPLES = 30
CALIBRATION_STABILITY_THRESHOLD = 0.1
FILTER_ALPHA = 0.8
JOYSTICK_MOVE_THRESHOLD = 0.001
ACCEL_HISTORY_SIZE = 5
# ===================================

import rospy
import math


class AutoStabilization:
    """Класс для автоматической стабилизации робота в покое и при ходьбе."""
    
    def __init__(self, gait_manager, speed_params, speed_mode, init_z_offset):
        self.gait_manager = gait_manager
        self.speed_params = speed_params
        self.speed_mode = speed_mode
        self.init_z_offset = init_z_offset
        
        self.rest_config = REST_STABILIZATION_CONFIG
        self.enabled = True
        self.rest_enabled = True
        self.walking_enabled = True
        
        self.last_rest_step_time = 0
        self.rest_cooldown_end_time = 0
        self.last_walking_update_time = 0
        self.walking_filtered_params = {}
        
        self.gravity_base_y = None
        self.gravity_calibrated = False
        self.calibration_samples = CALIBRATION_SAMPLES
        self.calibration_data = []
        self.calibration_in_progress = True
        
        self.filter_alpha = FILTER_ALPHA
        self.filtered_accel = {'x': 0.0, 'y': 0.0}
        self.accel_history = {'x': [], 'y': []}
        self.accel_history_size = ACCEL_HISTORY_SIZE
        
        rospy.loginfo("AutoStabilization module initialized (Rest + Walking)")
    
    def update_calibration(self, ax, ay):
        """Обновляет калибровку базового значения гравитации."""
        if not self.calibration_in_progress:
            return
        
        self.calibration_data.append({'x': ax, 'y': ay})
        
        if len(self.calibration_data) > self.calibration_samples:
            self.calibration_data.pop(0)
        
        if len(self.calibration_data) >= self.calibration_samples:
            y_values = [d['y'] for d in self.calibration_data]
            y_mean = sum(y_values) / len(y_values)
            y_variance = sum((y - y_mean) ** 2 for y in y_values) / len(y_values)
            y_std = math.sqrt(y_variance)
            
            if y_std < CALIBRATION_STABILITY_THRESHOLD:
                self.gravity_base_y = y_mean
                self.gravity_calibrated = True
                self.calibration_in_progress = False
                self.calibration_data = []
                rospy.loginfo(f"AutoStabilization calibration complete. Gravity base Y: {self.gravity_base_y:.3f} m/s² (std: {y_std:.3f})")
            else:
                rospy.logdebug(f"Calibration: data not stable (std: {y_std:.3f} > {CALIBRATION_STABILITY_THRESHOLD}), continuing...")
    
    def process(self, ax, ay, robot_state, status, x_move_amp, y_move_amp, angle_move_amp):
        """Обрабатывает данные ускорения и выполняет стабилизацию при необходимости."""
        if not self.enabled:
            return False
        
        current_time = rospy.get_time()
        
        if self.calibration_in_progress:
            self.update_calibration(ax, ay)
            return False
        
        if not self.gravity_calibrated:
            return False
        
        # Фильтрация ускорений
        if self.gravity_base_y is not None:
            ay_deviation = ay - self.gravity_base_y
        else:
            ay_deviation = ay
        
        self.filtered_accel['x'] = self.filter_alpha * ax + (1 - self.filter_alpha) * self.filtered_accel['x']
        self.filtered_accel['y'] = self.filter_alpha * ay_deviation + (1 - self.filter_alpha) * self.filtered_accel['y']
        
        # Добавляем в историю
        self.accel_history['x'].append(self.filtered_accel['x'])
        self.accel_history['y'].append(self.filtered_accel['y'])
        
        if len(self.accel_history['x']) > self.accel_history_size:
            self.accel_history['x'].pop(0)
            self.accel_history['y'].pop(0)
        
        # Определяем режим стабилизации
        rest_step_executed = False
        if status == 'stop' and self.rest_enabled:
            rest_step_executed = self._process_rest_stabilization(
                current_time, robot_state, x_move_amp, y_move_amp, angle_move_amp
            )
        elif status == 'move' and self.walking_enabled:
            # Убрана защита REST_STEP_PROTECTION_DURATION - она мешала нормальной работе
            # Стабилизация при ходьбе работает независимо от стабилизации покоя
            self._process_walking_stabilization(current_time)
        
        return rest_step_executed
    
    def _process_rest_stabilization(self, current_time, robot_state, x_move_amp, y_move_amp, angle_move_amp):
        """Обрабатывает стабилизацию в покое."""
        config = self.rest_config
        
        if current_time < self.rest_cooldown_end_time:
            return False
        
        if (robot_state != 'stand' or
            abs(x_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
            abs(y_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
            abs(angle_move_amp) > JOYSTICK_MOVE_THRESHOLD):
            return False
        
        if len(self.accel_history['x']) < 3:
            return False
        
        change_x = self.accel_history['x'][-1] - self.accel_history['x'][-2]
        change_y = self.accel_history['y'][-1] - self.accel_history['y'][-2]
        change_magnitude = math.sqrt(change_x * change_x + change_y * change_y)
        
        if change_magnitude < config['accel_change_threshold']:
            return False
        
        if len(self.accel_history['x']) >= 3:
            changes_x = []
            changes_y = []
            for i in range(len(self.accel_history['x'])-1, max(0, len(self.accel_history['x'])-4), -1):
                if i > 0:
                    changes_x.append(self.accel_history['x'][i] - self.accel_history['x'][i-1])
            for i in range(len(self.accel_history['y'])-1, max(0, len(self.accel_history['y'])-4), -1):
                if i > 0:
                    changes_y.append(self.accel_history['y'][i] - self.accel_history['y'][i-1])
            
            if changes_x and len(set(1 if c >= 0 else -1 for c in changes_x)) > 1:
                return False
            if changes_y and len(set(1 if c >= 0 else -1 for c in changes_y)) > 1:
                return False
        
        step_x = 0.0
        step_y = 0.0
        
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
        
        axes_changes.sort(key=lambda x: x[3])
        selected_axis, selected_sign, selected_magnitude, _ = axes_changes[0]
        
        step_direction_map = config['step_direction_map']
        direction_key = (selected_axis, selected_sign)
        if direction_key not in step_direction_map:
            rospy.logwarn(f"AutoStabilization: No direction mapping for {direction_key}")
            return False
        
        step_dir_x, step_dir_y = step_direction_map[direction_key]
        step_amplitude = self._calculate_rest_step_amplitude(change_magnitude, config)
        
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
        
        self._make_rest_step(step_x, step_y, change_magnitude, config)
        
        self.rest_cooldown_end_time = current_time + config['cooldown_duration']
        self.last_rest_step_time = current_time
        
        self.accel_history = {'x': [], 'y': []}
        
        rospy.loginfo(f"AutoStabilization [REST]: Step executed (x={step_x:.4f}, y={step_y:.4f}, change={change_magnitude:.3f} m/s²), cooldown={config['cooldown_duration']}s")
        
        return True
    
    def _process_walking_stabilization(self, current_time):
        """Обрабатывает стабилизацию при ходьбе."""
        config = WALKING_STABILIZATION_CONFIG
        
        # Проверяем, есть ли хотя бы один включенный параметр
        has_enabled_params = any([
            config['enable_period_time'],
            config['enable_step_fb_ratio'],
            config['enable_roll_offset'],
            config['enable_pitch_offset'],
            config['enable_y_offset'],
            config['enable_y_swap'],
            config['enable_z_swap'],
            config['enable_dsp_ratio']
        ])
        
        # Если все параметры выключены, не делаем ничего
        if not has_enabled_params:
            return
        
        if current_time - self.last_walking_update_time < config['update_interval']:
            return
        
        if len(self.accel_history['x']) < 2:
            return
        
        change_x = self.accel_history['x'][-1] - self.accel_history['x'][-2]
        change_y = self.accel_history['y'][-1] - self.accel_history['y'][-2]
        change_magnitude = math.sqrt(change_x * change_x + change_y * change_y)
        
        if change_magnitude < config['accel_change_threshold']:
            return
        
        gait_param = self.gait_manager.get_gait_param()
        params = self.speed_params[self.speed_mode]
        
        # НЕ перезаписываем gait_param базовыми значениями - они уже установлены speed_control
        # Только получаем базовые значения для расчетов
        period_time = list(params['period_time'])
        base_period_time = period_time[0]
        
        corrections = self._calculate_walking_corrections(
            change_x, change_y, change_magnitude, config, params, base_period_time
        )
        
        # Если нет корректировок, не обновляем параметры
        if not corrections:
            return
        
        filter_alpha = config['filter_alpha']
        has_actual_corrections = False
        
        for key, value in corrections.items():
            if key not in self.walking_filtered_params:
                self.walking_filtered_params[key] = value
                has_actual_corrections = True
            else:
                old_value = self.walking_filtered_params[key]
                self.walking_filtered_params[key] = (
                    filter_alpha * value + (1 - filter_alpha) * old_value
                )
                # Проверяем, изменилось ли значение значительно
                if abs(self.walking_filtered_params[key] - old_value) > 0.001:
                    has_actual_corrections = True
        
        # Если нет реальных изменений, не обновляем параметры
        if not has_actual_corrections:
            return
        
        # Сохраняем корректировки для применения в speed_control.process_axes
        # НЕ вызываем update_param - это останавливает движение!
        # Корректировки будут применены через метод apply_walking_corrections в speed_control
        
        self.last_walking_update_time = current_time
        rospy.logdebug(f"AutoStabilization [WALKING]: Corrections calculated (change={change_magnitude:.3f} m/s²)")
    
    def apply_walking_corrections(self, gait_param, period_time):
        """
        Применяет корректировки стабилизации при ходьбе к параметрам походки.
        Вызывается из speed_control.process_axes ПОСЛЕ установки базовых параметров.
        
        Args:
            gait_param: Словарь параметров походки (будет изменен)
            period_time: Список [period, x_swap, y_swap] (может быть изменен)
        
        Returns:
            bool: True если были применены корректировки
        """
        config = WALKING_STABILIZATION_CONFIG
        
        # Проверяем, есть ли включенные параметры
        has_enabled_params = any([
            config['enable_period_time'],
            config['enable_step_fb_ratio'],
            config['enable_roll_offset'],
            config['enable_pitch_offset'],
            config['enable_y_offset'],
            config['enable_y_swap'],
            config['enable_z_swap'],
            config['enable_dsp_ratio']
        ])
        
        if not has_enabled_params:
            return False
        
        applied = False
        
        # Применяем корректировки периода времени
        if config['enable_period_time'] and 'period_time' in self.walking_filtered_params:
            period_time[0] = int(self.walking_filtered_params['period_time'])
            applied = True
        
        # Применяем корректировки параметров походки
        if config['enable_step_fb_ratio'] and 'step_fb_ratio' in self.walking_filtered_params:
            gait_param['step_fb_ratio'] = self.walking_filtered_params['step_fb_ratio']
            applied = True
        
        if config['enable_roll_offset'] and 'init_roll_offset' in self.walking_filtered_params:
            gait_param['init_roll_offset'] = self.walking_filtered_params['init_roll_offset']
            applied = True
        
        if config['enable_pitch_offset'] and 'init_pitch_offset' in self.walking_filtered_params:
            gait_param['init_pitch_offset'] = self.walking_filtered_params['init_pitch_offset']
            applied = True
        
        if config['enable_y_offset'] and 'init_y_offset' in self.walking_filtered_params:
            gait_param['init_y_offset'] = self.walking_filtered_params['init_y_offset']
            applied = True
        
        if config['enable_y_swap'] and 'y_swap_amplitude' in self.walking_filtered_params:
            gait_param['y_swap_amplitude'] = self.walking_filtered_params['y_swap_amplitude']
            applied = True
        
        if config['enable_z_swap'] and 'z_swap_amplitude' in self.walking_filtered_params:
            gait_param['z_swap_amplitude'] = self.walking_filtered_params['z_swap_amplitude']
            applied = True
        
        if config['enable_dsp_ratio'] and 'dsp_ratio' in self.walking_filtered_params:
            gait_param['dsp_ratio'] = self.walking_filtered_params['dsp_ratio']
            applied = True
        
        return applied
    
    def _calculate_rest_step_amplitude(self, change_magnitude, config):
        """Вычисляет амплитуду шага стабилизации в покое."""
        threshold = config['accel_change_threshold']
        base = config['step_amplitude_base']
        max_amp = config['step_amplitude_max']
        max_for_max = config['accel_max_for_max_step']
        
        if change_magnitude <= threshold:
            amplitude = base
        elif change_magnitude >= max_for_max:
            amplitude = max_amp
        else:
            ratio = (change_magnitude - threshold) / (max_for_max - threshold)
            amplitude = base + (max_amp - base) * ratio
        
        amplitude = max(base, min(max_amp, amplitude))
        return amplitude
    
    def _calculate_rest_step_speed(self, change_magnitude, config):
        """Вычисляет скорость шага стабилизации в покое."""
        threshold = config['accel_change_threshold']
        normal = config['speed_factor_normal']
        min_factor = config['speed_factor_min']
        max_for_max = config['accel_max_for_max_step']
        
        if change_magnitude <= threshold:
            return normal
        
        if change_magnitude >= max_for_max:
            return min_factor
        
        ratio = (change_magnitude - threshold) / (max_for_max - threshold)
        speed_reduction = normal - min_factor
        return normal - ratio * speed_reduction
    
    def _calculate_rest_arm_swing(self, change_magnitude, config):
        """Вычисляет размах рук для стабилизации в покое."""
        base = config['arm_swing_base']
        max_swing = config['arm_swing_max']
        threshold_low = config['arm_swing_threshold_low']
        threshold_high = config['arm_swing_threshold_high']
        
        if change_magnitude <= threshold_low:
            return base
        elif change_magnitude >= threshold_high:
            return max_swing
        else:
            ratio = (change_magnitude - threshold_low) / (threshold_high - threshold_low)
            return base + (max_swing - base) * ratio
    
    def _make_rest_step(self, step_x, step_y, change_magnitude, config):
        """Выполняет шаг стабилизации в покое."""
        try:
            gait_param = self.gait_manager.get_gait_param()
            params = self.speed_params[self.speed_mode]
            period_time = list(params['period_time'])
            
            speed_factor = self._calculate_rest_step_speed(change_magnitude, config)
            period_time[0] = int(period_time[0] * speed_factor)
            
            gait_param.update(params.get('gait_base', {}))
            gait_param['body_height'] = self.init_z_offset
            
            arm_swing = self._calculate_rest_arm_swing(change_magnitude, config)
            
            self.gait_manager.set_step(
                period_time,
                step_x,
                step_y,
                0,
                gait_param,
                arm_swap=arm_swing,
                step_num=1
            )
        except Exception as e:
            rospy.logwarn(f"AutoStabilization [REST]: Error making step: {e}")
    
    def _calculate_walking_corrections(self, change_x, change_y, change_magnitude, config, speed_params, base_period_time):
        """Вычисляет корректировки параметров походки для стабилизации при ходьбе."""
        corrections = {}
        
        threshold = config['accel_change_threshold']
        normalized_change = min(1.0, change_magnitude / (threshold * 5))
        
        gait_base = speed_params.get('gait_base', {})
        base_step_fb_ratio = gait_base.get('step_fb_ratio', 0.028)
        base_dsp_ratio = gait_base.get('dsp_ratio', 0.2)
        base_roll_offset = gait_base.get('init_roll_offset', 0.0)
        base_pitch_offset = gait_base.get('init_pitch_offset', 0.0)
        base_y_offset = gait_base.get('init_y_offset', 0.0)
        base_y_swap = gait_base.get('y_swap_amplitude', 0.02)
        base_z_swap = gait_base.get('z_swap_amplitude', 0.006)
        
        if config['enable_period_time']:
            reduction_max = config['period_time_reduction_max']
            sensitivity = config['period_time_sensitivity']
            reduction = normalized_change * reduction_max * sensitivity
            corrected_period = base_period_time * (1.0 + reduction)
            corrections['period_time'] = max(base_period_time, int(corrected_period))
        
        if config['enable_step_fb_ratio']:
            reduction_max = config['step_fb_ratio_reduction_max']
            sensitivity = config['step_fb_ratio_sensitivity']
            reduction = normalized_change * reduction_max * sensitivity
            corrected_ratio = base_step_fb_ratio * (1.0 - reduction)
            corrections['step_fb_ratio'] = max(base_step_fb_ratio * 0.5, corrected_ratio)
        
        if config['enable_roll_offset']:
            roll_max = config['roll_offset_max']
            roll_sens = config['roll_sensitivity']
            roll_correction = base_roll_offset + math.copysign(
                normalized_change * roll_max * roll_sens * abs(change_x) / change_magnitude if change_magnitude > 0 else 0,
                change_x
            )
            corrections['init_roll_offset'] = max(-roll_max, min(roll_max, roll_correction))
        
        if config['enable_pitch_offset']:
            pitch_max = config['pitch_offset_max']
            pitch_sens = config['pitch_sensitivity']
            pitch_correction = base_pitch_offset + math.copysign(
                normalized_change * pitch_max * pitch_sens * abs(change_y) / change_magnitude if change_magnitude > 0 else 0,
                change_y
            )
            corrections['init_pitch_offset'] = max(-pitch_max, min(pitch_max, pitch_correction))
        
        if config['enable_y_offset']:
            y_offset_max = config['y_offset_max']
            y_offset_sens = config['y_offset_sensitivity']
            y_offset_correction = base_y_offset + math.copysign(
                normalized_change * y_offset_max * y_offset_sens,
                change_y
            )
            corrections['init_y_offset'] = max(-y_offset_max, min(y_offset_max, y_offset_correction))
        
        if config['enable_y_swap']:
            y_swap_max = config['y_swap_max']
            y_swap_sens = config['y_swap_sensitivity']
            y_swap_correction = base_y_swap + normalized_change * (y_swap_max - base_y_swap) * y_swap_sens
            corrections['y_swap_amplitude'] = max(base_y_swap, min(y_swap_max, y_swap_correction))
        
        if config['enable_z_swap']:
            z_swap_max = config['z_swap_max']
            z_swap_sens = config['z_swap_sensitivity']
            z_swap_correction = base_z_swap + normalized_change * (z_swap_max - base_z_swap) * z_swap_sens
            corrections['z_swap_amplitude'] = max(base_z_swap, min(z_swap_max, z_swap_correction))
        
        if config['enable_dsp_ratio']:
            increase_max = config['dsp_ratio_increase_max']
            sensitivity = config['dsp_ratio_sensitivity']
            increase = normalized_change * increase_max * sensitivity
            corrected_dsp = base_dsp_ratio + increase
            corrections['dsp_ratio'] = min(base_dsp_ratio + increase_max, corrected_dsp)
        
        return corrections
    
    def reset_walking_corrections(self):
        """
        Сбрасывает корректировки стабилизации при ходьбе.
        Вызывается при остановке движения для возврата к базовым параметрам.
        """
        # Очищаем отфильтрованные параметры, чтобы вернуться к базовым значениям
        self.walking_filtered_params = {}
        self.last_walking_update_time = 0
    
    def reset(self):
        """Сбрасывает состояние стабилизации."""
        self.rest_cooldown_end_time = 0
        self.last_rest_step_time = 0
        self.last_walking_update_time = 0
        self.filtered_accel = {'x': 0.0, 'y': 0.0}
        self.accel_history = {'x': [], 'y': []}
        self.walking_filtered_params = {}
