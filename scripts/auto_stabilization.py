#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль автостабилизации робота в покое и при ходьбе.

СТАБИЛИЗАЦИЯ В ПОКОЕ:
- Выполняет корректирующие шаги при обнаружении нестабильности
- Готова к перестройке с учетом новых параметров IMU (orientation, angular_velocity, linear_acceleration)

СТАБИЛИЗАЦИЯ ПРИ ХОДЬБЕ:
- Динамически корректирует параметры походки на основе IMU данных
- Работает независимо от стабилизации покоя
- Готова к перестройке с учетом новых параметров IMU (orientation, angular_velocity, linear_acceleration)
"""

# ========== КОНФИГУРАЦИЯ: СТАБИЛИЗАЦИЯ В ПОКОЕ ==========
REST_STABILIZATION_CONFIG = {
    # Критические углы по каждой оси (в градусах) - после которых начинается стабилизация
    'critical_angle_roll': 25.0,   # Критический угол крена (наклон влево/вправо)
    'critical_angle_pitch': 34.0,  # Критический угол тангажа (наклон вперед/назад)
    'critical_angle_yaw': 15.0,     # Критический угол рыскания (поворот)
    
    # Параметры для gait_manager при стабилизации (отдельные от джойстика)
    'gait_params': {
        'period_time': [300, 0.2, 0.022],  # [period_ms, x_swap, y_swap] - быстрые шаги
        'gait_base': {
            'dsp_ratio': 0.25,              # Двойная опора - больше стабильности
            'step_fb_ratio': 0.020,         # Дистанция шага (меньше для точности)
            'y_swap_amplitude': 0.015,      # Амплитуда обмена по Y
            'z_swap_amplitude': 0.008,      # Амплитуда подъема ноги
            'init_y_offset': -0.005,
            'init_roll_offset': 0.0,
            'init_pitch_offset': 0.0
        }
    },
    
    # Коэффициенты для расчета амплитуды шага на основе угла наклона
    'step_amplitude_coefficients': {
        'roll': 0.002,   # Коэффициент для шага по Y при наклоне по roll (м/градус)
        'pitch': 0.002,  # Коэффициент для шага по X при наклоне по pitch (м/градус)
        'yaw': 2.0,      # Коэффициент для поворота при наклоне по yaw (градус/градус)
    },
    
    # Максимальные амплитуды шагов стабилизации
    'max_step_amplitudes': {
        'x': 0.015,      # Максимальный шаг вперед/назад (м)
        'y': 0.015,      # Максимальный шаг влево/вправо (м)
        'angle': 8.0,    # Максимальный поворот (градусы)
    },
    
    # Динамический подбор параметров
    'dynamic_tuning_enabled': True,  # Включить/выключить динамический подбор параметров
    'tuning_coefficients': {
        'period_time': 0.1,         # Коэффициент изменения периода (0.0-1.0)
        'step_amplitude': 0.05,     # Коэффициент изменения амплитуды шага (0.0-1.0)
        'gait_base': 0.05,          # Коэффициент изменения gait_base параметров (0.0-1.0)
    },
    
    # Параметр возврата
    'return_enabled': True,         # Включить/выключить возврат после стабилизации
    'return_delay': 0.5,            # Задержка перед началом возврата (сек)
    'return_stable_threshold': 3.0, # Порог стабильности для начала возврата (градусы)
    
    # Стабилизация YAW (поворота)
    'yaw_stabilization_enabled': True,  # Включить/выключить стабилизацию поворота
    'yaw_stabilization_delay': 2.0,     # Задержка после прекращения воздействия (сек)
    'yaw_stable_threshold': 5.0,        # Порог стабильности YAW для начала разворота (градусы)
    
    # Интервалы между шагами стабилизации
    'step_interval': 0.1,           # Минимальный интервал между шагами (сек)
}

# ========== КОНФИГУРАЦИЯ: СТАБИЛИЗАЦИЯ ПРИ ХОДЬБЕ ==========
WALKING_STABILIZATION_CONFIG = {
    # Конфигурация будет добавлена при перестройке с новыми параметрами IMU
}

# ========== ОБЩИЕ ПАРАМЕТРЫ ==========
JOYSTICK_MOVE_THRESHOLD = 0.001
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
        
        self.rest_config = REST_STABILIZATION_CONFIG.copy()
        self.walking_config = WALKING_STABILIZATION_CONFIG.copy()
        self.enabled = True
        self.rest_enabled = True
        self.walking_enabled = True
        
        # Состояние стабилизации в покое
        self.rest_stabilization_state = {
            'last_step_time': 0,
            'steps_taken': {'x': 0, 'y': 0, 'angle': 0},  # Счетчик шагов для возврата
            'last_tilt': {'roll': 0, 'pitch': 0, 'yaw': 0},  # Последний наклон
            'stabilization_active': False,  # Активна ли стабилизация
            'return_pending': False,  # Ожидается ли возврат
            'return_start_time': None,  # Время начала возврата
            'yaw_stabilization_pending': False,  # Ожидается ли стабилизация YAW
            'yaw_stabilization_start_time': None,  # Время начала стабилизации YAW
            'last_yaw': 0,  # Последний YAW для отслеживания изменений
            'yaw_change_time': None,  # Время последнего изменения YAW
            'last_imu_time': 0,  # Время последнего получения данных IMU
            'last_processed_angles': None,  # Последние обработанные углы для сравнения
        }
        
        rospy.loginfo("AutoStabilization module initialized (Rest + Walking)")
        rospy.loginfo("  Rest stabilization: Ready with IMU parameters")
    
    def process(self, imu_data, robot_state, status, x_move_amp, y_move_amp, angle_move_amp):
        """
        Обрабатывает данные IMU и выполняет стабилизацию при необходимости.
        
        Args:
            imu_data: Словарь с данными IMU (orientation, angular_velocity, linear_acceleration)
            robot_state: Состояние робота ('stand', 'lie_to_stand', 'recline_to_stand')
            status: Статус движения ('move' или 'stop')
            x_move_amp: Амплитуда движения по X
            y_move_amp: Амплитуда движения по Y
            angle_move_amp: Амплитуда поворота
        
        Returns:
            bool: True если был выполнен шаг стабилизации
        """
        if not self.enabled:
            return False
        
        if imu_data is None:
            return False
        
        # Определяем режим стабилизации
        rest_step_executed = False
        if status == 'stop' and self.rest_enabled:
            rest_step_executed = self._process_rest_stabilization(
                imu_data, robot_state, x_move_amp, y_move_amp, angle_move_amp
            )
        elif status == 'move' and self.walking_enabled:
            self._process_walking_stabilization(imu_data)
        
        return rest_step_executed
    
    def _process_rest_stabilization(self, imu_data, robot_state, x_move_amp, y_move_amp, angle_move_amp):
        """
        Обрабатывает стабилизацию в покое.
        
        Args:
            imu_data: Словарь с данными IMU
            robot_state: Состояние робота
            x_move_amp: Амплитуда движения по X (от джойстика)
            y_move_amp: Амплитуда движения по Y (от джойстика)
            angle_move_amp: Амплитуда поворота (от джойстика)
        
        Returns:
            bool: True если был выполнен шаг стабилизации
        """
        # Проверяем, нет ли движения от джойстика
        joystick_moving = (abs(x_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(y_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(angle_move_amp) > JOYSTICK_MOVE_THRESHOLD)
        
        if joystick_moving:
            # Если есть движение от джойстика, сбрасываем состояние стабилизации
            if self.rest_stabilization_state['stabilization_active']:
                rospy.loginfo("🎮 Joystick movement detected - stopping stabilization")
            self._reset_rest_stabilization_state()
            return False
        
        # Получаем углы ориентации - ВАЖНО: используем свежие данные из текущего вызова
        orientation = imu_data.get('orientation', {})
        if not orientation:
            rospy.logwarn("⚠️ No orientation data in IMU")
            return False
            
        # ВАЖНО: Получаем свежие значения каждый раз, не используем кэш
        roll_deg = orientation.get('roll_deg', 0)
        pitch_deg = orientation.get('pitch_deg', 0)
        yaw_deg = orientation.get('yaw_deg', 0)
        
        current_time = rospy.get_time()
        state = self.rest_stabilization_state
        
        # Проверяем свежесть данных IMU - если данные старые, не обрабатываем
        # Это предотвращает обработку устаревших данных, если callback заблокирован
        imu_age = current_time - state['last_imu_time']
        max_imu_age = 0.5  # Максимальный возраст данных IMU (сек)
        
        # Обновляем время получения данных
        state['last_imu_time'] = current_time
        
        # Проверяем, изменились ли углы (для отладки)
        current_angles = (roll_deg, pitch_deg, yaw_deg)
        if state['last_processed_angles'] is not None:
            angle_change = (
                abs(roll_deg - state['last_processed_angles'][0]),
                abs(pitch_deg - state['last_processed_angles'][1]),
                abs(yaw_deg - state['last_processed_angles'][2])
            )
            if max(angle_change) > 1.0:  # Значительное изменение углов
                rospy.loginfo(f"📊 Angle change detected: roll={angle_change[0]:.2f}°, "
                            f"pitch={angle_change[1]:.2f}°, yaw={angle_change[2]:.2f}°")
        
        state['last_processed_angles'] = current_angles
        
        # Если данные слишком старые, пропускаем обработку
        if imu_age > max_imu_age and state['last_imu_time'] > 0:
            rospy.logwarn(f"⚠️ IMU data too old: {imu_age:.3f}s (max: {max_imu_age}s) - skipping")
            return False
        
        # Логируем текущие углы для отладки (только если стабилизация активна)
        if state['stabilization_active']:
            rospy.loginfo(f"📊 Current angles: roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}° (age: {imu_age:.3f}s)")
        
        # Проверяем необходимость стабилизации по наклону
        needs_stabilization = self._check_stabilization_needed(roll_deg, pitch_deg, yaw_deg)
        
        # Обрабатываем возврат после стабилизации
        if self.rest_config['return_enabled'] and state['return_pending']:
            # Если робот снова наклонился во время возврата - прерываем возврат
            if needs_stabilization:
                rospy.logwarn("⚠️ Robot tilted during return - canceling return and restarting stabilization")
                state['return_pending'] = False
                state['return_start_time'] = None
            elif state['return_start_time'] is None:
                state['return_start_time'] = current_time
            
            # Проверяем задержку перед возвратом
            if state['return_start_time'] is not None and current_time - state['return_start_time'] >= self.rest_config['return_delay']:
                # Проверяем стабильность перед возвратом
                if self._is_stable(roll_deg, pitch_deg, yaw_deg, self.rest_config['return_stable_threshold']):
                    return self._execute_return_steps()
                else:
                    # Робот нестабилен - отменяем возврат
                    rospy.logwarn("⚠️ Robot unstable during return - canceling return")
                    state['return_pending'] = False
                    state['return_start_time'] = None
        
        # Обрабатываем стабилизацию YAW
        if self.rest_config['yaw_stabilization_enabled']:
            self._process_yaw_stabilization(yaw_deg, current_time)
        
        # Выполняем стабилизацию по наклону
        if needs_stabilization:
            # Проверяем, не стабилен ли уже робот (чтобы не продолжать стабилизацию бесконечно)
            # Используем более строгий порог для остановки - половина критического угла
            config = self.rest_config
            stable_threshold_roll = config['critical_angle_roll'] * 0.5
            stable_threshold_pitch = config['critical_angle_pitch'] * 0.5
            is_stable_now = (abs(roll_deg) < stable_threshold_roll and 
                            abs(pitch_deg - 90.0) < stable_threshold_pitch)
            
            if is_stable_now and state['stabilization_active']:
                # Робот уже стабилен - останавливаем стабилизацию и движение
                rospy.loginfo(f"✅ Stabilization: Robot is stable (roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°) - stopping immediately")
                try:
                    self.gait_manager.stop()
                except Exception as e:
                    rospy.logwarn(f"Error stopping gait_manager: {e}")
                state['stabilization_active'] = False
                state['last_step_time'] = 0  # Сбрасываем таймер, чтобы можно было сразу начать возврат
                if self.rest_config['return_enabled'] and self._has_steps_to_return():
                    state['return_pending'] = True
                    state['return_start_time'] = None
                    rospy.loginfo(f"📤 Return planned: {state['steps_taken']}")
                return False
            
            # Робот нестабилен - продолжаем стабилизацию
            if not state['stabilization_active']:
                rospy.logwarn(f"⚠️ Stabilization started: roll={roll_deg:.2f}° (critical: {config['critical_angle_roll']:.1f}°), "
                             f"pitch={pitch_deg:.2f}° (deviation from 90°: {abs(pitch_deg - 90.0):.2f}°, critical: {config['critical_angle_pitch']:.1f}°)")
            
            state['stabilization_active'] = True
            state['last_tilt'] = {'roll': roll_deg, 'pitch': pitch_deg, 'yaw': yaw_deg}
            
            # Проверяем интервал между шагами
            time_since_last_step = current_time - state['last_step_time']
            if time_since_last_step >= self.rest_config['step_interval']:
                # ВАЖНО: Используем ТЕКУЩИЕ углы для вычисления амплитуд (не кэшированные)
                # Также проверяем, что данные свежие (не старше 0.2 сек)
                if imu_age < 0.2:
                    step_executed = self._execute_stabilization_step(roll_deg, pitch_deg)
                    if step_executed:
                        state['last_step_time'] = current_time
                        # Сбрасываем возврат, так как началась новая стабилизация
                        state['return_pending'] = False
                        state['return_start_time'] = None
                    return step_executed
                else:
                    rospy.logwarn(f"⚠️ Skipping step - IMU data too old: {imu_age:.3f}s")
                    return False
            else:
                # Ждем интервал - не выполняем шаг, но и не останавливаем стабилизацию
                # Логируем, если данные изменились, но мы еще ждем интервал
                if state['last_processed_angles'] is not None:
                    angle_change = max(
                        abs(roll_deg - state['last_processed_angles'][0]),
                        abs(pitch_deg - state['last_processed_angles'][1])
                    )
                    if angle_change > 2.0:  # Значительное изменение
                        rospy.loginfo(f"⏳ Waiting for step interval: {time_since_last_step:.3f}s / {self.rest_config['step_interval']:.3f}s, "
                                    f"angle change: {angle_change:.2f}°")
                return False
        else:
            # Наклон в пределах нормы - ОБЯЗАТЕЛЬНО останавливаем движение, если оно было активно
            if state['stabilization_active']:
                # Завершили стабилизацию - останавливаем движение и планируем возврат
                rospy.loginfo(f"✅ Stabilization: Angle within normal range (roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°) - stopping")
                try:
                    self.gait_manager.stop()
                except Exception as e:
                    rospy.logwarn(f"Error stopping gait_manager: {e}")
                
                state['stabilization_active'] = False
                state['last_step_time'] = 0  # Сбрасываем таймер
                if self.rest_config['return_enabled'] and self._has_steps_to_return():
                    state['return_pending'] = True
                    state['return_start_time'] = None
                    rospy.loginfo(f"📤 Return planned: {state['steps_taken']}")
            elif state['return_pending']:
                # Если возврат активен, но робот снова наклонился - отменяем возврат
                # (это уже обработано выше, но на всякий случай)
                pass
        
        return False
    
    def _check_stabilization_needed(self, roll_deg, pitch_deg, yaw_deg):
        """
        Проверяет, нужна ли стабилизация на основе углов ориентации.
        
        Args:
            roll_deg: Угол крена в градусах
            pitch_deg: Угол тангажа в градусах
            yaw_deg: Угол рыскания в градусах
        
        Returns:
            bool: True если нужна стабилизация
        """
        config = self.rest_config
        critical_roll = config['critical_angle_roll']
        critical_pitch = config['critical_angle_pitch']
        
        # Проверяем roll и pitch (yaw обрабатывается отдельно)
        roll_exceeded = abs(roll_deg) > critical_roll
        # Для pitch: нормальное положение около 90°, проверяем отклонение
        pitch_deviation = abs(pitch_deg - 90.0)
        pitch_exceeded = pitch_deviation > critical_pitch
        
        return roll_exceeded or pitch_exceeded
    
    def _execute_stabilization_step(self, roll_deg, pitch_deg):
        """
        Выполняет шаг стабилизации в сторону наклона.
        
        Args:
            roll_deg: Угол крена в градусах
            pitch_deg: Угол тангажа в градусах
        
        Returns:
            bool: True если шаг был выполнен
        """
        config = self.rest_config
        state = self.rest_stabilization_state
        
        # Получаем параметры походки для стабилизации
        gait_param = self.gait_manager.get_gait_param().copy()
        gait_params = config['gait_params']
        period_time = list(gait_params['period_time'])
        
        # Применяем базовые параметры стабилизации
        gait_param.update(gait_params.get('gait_base', {}))
        gait_param['init_z_offset'] = self.init_z_offset
        
        # Вычисляем амплитуды шагов на основе углов наклона
        coeffs = config['step_amplitude_coefficients']
        max_amps = config['max_step_amplitudes']
        
        # Roll: наклон влево (положительный) -> шаг влево (положительный Y)
        #       наклон вправо (отрицательный) -> шаг вправо (отрицательный Y)
        y_amplitude = roll_deg * coeffs['roll']
        y_amplitude = max(-max_amps['y'], min(max_amps['y'], y_amplitude))
        
        # Pitch: нормальное положение около 90°
        #        наклон вперед (pitch < 90) -> шаг вперед (положительный X)
        #        наклон назад (pitch > 90) -> шаг назад (отрицательный X)
        pitch_deviation = pitch_deg - 90.0
        x_amplitude = -pitch_deviation * coeffs['pitch']  # Отрицательный, так как наклон вперед требует шага вперед
        x_amplitude = max(-max_amps['x'], min(max_amps['x'], x_amplitude))
        
        # Угол поворота не используем для стабилизации наклона (только для YAW стабилизации)
        angle_amplitude = 0
        
        # Проверяем, есть ли смысл делать шаг (амплитуды должны быть значимыми)
        min_amplitude = 0.001  # Минимальная амплитуда для выполнения шага
        if abs(x_amplitude) < min_amplitude and abs(y_amplitude) < min_amplitude:
            # Амплитуды слишком малы - не выполняем шаг
            rospy.logdebug(f"⚠️ Amplitudes too small: X={x_amplitude:.4f}, Y={y_amplitude:.4f} - skipping step")
            return False
        
        # Применяем динамическую настройку параметров, если включена
        if config['dynamic_tuning_enabled']:
            self._apply_dynamic_tuning(gait_param, period_time, abs(roll_deg), abs(pitch_deviation))
        
        # ВАЖНО: Используем step_num=0 для непрерывного движения (как в speed_control)
        # Это НЕ блокирует поток, в отличие от step_num=1
        # Не вызываем stop() - просто обновляем параметры движения
        try:
            self.gait_manager.set_step(
                period_time,
                x_amplitude,
                y_amplitude,
                angle_amplitude,
                gait_param,
                step_num=0  # 0 = непрерывное движение (неблокирующий), 1 = один шаг (блокирующий)
            )
            
            # Обновляем счетчик для возврата (при step_num=0 это непрерывное движение)
            # Считаем направление и время движения вместо количества шагов
            if abs(x_amplitude) > 0.001:
                state['steps_taken']['x'] = 1 if x_amplitude > 0 else -1  # Направление
            if abs(y_amplitude) > 0.001:
                state['steps_taken']['y'] = 1 if y_amplitude > 0 else -1  # Направление
            
            rospy.loginfo(f"🔄 Stabilization step: X={x_amplitude:.4f}, Y={y_amplitude:.4f}, "
                         f"Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°")
            return True
        except Exception as e:
            rospy.logerr(f"Error executing stabilization step: {e}")
            return False
    
    def _apply_dynamic_tuning(self, gait_param, period_time, roll_magnitude, pitch_magnitude):
        """
        Применяет динамическую настройку параметров на основе величины наклона.
        
        Args:
            gait_param: Словарь параметров походки (будет изменен)
            period_time: Список [period, x_swap, y_swap] (может быть изменен)
            roll_magnitude: Величина наклона по roll (градусы)
            pitch_magnitude: Величина наклона по pitch (градусы)
        """
        config = self.rest_config
        tuning = config['tuning_coefficients']
        
        # Вычисляем общую величину наклона (нормализованную)
        max_tilt = max(roll_magnitude, pitch_magnitude)
        critical_angle = max(config['critical_angle_roll'], config['critical_angle_pitch'])
        tilt_factor = min(1.0, max_tilt / (critical_angle * 2))  # 0.0 - 1.0
        
        # Настраиваем период времени (меньше период = быстрее шаги)
        if tuning['period_time'] > 0:
            period_reduction = tuning['period_time'] * tilt_factor
            period_time[0] = int(period_time[0] * (1.0 - period_reduction))
            period_time[0] = max(200, period_time[0])  # Минимум 200мс
        
        # Настраиваем амплитуды шагов (больше наклон = больше шаг)
        if tuning['step_amplitude'] > 0:
            amplitude_increase = tuning['step_amplitude'] * tilt_factor
            if 'step_fb_ratio' in gait_param:
                gait_param['step_fb_ratio'] *= (1.0 + amplitude_increase)
            if 'y_swap_amplitude' in gait_param:
                gait_param['y_swap_amplitude'] *= (1.0 + amplitude_increase)
    
    def _is_stable(self, roll_deg, pitch_deg, yaw_deg, threshold):
        """
        Проверяет, находится ли робот в стабильном положении.
        
        Args:
            roll_deg: Угол крена в градусах
            pitch_deg: Угол тангажа в градусах
            yaw_deg: Угол рыскания в градусах
            threshold: Порог стабильности (градусы)
        
        Returns:
            bool: True если робот стабилен
        """
        roll_stable = abs(roll_deg) < threshold
        pitch_deviation = abs(pitch_deg - 90.0)
        pitch_stable = pitch_deviation < threshold
        
        return roll_stable and pitch_stable
    
    def _has_steps_to_return(self):
        """Проверяет, есть ли шаги для возврата."""
        steps = self.rest_stabilization_state['steps_taken']
        return (abs(steps['x']) > 0 or abs(steps['y']) > 0)
    
    def _execute_return_steps(self):
        """
        Выполняет возврат после стабилизации (при step_num=0 это непрерывное движение).
        
        Returns:
            bool: True если возврат выполняется
        """
        state = self.rest_stabilization_state
        steps = state['steps_taken']
        current_time = rospy.get_time()
        
        # Определяем направление возврата (противоположное направлению стабилизации)
        x_return = 0
        y_return = 0
        
        if abs(steps['x']) > 0:
            x_return = -0.01 if steps['x'] > 0 else 0.01  # Движение в противоположную сторону
        if abs(steps['y']) > 0:
            y_return = -0.01 if steps['y'] > 0 else 0.01  # Движение в противоположную сторону
        
        if abs(x_return) < 0.001 and abs(y_return) < 0.001:
            # Возврат завершен - останавливаем движение
            rospy.loginfo("✅ Return completed - stopping movement")
            try:
                self.gait_manager.stop()
            except Exception as e:
                rospy.logwarn(f"Error stopping gait_manager after return: {e}")
            state['return_pending'] = False
            state['return_start_time'] = None
            state['steps_taken'] = {'x': 0, 'y': 0, 'angle': 0}
            return False
        
        # Выполняем возврат (непрерывное движение)
        config = self.rest_config
        gait_param = self.gait_manager.get_gait_param().copy()
        gait_params = config['gait_params']
        period_time = list(gait_params['period_time'])
        
        gait_param.update(gait_params.get('gait_base', {}))
        gait_param['init_z_offset'] = self.init_z_offset
        
        try:
            self.gait_manager.set_step(
                period_time,
                x_return,
                y_return,
                0,
                gait_param,
                step_num=0  # Непрерывное движение для возврата
            )
            
            # Возврат выполняется определенное время
            return_duration = 1.0  # Время возврата в секундах
            # return_start_time уже установлен в _process_rest_stabilization
            
            if state['return_start_time'] is not None and current_time - state['return_start_time'] >= return_duration:
                # Время возврата истекло - останавливаем
                rospy.loginfo("✅ Return time completed - stopping movement")
                try:
                    self.gait_manager.stop()
                except Exception as e:
                    rospy.logwarn(f"Error stopping gait_manager after return: {e}")
                state['return_pending'] = False
                state['return_start_time'] = None
                state['steps_taken'] = {'x': 0, 'y': 0, 'angle': 0}
                return False
            
            rospy.loginfo(f"↩️ Return: X={x_return:.4f}, Y={y_return:.4f}, "
                         f"time: {current_time - state['return_start_time']:.2f}s / {return_duration:.2f}s")
            return True
        except Exception as e:
            rospy.logerr(f"Error executing return step: {e}")
            return False
    
    def _process_yaw_stabilization(self, yaw_deg, current_time):
        """
        Обрабатывает стабилизацию поворота по YAW.
        
        Args:
            yaw_deg: Угол рыскания в градусах
            current_time: Текущее время
        """
        config = self.rest_config
        state = self.rest_stabilization_state
        critical_yaw = config['critical_angle_yaw']
        
        # Отслеживаем изменения YAW
        yaw_change = abs(yaw_deg - state['last_yaw'])
        
        if yaw_change > 2.0:  # Значительное изменение YAW
            state['yaw_change_time'] = current_time
            state['yaw_stabilization_pending'] = False
            state['yaw_stabilization_start_time'] = None
        
        state['last_yaw'] = yaw_deg
        
        # Проверяем, нужна ли стабилизация YAW
        if abs(yaw_deg) > critical_yaw:
            # YAW превысил критический угол - сбрасываем таймер
            state['yaw_stabilization_pending'] = False
            state['yaw_stabilization_start_time'] = None
            return
        
        # Если YAW стабилен и был поворот, планируем разворот назад
        if state['yaw_change_time'] is not None:
            time_since_change = current_time - state['yaw_change_time']
            
            if time_since_change >= config['yaw_stabilization_delay']:
                # Прошло достаточно времени - начинаем стабилизацию
                if not state['yaw_stabilization_pending']:
                    state['yaw_stabilization_pending'] = True
                    state['yaw_stabilization_start_time'] = current_time
                
                # Проверяем стабильность YAW перед разворотом
                if abs(yaw_deg) < config['yaw_stable_threshold']:
                    # YAW уже стабилен - не нужно разворачиваться
                    state['yaw_stabilization_pending'] = False
                    state['yaw_stabilization_start_time'] = None
                    state['yaw_change_time'] = None
                else:
                    # Выполняем разворот назад
                    self._execute_yaw_stabilization_step(yaw_deg)
    
    def _execute_yaw_stabilization_step(self, yaw_deg):
        """
        Выполняет шаг стабилизации YAW (разворот назад).
        
        Args:
            yaw_deg: Угол рыскания в градусах
        """
        config = self.rest_config
        state = self.rest_stabilization_state
        current_time = rospy.get_time()
        
        # Проверяем интервал между шагами
        if current_time - state['last_step_time'] < self.rest_config['step_interval']:
            return
        
        # Вычисляем угол разворота (в противоположную сторону)
        coeffs = config['step_amplitude_coefficients']
        max_amps = config['max_step_amplitudes']
        
        angle_amplitude = -yaw_deg * coeffs['yaw']  # Отрицательный для разворота назад
        angle_amplitude = max(-max_amps['angle'], min(max_amps['angle'], angle_amplitude))
        
        # Получаем параметры походки
        gait_param = self.gait_manager.get_gait_param().copy()
        gait_params = config['gait_params']
        period_time = list(gait_params['period_time'])
        
        gait_param.update(gait_params.get('gait_base', {}))
        gait_param['init_z_offset'] = self.init_z_offset
        
        try:
            self.gait_manager.set_step(
                period_time,
                0,  # Без движения вперед/назад
                0,  # Без движения влево/вправо
                angle_amplitude,
                gait_param,
                step_num=0  # Непрерывное движение для YAW стабилизации
            )
            
            state['last_step_time'] = current_time
            rospy.loginfo(f"🔄 YAW stabilization step: angle={angle_amplitude:.2f}°, yaw={yaw_deg:.2f}°")
        except Exception as e:
            rospy.logerr(f"Error executing YAW stabilization step: {e}")
    
    def _reset_rest_stabilization_state(self):
        """Сбрасывает состояние стабилизации в покое."""
        state = self.rest_stabilization_state
        state['steps_taken'] = {'x': 0, 'y': 0, 'angle': 0}
        state['stabilization_active'] = False
        state['return_pending'] = False
        state['return_start_time'] = None
        state['yaw_stabilization_pending'] = False
        state['yaw_stabilization_start_time'] = None
        state['yaw_change_time'] = None
        state['last_processed_angles'] = None
    
    def _process_walking_stabilization(self, imu_data):
        """
        Обрабатывает стабилизацию при ходьбе.
        
        Args:
            imu_data: Словарь с данными IMU
        """
        # TODO: Перестроить с учетом новых параметров IMU
        # Использовать: imu_data['orientation'], imu_data['angular_velocity'], imu_data['linear_acceleration']
        pass
    
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
        # TODO: Перестроить с учетом новых параметров IMU
        return False
    
    def reset_walking_corrections(self):
        """
        Сбрасывает корректировки стабилизации при ходьбе.
        Вызывается при остановке движения для возврата к базовым параметрам.
        """
        # TODO: Реализовать при перестройке
        pass
    
    def reset(self):
        """Сбрасывает состояние стабилизации."""
        self._reset_rest_stabilization_state()
        self.reset_walking_corrections()
