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
    # Эталонная ориентация (нормальное положение робота)
    'reference_mode': 'auto',  # 'none' - без эталона, 'manual' - вручную, 'auto' - авто (через 3 сек после запуска)
    'reference_manual': {  # Используется только если reference_mode='manual'
        'roll': 0.0,
        'pitch': 90.0,  # Нормальное положение около 90°
        'yaw': 0.0
    },
    'reference_auto_delay': 3.0,  # Задержка перед сбором эталона (сек)
    
    # Включение/выключение стабилизации по осям
    'stabilization_enabled': {
        'roll': True,   # Стабилизация по крену (влево/вправо)
        'pitch': True,  # Стабилизация по тангажу (вперед/назад)
        'yaw': False,    # Стабилизация по рысканию (поворот)
    },
    
    # Критические углы (диапазоны отклонения от эталона)
    'critical_angles': {
        'roll': {'left': 15.0, 'right': 15.0},   # Влево/вправо
        'pitch': {'forward': 15.0, 'backward': 17.0},  # Вперед/назад (отклонение от 90°)
        'yaw': {'left': 10.0, 'right': 10.0},    # Поворот влево/вправо
    },
    
    # Порог стабильности для остановки стабилизации (отклонение от эталона)
    'stable_threshold': {
        'roll': 30.0,
        'pitch': 30.0,
        'yaw': 30.0,
    },
    
    # Активация по угловой скорости (для быстрой реакции)
    'angular_velocity_enabled': True,  # Включить/выключить активацию по угловой скорости
    'angular_velocity_thresholds': {  # Пороги угловой скорости (рад/с) - при превышении сразу активируется стабилизация
        'roll': 0.3,   # Порог угловой скорости по roll (рад/с)
        'pitch': 0.3,  # Порог угловой скорости по pitch (рад/с)
        'yaw': 0.5,    # Порог угловой скорости по yaw (рад/с)
    },
    'oscillation_detection': {  # Обнаружение качания (осцилляции)
        'enabled': True,  # Включить/выключить обнаружение качания
        'direction_change_threshold': 0.5,  # Если направление угловой скорости меняется чаще чем раз в X секунд - это качание
        'min_velocity_for_oscillation': 0.1,  # Минимальная угловая скорость для учета изменения направления (рад/с)
    },
    
    # Cooldown после стабилизации
    'cooldown_after_stabilization': 1.0,  # Время после стабилизации, когда новая стабилизация не активируется (сек)
    
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
    
    # Коэффициенты для расчета амплитуды движения на основе отклонения от эталона
    'correction_coefficients': {
        'roll': 0.002,   # Коэффициент для движения по Y при отклонении roll (м/градус)
        'pitch': 0.002,  # Коэффициент для движения по X при отклонении pitch (м/градус)
        'yaw': 2.0,      # Коэффициент для поворота при отклонении yaw (градус/градус)
    },
    
    # Максимальные амплитуды движения стабилизации
    'max_amplitudes': {
        'x': 0.015,      # Максимальное движение вперед/назад (м)
        'y': 0.015,      # Максимальное движение влево/вправо (м)
        'angle': 8.0,    # Максимальный поворот (градусы)
    },
    
    # Интервал обновления параметров стабилизации
    'update_interval': 0.1,  # Интервал обновления параметров (сек)
    
    # Параметр возврата
    'return_enabled': False,  # Включить/выключить возврат после стабилизации
    'return_delay': 0.5,      # Задержка перед началом возврата (сек)
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
            'reference': None,  # Эталонная ориентация {'roll': x, 'pitch': y, 'yaw': z}
            'reference_set': False,  # Установлен ли эталон
            'reference_set_time': None,  # Время установки эталона
            'stabilization_active': False,  # Активна ли стабилизация
            'last_update_time': 0,  # Время последнего обновления параметров
            'movement_history': [],  # История перемещений для возврата [{'x': x, 'y': y, 'angle': a, 'time': t}, ...]
            'return_pending': False,  # Ожидается ли возврат
            'return_start_time': None,  # Время начала возврата
            'cooldown_until': 0,  # Время до которого стабилизация не активируется (cooldown)
            'angular_velocity_history': {  # История угловой скорости для обнаружения качания
                'roll': [],  # [(velocity, time, direction), ...]
                'pitch': [],
                'yaw': []
            },
        }
        
        # Инициализация эталона
        self._init_reference()
        
        rospy.loginfo("AutoStabilization module initialized (Rest + Walking)")
        rospy.loginfo("  Rest stabilization: Simplified dynamic approach")
    
    def _init_reference(self):
        """Инициализирует эталонную ориентацию в зависимости от режима."""
        config = self.rest_config
        mode = config.get('reference_mode', 'auto')
        
        if mode == 'manual':
            self.rest_stabilization_state['reference'] = config['reference_manual'].copy()
            self.rest_stabilization_state['reference_set'] = True
            rospy.loginfo(f"📐 Reference orientation (manual): {self.rest_stabilization_state['reference']}")
        elif mode == 'auto':
            # Эталон будет установлен автоматически через delay секунд
            self.rest_stabilization_state['reference_set'] = False
            rospy.loginfo(f"📐 Reference orientation: will be set automatically after {config['reference_auto_delay']}s")
        else:  # 'none'
            # Без эталона - используем абсолютные углы
            self.rest_stabilization_state['reference'] = {'roll': 0.0, 'pitch': 90.0, 'yaw': 0.0}
            self.rest_stabilization_state['reference_set'] = True
            rospy.loginfo("📐 Reference orientation: disabled (using absolute angles)")
    
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
        Упрощенная стабилизация: при выходе за лимит активируем ходьбу и обновляем параметры.
        
        Args:
            imu_data: Словарь с данными IMU
            robot_state: Состояние робота
            x_move_amp: Амплитуда движения по X (от джойстика)
            y_move_amp: Амплитуда движения по Y (от джойстика)
            angle_move_amp: Амплитуда поворота (от джойстика)
        
        Returns:
            bool: True если была выполнена стабилизация
        """
        # Проверяем, нет ли движения от джойстика
        joystick_moving = (abs(x_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(y_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(angle_move_amp) > JOYSTICK_MOVE_THRESHOLD)
        
        if joystick_moving:
            if self.rest_stabilization_state['stabilization_active']:
                rospy.loginfo("🎮 Joystick movement detected - stopping stabilization")
            self._reset_rest_stabilization_state()
            return False
        
        # Проверяем, не упал ли робот - если упал, прекращаем стабилизацию
        if robot_state != 'stand':
            if self.rest_stabilization_state['stabilization_active']:
                rospy.logwarn(f"⚠️ Robot fell ({robot_state}) - stopping stabilization")
                try:
                    self.gait_manager.stop()
                except Exception as e:
                    rospy.logwarn(f"Error stopping gait_manager: {e}")
                self._reset_rest_stabilization_state()
            return False
        
        # Получаем углы ориентации
        orientation = imu_data.get('orientation', {})
        if not orientation:
            return False
            
        roll_deg = orientation.get('roll_deg', 0)
        pitch_deg = orientation.get('pitch_deg', 0)
        yaw_deg = orientation.get('yaw_deg', 0)
        
        # Получаем угловую скорость
        angular_velocity = imu_data.get('angular_velocity', {})
        roll_vel = angular_velocity.get('x', 0)  # Угловая скорость по roll (рад/с)
        pitch_vel = angular_velocity.get('y', 0)  # Угловая скорость по pitch (рад/с)
        yaw_vel = angular_velocity.get('z', 0)   # Угловая скорость по yaw (рад/с)
        
        current_time = rospy.get_time()
        state = self.rest_stabilization_state
        config = self.rest_config
        
        # Устанавливаем эталон, если нужно (auto режим)
        if not state['reference_set'] and config['reference_mode'] == 'auto':
            if state['reference_set_time'] is None:
                state['reference_set_time'] = current_time
            elif current_time - state['reference_set_time'] >= config['reference_auto_delay']:
                state['reference'] = {'roll': roll_deg, 'pitch': pitch_deg, 'yaw': yaw_deg}
                state['reference_set'] = True
                rospy.loginfo(f"📐 Reference orientation (auto): {state['reference']}")
        
        if not state['reference_set']:
            return False  # Ждем установки эталона
        
        # Проверяем cooldown - если недавно была стабилизация, не активируем новую
        if current_time < state['cooldown_until']:
            return False  # В режиме cooldown
        
        # Вычисляем отклонения от эталона
        ref = state['reference']
        roll_dev = roll_deg - ref['roll']
        pitch_dev = pitch_deg - ref['pitch']
        yaw_dev = yaw_deg - ref['yaw']
        
        # Обновляем историю угловой скорости для обнаружения качания
        oscillation_detected = False
        if config.get('oscillation_detection', {}).get('enabled', True):
            oscillation_detected = self._check_oscillation(roll_vel, pitch_vel, yaw_vel, current_time, config)
            if oscillation_detected and state['stabilization_active']:
                # Обнаружено качание - прекращаем стабилизацию
                rospy.logwarn("🔄 Oscillation detected - stopping stabilization to prevent extra steps")
                try:
                    self.gait_manager.stop()
                except Exception as e:
                    rospy.logwarn(f"Error stopping gait_manager: {e}")
                state['stabilization_active'] = False
                state['cooldown_until'] = current_time + config.get('cooldown_after_stabilization', 1.0)
                return False
        
        # Проверяем, вышли ли за критические углы (с учетом включения/выключения осей)
        critical = config['critical_angles']
        enabled = config['stabilization_enabled']
        
        roll_exceeded = False
        if enabled['roll']:
            roll_critical = critical['roll']['left'] if roll_dev > 0 else critical['roll']['right']
            roll_exceeded = abs(roll_dev) > roll_critical
        
        pitch_exceeded = False
        if enabled['pitch']:
            pitch_critical = critical['pitch']['forward'] if pitch_dev < 0 else critical['pitch']['backward']
            pitch_exceeded = abs(pitch_dev) > pitch_critical
        
        yaw_exceeded = False
        if enabled['yaw']:
            yaw_critical = critical['yaw']['left'] if yaw_dev > 0 else critical['yaw']['right']
            yaw_exceeded = abs(yaw_dev) > yaw_critical
        
        # Проверяем угловую скорость (для быстрой реакции, только если не качание)
        velocity_exceeded = False
        if config.get('angular_velocity_enabled', True) and not oscillation_detected:
            vel_thresholds = config.get('angular_velocity_thresholds', {})
            # Активируем только если угловая скорость в одном направлении (не качание)
            if enabled['roll'] and abs(roll_vel) > vel_thresholds.get('roll', 0.3):
                # Проверяем, что скорость в одном направлении (не меняется знак)
                if self._is_velocity_consistent('roll', roll_vel, current_time, config):
                    velocity_exceeded = True
                    rospy.logwarn(f"⚡ High angular velocity (consistent): roll_vel={roll_vel:.3f} rad/s")
            if enabled['pitch'] and abs(pitch_vel) > vel_thresholds.get('pitch', 0.3):
                if self._is_velocity_consistent('pitch', pitch_vel, current_time, config):
                    velocity_exceeded = True
                    rospy.logwarn(f"⚡ High angular velocity (consistent): pitch_vel={pitch_vel:.3f} rad/s")
            if enabled['yaw'] and abs(yaw_vel) > vel_thresholds.get('yaw', 0.5):
                if self._is_velocity_consistent('yaw', yaw_vel, current_time, config):
                    velocity_exceeded = True
                    rospy.logwarn(f"⚡ High angular velocity (consistent): yaw_vel={yaw_vel:.3f} rad/s")
        
        needs_stabilization = roll_exceeded or pitch_exceeded or yaw_exceeded or velocity_exceeded
        
        # Обрабатываем возврат
        if config['return_enabled'] and state['return_pending']:
            if needs_stabilization:
                state['return_pending'] = False
                state['return_start_time'] = None
            elif state['return_start_time'] is None:
                state['return_start_time'] = current_time
            elif current_time - state['return_start_time'] >= config['return_delay']:
                return self._execute_return()
        
        # Проверяем стабильность (только по включенным осям)
        # НЕ учитываем угловую скорость для продолжения стабилизации - если углы стабильны, останавливаемся
        stable = config['stable_threshold']
        is_stable = True
        if enabled['roll']:
            is_stable = is_stable and abs(roll_dev) < stable['roll']
        if enabled['pitch']:
            is_stable = is_stable and abs(pitch_dev) < stable['pitch']
        if enabled['yaw']:
            is_stable = is_stable and abs(yaw_dev) < stable['yaw']
        
        if needs_stabilization:
            # Активируем стабилизацию
            if not state['stabilization_active']:
                reason = []
                if roll_exceeded:
                    reason.append(f"roll_dev={roll_dev:.2f}°")
                if pitch_exceeded:
                    reason.append(f"pitch_dev={pitch_dev:.2f}°")
                if yaw_exceeded:
                    reason.append(f"yaw_dev={yaw_dev:.2f}°")
                if velocity_exceeded:
                    reason.append(f"angular_vel: roll={roll_vel:.3f}, pitch={pitch_vel:.3f}, yaw={yaw_vel:.3f} rad/s")
                
                rospy.logwarn(f"⚠️ Stabilization started: {', '.join(reason)}")
                state['stabilization_active'] = True
                state['movement_history'] = []
            
            # Обновляем параметры движения
            if current_time - state['last_update_time'] >= config['update_interval']:
                self._update_stabilization_movement(roll_dev, pitch_dev, yaw_dev, current_time)
                state['last_update_time'] = current_time
                return True
        elif state['stabilization_active']:
            # Стабилизация завершена
            rospy.loginfo(f"✅ Stabilization: Robot stable (roll_dev={roll_dev:.2f}°, pitch_dev={pitch_dev:.2f}°, yaw_dev={yaw_dev:.2f}°)")
            try:
                self.gait_manager.stop()
            except Exception as e:
                rospy.logwarn(f"Error stopping gait_manager: {e}")
            state['stabilization_active'] = False
            # Устанавливаем cooldown после стабилизации
            state['cooldown_until'] = current_time + config.get('cooldown_after_stabilization', 1.0)
            # Очищаем историю угловой скорости
            state['angular_velocity_history'] = {'roll': [], 'pitch': [], 'yaw': []}
            
            if config['return_enabled'] and len(state['movement_history']) > 0:
                state['return_pending'] = True
                state['return_start_time'] = None
        
        return False
    
    def _update_stabilization_movement(self, roll_dev, pitch_dev, yaw_dev, current_time):
        """
        Обновляет параметры движения стабилизации через set_step (запускает движение).
        
        Args:
            roll_dev: Отклонение roll от эталона (градусы)
            pitch_dev: Отклонение pitch от эталона (градусы)
            yaw_dev: Отклонение yaw от эталона (градусы)
            current_time: Текущее время
        """
        config = self.rest_config
        state = self.rest_stabilization_state
        
        # Получаем параметры походки
        gait_param = self.gait_manager.get_gait_param().copy()
        gait_params = config['gait_params']
        period_time = list(gait_params['period_time'])
        
        # Применяем базовые параметры стабилизации
        gait_param.update(gait_params.get('gait_base', {}))
        gait_param['init_z_offset'] = self.init_z_offset
        
        # Вычисляем амплитуды движения на основе отклонений
        coeffs = config['correction_coefficients']
        max_amps = config['max_amplitudes']
        
        # Roll: отклонение влево (положительное) -> движение влево (положительный Y)
        y_amplitude = roll_dev * coeffs['roll']
        y_amplitude = max(-max_amps['y'], min(max_amps['y'], y_amplitude))
        
        # Pitch: отклонение вперед (отрицательное) -> движение вперед (положительный X)
        x_amplitude = -pitch_dev * coeffs['pitch']
        x_amplitude = max(-max_amps['x'], min(max_amps['x'], x_amplitude))
        
        # YAW: отклонение влево (положительное) -> поворот влево (положительный угол)
        # Только если стабилизация YAW включена
        if config['stabilization_enabled']['yaw']:
            angle_amplitude = yaw_dev * coeffs['yaw']
            angle_amplitude = max(-max_amps['angle'], min(max_amps['angle'], angle_amplitude))
        else:
            angle_amplitude = 0
        
        # Используем set_step для запуска движения (как в speed_control)
        try:
            self.gait_manager.set_step(
                period_time,
                x_amplitude,
                y_amplitude,
                angle_amplitude,
                gait_param,
                step_num=0  # Непрерывное движение (неблокирующий)
            )
            
            # Сохраняем в историю перемещений
            state['movement_history'].append({
                'x': x_amplitude,
                'y': y_amplitude,
                'angle': angle_amplitude,
                'time': current_time
            })
            
            # Ограничиваем размер истории (последние 100 записей)
            if len(state['movement_history']) > 100:
                state['movement_history'].pop(0)
            
            rospy.loginfo(f"🔄 Stabilization: X={x_amplitude:.4f}, Y={y_amplitude:.4f}, "
                         f"Angle={angle_amplitude:.2f}°, dev: roll={roll_dev:.2f}°, pitch={pitch_dev:.2f}°, yaw={yaw_dev:.2f}°")
        except Exception as e:
            rospy.logerr(f"Error updating stabilization movement: {e}")
    
    def _execute_return(self):
        """
        Выполняет возврат на основе истории перемещений.
        
        Returns:
            bool: True если возврат выполняется
        """
        state = self.rest_stabilization_state
        history = state['movement_history']
        
        if len(history) == 0:
            state['return_pending'] = False
            return False
        
        # Вычисляем суммарное смещение
        total_x = sum(h['x'] for h in history)
        total_y = sum(h['y'] for h in history)
        total_angle = sum(h['angle'] for h in history)
        
        # Вычисляем обратное движение (противоположное направление)
        return_x = -total_x / len(history) if len(history) > 0 else 0
        return_y = -total_y / len(history) if len(history) > 0 else 0
        return_angle = -total_angle / len(history) if len(history) > 0 else 0
        
        # Ограничиваем амплитуды
        config = self.rest_config
        max_amps = config['max_amplitudes']
        return_x = max(-max_amps['x'], min(max_amps['x'], return_x))
        return_y = max(-max_amps['y'], min(max_amps['y'], return_y))
        return_angle = max(-max_amps['angle'], min(max_amps['angle'], return_angle))
        
        if abs(return_x) < 0.001 and abs(return_y) < 0.001 and abs(return_angle) < 0.1:
            # Возврат завершен
            rospy.loginfo("✅ Return completed")
            try:
                self.gait_manager.stop()
            except Exception as e:
                rospy.logwarn(f"Error stopping gait_manager after return: {e}")
            state['return_pending'] = False
            state['movement_history'] = []
            return False
        
        # Выполняем возврат через set_step
        gait_param = self.gait_manager.get_gait_param().copy()
        gait_params = config['gait_params']
        period_time = list(gait_params['period_time'])
        gait_param.update(gait_params.get('gait_base', {}))
        gait_param['init_z_offset'] = self.init_z_offset
        
        try:
            self.gait_manager.set_step(
                period_time,
                return_x,
                return_y,
                return_angle,
                gait_param,
                step_num=0  # Непрерывное движение
            )
            rospy.loginfo(f"↩️ Return: X={return_x:.4f}, Y={return_y:.4f}, Angle={return_angle:.2f}°")
            return True
        except Exception as e:
            rospy.logerr(f"Error executing return: {e}")
            return False
    
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
    
    def _check_oscillation(self, roll_vel, pitch_vel, yaw_vel, current_time, config):
        """
        Проверяет, есть ли качание (осцилляция) по угловой скорости.
        
        Args:
            roll_vel: Угловая скорость по roll (рад/с)
            pitch_vel: Угловая скорость по pitch (рад/с)
            yaw_vel: Угловая скорость по yaw (рад/с)
            current_time: Текущее время
            config: Конфигурация
        
        Returns:
            bool: True если обнаружено качание
        """
        osc_config = config.get('oscillation_detection', {})
        if not osc_config.get('enabled', True):
            return False
        
        threshold = osc_config.get('direction_change_threshold', 0.5)
        min_vel = osc_config.get('min_velocity_for_oscillation', 0.1)
        state = self.rest_stabilization_state
        enabled = config['stabilization_enabled']
        
        # Проверяем каждую ось
        for axis, vel, axis_name in [('roll', roll_vel, 'roll'), ('pitch', pitch_vel, 'pitch'), ('yaw', yaw_vel, 'yaw')]:
            if not enabled[axis]:
                continue
            
            if abs(vel) < min_vel:
                continue  # Скорость слишком мала для учета
            
            history = state['angular_velocity_history'][axis]
            direction = 1 if vel > 0 else -1
            
            # Добавляем текущее значение
            history.append({
                'velocity': vel,
                'direction': direction,
                'time': current_time
            })
            
            # Ограничиваем размер истории (последние 20 записей)
            if len(history) > 20:
                history.pop(0)
            
            # Проверяем, как часто меняется направление
            if len(history) >= 3:
                # Считаем изменения направления за последние записи
                direction_changes = 0
                for i in range(1, len(history)):
                    if history[i]['direction'] != history[i-1]['direction']:
                        direction_changes += 1
                
                # Если направление меняется часто - это качание
                if direction_changes >= 2:
                    time_span = history[-1]['time'] - history[0]['time']
                    if time_span > 0 and direction_changes / time_span > 1.0 / threshold:
                        rospy.logwarn(f"🔄 Oscillation detected on {axis_name}: {direction_changes} direction changes in {time_span:.2f}s")
                        return True
        
        return False
    
    def _is_velocity_consistent(self, axis, velocity, current_time, config):
        """
        Проверяет, что угловая скорость в одном направлении (не меняется знак).
        
        Args:
            axis: Ось ('roll', 'pitch', 'yaw')
            velocity: Текущая угловая скорость (рад/с)
            current_time: Текущее время
            config: Конфигурация
        
        Returns:
            bool: True если скорость в одном направлении
        """
        osc_config = config.get('oscillation_detection', {})
        min_vel = osc_config.get('min_velocity_for_oscillation', 0.1)
        
        if abs(velocity) < min_vel:
            return False  # Скорость слишком мала
        
        state = self.rest_stabilization_state
        history = state['angular_velocity_history'][axis]
        current_direction = 1 if velocity > 0 else -1
        
        # Если история пуста или последнее направление совпадает - скорость согласована
        if len(history) == 0:
            return True
        
        # Проверяем последние несколько записей
        recent_count = min(3, len(history))
        consistent = True
        for i in range(-recent_count, 0):
            if history[i]['direction'] != current_direction:
                consistent = False
                break
        
        return consistent
    
    def _reset_rest_stabilization_state(self):
        """Сбрасывает состояние стабилизации в покое."""
        state = self.rest_stabilization_state
        state['stabilization_active'] = False
        state['return_pending'] = False
        state['return_start_time'] = None
        state['movement_history'] = []
        state['last_update_time'] = 0
        state['cooldown_until'] = 0
        state['angular_velocity_history'] = {'roll': [], 'pitch': [], 'yaw': []}
    
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
