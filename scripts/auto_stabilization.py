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
        'roll': 20.0,
        'pitch': 20.0,
        'yaw': 20.0,
    },
    
    # Параметры угловой скорости для активации и первого шага стабилизации
    'angular_velocity': {
        'activation_enabled': True,  # Включить/выключить активацию стабилизации по угловой скорости
        'activation_thresholds': {  # Пороговые значения угловой скорости для активации стабилизации (рад/с)
            'roll': 0.5,   # Минимальная угловая скорость по roll для активации
            'pitch': 0.5,  # Минимальная угловая скорость по pitch для активации
            'yaw': 0.5,    # Минимальная угловая скорость по yaw для активации
        },
        'min_velocity': 0.1,  # Минимальная угловая скорость для использования в первом шаге (рад/с)
        'coefficients': {
            'roll': 0.015,   # Коэффициент для roll (м/(рад/с)) - конвертация скорости в амплитуду Y
            'pitch': 0.015,  # Коэффициент для pitch (м/(рад/с)) - конвертация скорости в амплитуду X
            'yaw': 15.0,     # Коэффициент для yaw (градус/(рад/с)) - конвертация скорости в угол поворота
        },
    },
    
    # Определение качания (oscillation) - для предотвращения лишних шагов
    'oscillation_detection': {
        'enabled': True,  # Включить/выключить определение качания
        'direction_change_threshold': 1.2,  # Время в секундах - если направление угловой скорости меняется чаще, это качание
        'min_velocity_for_oscillation': 0.1,  # Минимальная угловая скорость для учета в определении качания (рад/с)
    },
    
    # Cooldown после стабилизации - период, в течение которого новая стабилизация не активируется
    'cooldown_after_stabilization': 1.6,  # Время в секундах после завершения стабилизации, когда новая не активируется
    
    # Параметры для gait_manager при стабилизации (отдельные от джойстика)
    # ВАЖНО: Согласно документации (Lesson 5, 7), первый параметр set_step() - это список:
    # [period_time, dsp_ratio, y_swap_amplitude]
    # где:
    #   - period_time: время подъема одной ноги за цикл (мс)
    #   - dsp_ratio: доля времени в цикле, когда обе ноги на земле (0-1)
    #   - y_swap_amplitude: амплитуда качания влево-вправо при ходьбе (м)
    'gait_params': {
        'period_time': [260, 0.25, 0.025],  # [period_ms, dsp_ratio, y_swap_amplitude] - быстрые шаги
        'gait_base': {
            # ПРИМЕЧАНИЕ: dsp_ratio и y_swap_amplitude теперь в period_time[1] и period_time[2]
            # Остальные параметры походки:
            'step_fb_ratio': 0.020,         # Базовая дистанция шага вперед/назад (м)
            'z_swap_amplitude': 0.011,      # Амплитуда подъема ноги (м) - высота шага
            'init_y_offset': -0.005,        # Начальное смещение по Y (м)
            'init_roll_offset': 0.0,        # Начальное смещение по roll (градусы)
            'init_pitch_offset': 0.0        # Начальное смещение по pitch (градусы)
        }
    },
    
    # Коэффициенты для расчета амплитуды движения на основе отклонения от эталона
    'correction_coefficients': {
        'roll': 0.002,   # Коэффициент для движения по Y при отклонении roll (м/градус)
        'pitch': 0.002,  # Коэффициент для движения по X при отклонении pitch (м/градус)
        'yaw': 2.0,      # Коэффициент для поворота при отклонении yaw (градус/градус)
    },
    
    # Максимальные амплитуды движения стабилизации
    # ВАЖНО: Согласно документации (Lesson 4, 7):
    # - x_amplitude: длина шага в направлении X (м) - расстояние за один шаг
    # - y_amplitude: длина шага в направлении Y (м) - расстояние за один шаг
    # - rotation_angle: угол поворота (градусы)
    # ПРИМЕЧАНИЕ: В документации указано, что каждый параметр имеет свой диапазон,
    # и превышение лимитов может привести к повреждению сервоприводов.
    # Примеры из документации: x_move_amplitude=0.03 м, angle_move_amplitude=0.04 (вероятно градусы)
    # Текущие значения установлены консервативно для безопасности.
    # МАКСИМАЛЬНЫЕ ЗНАЧЕНИЯ (требуют тестирования):
    # - x_amplitude: до 0.05-0.08 м (примеры в speed_control: 0.01-0.015 м)
    # - y_amplitude: до 0.05-0.08 м (примеры в speed_control: 0.015 м)
    # - rotation_angle: до 15-20 градусов (примеры в speed_control: 8-10 градусов)
    'max_amplitudes': {
        'x': 0.075,      # Максимальная длина шага вперед/назад (м) - ТЕКУЩЕЕ: консервативное значение
        'y': 0.035,      # Максимальная длина шага влево/вправо (м) - ТЕКУЩЕЕ: консервативное значение
        'angle': 8.0,    # Максимальный угол поворота (градусы) - ТЕКУЩЕЕ: консервативное значение
    },
    
    # Максимальные параметры походки (для справки и будущего использования)
    # Согласно документации и примерам в speed_control.py:
    'max_gait_params': {
        'period_time': {
            'min': 200,      # Минимальный период (мс) - быстрые шаги, может быть нестабильно
            'max': 800,      # Максимальный период (мс) - медленные шаги, более стабильно
            'current': 400,  # Текущее значение (мс)
            'note': 'Время подъема одной ноги за цикл. Меньше = быстрее, но менее стабильно'
        },
        'dsp_ratio': {
            'min': 0.15,     # Минимальная доля двойной опоры (быстрее, менее стабильно)
            'max': 0.35,     # Максимальная доля двойной опоры (медленнее, более стабильно)
            'current': 0.15, # Текущее значение
            'note': 'Доля времени в цикле, когда обе ноги на земле (0-1). Больше = стабильнее'
        },
        'y_swap_amplitude': {
            'min': 0.015,    # Минимальная амплитуда качания (м)
            'max': 0.040,    # Максимальная амплитуда качания (м)
            'current': 0.025,# Текущее значение (м)
            'note': 'Амплитуда качания влево-вправо при ходьбе (м)'
        },
        'step_fb_ratio': {
            'min': 0.015,    # Минимальная базовая дистанция шага (м)
            'max': 0.040,    # Максимальная базовая дистанция шага (м) - примеры: 0.025-0.030
            'current': 0.040,# Текущее значение (м)
            'note': 'Базовая дистанция шага вперед/назад (м). Может взаимодействовать с x_amplitude'
        },
        'z_swap_amplitude': {
            'min': 0.005,    # Минимальная высота подъема ноги (м)
            'max': 0.020,    # Максимальная высота подъема ноги (м) - примеры: 0.006-0.011
            'current': 0.011,# Текущее значение (м)
            'note': 'Амплитуда подъема ноги - высота шага (м)'
        },
    },
    
    # Интервал обновления параметров стабилизации (0 = обновлять каждый раз, без задержки)
    'update_interval': 0.0,  # Интервал обновления параметров (сек) - 0 для максимальной скорости реакции
    
    # Параметр возврата
    'return_enabled': False,  # Включить/выключить возврат после стабилизации
    'return_delay': 0.5,      # Задержка перед началом возврата (сек)
}

# ========== КОНФИГУРАЦИЯ: СТАБИЛИЗАЦИЯ ПРИ ХОДЬБЕ ==========
WALKING_STABILIZATION_CONFIG = {
    # Включение/выключение стабилизации при ходьбе
    'enabled': True,  # Глобальное включение/выключение PID стабилизации
    
    # PID параметры для удержания вертикального положения туловища
    'pid': {
        'roll': {
            'enabled': True,  # Включить/выключить стабилизацию по roll
            'kp': 0.01,       # Пропорциональный коэффициент (м/градус)
            'ki': 0.001,      # Интегральный коэффициент
            'kd': 0.005,      # Дифференциальный коэффициент
            'max_output': 0.02,  # Максимальное смещение по Y (м)
            'target': 0.0,    # Целевой угол roll (градусы) - вертикально
            'integral_limit': 10.0,  # Лимит интегральной составляющей (anti-windup)
        },
        'pitch': {
            'enabled': True,  # Включить/выключить стабилизацию по pitch
            'kp': 0.01,       # Пропорциональный коэффициент (м/градус)
            'ki': 0.001,      # Интегральный коэффициент
            'kd': 0.005,      # Дифференциальный коэффициент
            'max_output': 0.02,  # Максимальное смещение по X (м)
            'target': 90.0,   # Целевой угол pitch (градусы) - вертикально
            'integral_limit': 10.0,  # Лимит интегральной составляющей (anti-windup)
        },
    },
    
    # Пороги активации (не реагируем на мелкие отклонения)
    'activation_threshold': {
        'roll': 2.0,   # Минимальное отклонение для активации (градусы)
        'pitch': 2.0,  # Минимальное отклонение для активации (градусы)
    },
    
    # Ограничения на смещения туловища (безопасные пределы)
    'limits': {
        'init_x_offset': {'min': -0.03, 'max': 0.03},  # м
        'init_y_offset': {'min': -0.03, 'max': 0.03},  # м
        'init_roll_offset': {'min': -5.0, 'max': 5.0},  # градусы
        'init_pitch_offset': {'min': -5.0, 'max': 5.0},  # градусы
    },
    
    # Коэффициенты для преобразования PID выхода в смещения туловища
    'correction_ratios': {
        'roll_to_y_offset': 1.0,      # Коэффициент для init_y_offset от roll PID
        'roll_to_roll_offset': 0.5,   # Коэффициент для init_roll_offset от roll PID
        'pitch_to_x_offset': 1.0,     # Коэффициент для init_x_offset от pitch PID
        'pitch_to_pitch_offset': 0.5, # Коэффициент для init_pitch_offset от pitch PID
    },
    
    # Фильтрация данных IMU (для уменьшения шума)
    'filter': {
        'enabled': True,
        'alpha': 0.7,  # Коэффициент фильтрации (0-1), больше = меньше фильтрации
    },
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
        self.enabled = False
        self.rest_enabled = True
        self.walking_enabled = True
        
        # Состояние стабилизации при ходьбе (PID регулятор)
        self.walking_stabilization_state = {
            'last_imu_data': None,  # Последние данные IMU
            'last_time': None,      # Время последнего обновления
            'integral': {'roll': 0.0, 'pitch': 0.0},  # Интегральная составляющая PID
            'last_error': {'roll': 0.0, 'pitch': 0.0},  # Последняя ошибка для дифференциальной составляющей
            'filtered_orientation': {'roll': 0.0, 'pitch': 90.0},  # Отфильтрованная ориентация
        }
        
        # Состояние стабилизации в покое
        self.rest_stabilization_state = {
            'reference': None,  # Эталонная ориентация {'roll': x, 'pitch': y, 'yaw': z}
            'reference_set': False,  # Установлен ли эталон
            'reference_set_time': None,  # Время установки эталона
            'stabilization_active': False,  # Активна ли стабилизация
            'first_step': True,  # Первый шаг стабилизации (используем угловую скорость)
            'last_update_time': 0,  # Время последнего обновления параметров
            'movement_history': [],  # История перемещений для возврата [{'x': x, 'y': y, 'angle': a, 'time': t}, ...]
            'return_pending': False,  # Ожидается ли возврат
            'return_start_time': None,  # Время начала возврата
            'cooldown_until': 0,  # Время до которого действует cooldown (не активировать новую стабилизацию)
            'angular_velocity_history': [],  # История угловых скоростей для определения качания [{'roll': r, 'pitch': p, 'yaw': y, 'time': t, 'direction': d}, ...]
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
            # Сохраняем последние данные IMU для использования в apply_walking_corrections
            self.walking_stabilization_state['last_imu_data'] = imu_data
        
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
        # КРИТИЧНО: Проверяем состояние робота ПЕРВЫМ делом - если не стоит, немедленно останавливаем стабилизацию
        # Это предотвращает конфликт с логикой падений и подъема
        if robot_state != 'stand':
            # ВСЕГДА останавливаем движение, если робот не стоит (даже если стабилизация не активна)
            if self.rest_stabilization_state['stabilization_active']:
                rospy.logwarn(f"⚠️ Robot not standing ({robot_state}) - stopping stabilization immediately")
            else:
                # Даже если стабилизация не активна, но робот не стоит - останавливаем на всякий случай
                rospy.logdebug(f"⚠️ Robot not standing ({robot_state}) - ensuring gait_manager is stopped")
            try:
                self.gait_manager.stop()
            except Exception as e:
                rospy.logwarn(f"Error stopping gait_manager: {e}")
            self._reset_rest_stabilization_state()
            return False  # Не запускаем и не продолжаем стабилизацию если робот не стоит
        
        # Проверяем, нет ли движения от джойстика
        joystick_moving = (abs(x_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(y_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(angle_move_amp) > JOYSTICK_MOVE_THRESHOLD)
        
        if joystick_moving:
            if self.rest_stabilization_state['stabilization_active']:
                rospy.loginfo("🎮 Joystick movement detected - stopping stabilization")
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
        
        # Получаем угловую скорость (для первого шага)
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
        
        # Вычисляем отклонения от эталона
        ref = state['reference']
        roll_dev = roll_deg - ref['roll']
        pitch_dev = pitch_deg - ref['pitch']
        yaw_dev = yaw_deg - ref['yaw']
        
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
        
        # Проверяем активацию по угловой скорости (быстрая реакция)
        velocity_exceeded = False
        if config.get('angular_velocity', {}).get('activation_enabled', False):
            vel_config = config.get('angular_velocity', {})
            vel_thresholds = vel_config.get('activation_thresholds', {})
            
            if enabled['roll'] and abs(roll_vel) > vel_thresholds.get('roll', 0.3):
                velocity_exceeded = True
            if enabled['pitch'] and abs(pitch_vel) > vel_thresholds.get('pitch', 0.3):
                velocity_exceeded = True
            if enabled['yaw'] and abs(yaw_vel) > vel_thresholds.get('yaw', 0.3):
                velocity_exceeded = True
        
        # Стабилизация нужна если превышены углы ИЛИ угловая скорость
        needs_stabilization = roll_exceeded or pitch_exceeded or yaw_exceeded or velocity_exceeded
        
        # Проверяем cooldown - не активируем новую стабилизацию сразу после предыдущей
        if needs_stabilization and current_time < state['cooldown_until']:
            rospy.logdebug(f"⏳ Stabilization blocked by cooldown (until {state['cooldown_until']:.2f}s)")
            needs_stabilization = False
        
        # Обновляем историю угловых скоростей для определения качания
        if config.get('oscillation_detection', {}).get('enabled', False):
            self._update_angular_velocity_history(roll_vel, pitch_vel, yaw_vel, current_time, config)
        
        # Проверяем качание - если робот качается, прекращаем стабилизацию
        if state['stabilization_active'] and config.get('oscillation_detection', {}).get('enabled', False):
            if self._check_oscillation(roll_vel, pitch_vel, yaw_vel, current_time, config):
                rospy.logwarn("🔄 Oscillation detected - stopping stabilization")
                try:
                    self.gait_manager.stop()
                except Exception as e:
                    rospy.logwarn(f"Error stopping gait_manager: {e}")
                state['stabilization_active'] = False
                state['cooldown_until'] = current_time + config.get('cooldown_after_stabilization', 1.0)
                return False
        
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
                # Определяем причину активации
                activation_reason = []
                if roll_exceeded or pitch_exceeded or yaw_exceeded:
                    activation_reason.append(f"angles (roll_dev={roll_dev:.2f}°, pitch_dev={pitch_dev:.2f}°, yaw_dev={yaw_dev:.2f}°)")
                if velocity_exceeded:
                    activation_reason.append(f"velocity (roll_vel={roll_vel:.3f}, pitch_vel={pitch_vel:.3f}, yaw_vel={yaw_vel:.3f} rad/s)")
                
                reason_str = " + ".join(activation_reason)
                rospy.logwarn(f"⚠️ Stabilization started: {reason_str}")
                state['stabilization_active'] = True
                state['first_step'] = True  # Первый шаг - используем угловую скорость
                state['movement_history'] = []
                state['last_update_time'] = 0  # Сбрасываем таймер для немедленного выполнения
            
            # Обновляем параметры движения (без задержки для максимальной скорости реакции)
            # Если update_interval = 0, обновляем каждый раз
            update_interval = config.get('update_interval', 0.0)
            if update_interval <= 0 or current_time - state['last_update_time'] >= update_interval:
                self._update_stabilization_movement(
                    roll_dev, pitch_dev, yaw_dev,
                    roll_vel, pitch_vel, yaw_vel,
                    state['first_step'],
                    current_time
                )
                state['first_step'] = False  # После первого шага переключаемся на углы
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
            # Устанавливаем cooldown после завершения стабилизации
            state['cooldown_until'] = current_time + config.get('cooldown_after_stabilization', 1.0)
            
            if config['return_enabled'] and len(state['movement_history']) > 0:
                state['return_pending'] = True
                state['return_start_time'] = None
        
        return False
    
    def _update_stabilization_movement(self, roll_dev, pitch_dev, yaw_dev,
                                       roll_vel, pitch_vel, yaw_vel,
                                       first_step, current_time):
        """
        Обновляет параметры движения стабилизации через set_step (запускает движение).
        
        Args:
            roll_dev: Отклонение roll от эталона (градусы)
            pitch_dev: Отклонение pitch от эталона (градусы)
            yaw_dev: Отклонение yaw от эталона (градусы)
            roll_vel: Угловая скорость по roll (рад/с)
            pitch_vel: Угловая скорость по pitch (рад/с)
            yaw_vel: Угловая скорость по yaw (рад/с)
            first_step: True если это первый шаг стабилизации (используем угловую скорость)
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
        
        # Вычисляем амплитуды движения
        coeffs = config['correction_coefficients']
        max_amps = config['max_amplitudes']
        
        if first_step:
            # Первый шаг: используем угловую скорость для определения направления
            # Это дает быструю реакцию даже при небольших углах
            # ВАЖНО: Используем ту же логику знаков, что и для углов, чтобы направления совпадали
            
            vel_config = config.get('angular_velocity', {})
            min_vel = vel_config.get('min_velocity', 0.2)
            vel_coeffs = vel_config.get('coefficients', {
                'roll': 0.015,
                'pitch': 0.015,
                'yaw': 15.0
            })
            
            # Roll: используем угловую скорость с той же логикой знаков, что и для углов
            # roll_dev > 0 (отклонение влево) -> y > 0 (шаг влево) -> y = roll_dev * coeff
            # Для угловой скорости: roll_vel > 0 (падение влево) -> y > 0 (шаг влево) -> y = roll_vel * vel_coeff
            if abs(roll_vel) > min_vel:
                y_amplitude = roll_vel * vel_coeffs['roll']  # Тот же знак, что и для углов
            else:
                # Если скорость мала, используем углы
                y_amplitude = roll_dev * coeffs['roll']
            
            # Pitch: используем угловую скорость с той же логикой знаков, что и для углов
            # pitch_dev < 0 (наклон вперед) -> x > 0 (шаг вперед) -> x = -pitch_dev * coeff
            # Для угловой скорости: pitch_vel < 0 (падение вперед) -> x > 0 (шаг вперед) -> x = -pitch_vel * vel_coeff
            if abs(pitch_vel) > min_vel:
                x_amplitude = pitch_vel * vel_coeffs['pitch']  # Тот же знак, что и для углов
            else:
                x_amplitude = pitch_dev * coeffs['pitch']
            
            # YAW: используем угловую скорость с той же логикой знаков, что и для углов
            # yaw_dev > 0 (отклонение влево) -> angle > 0 (поворот влево) -> angle = yaw_dev * coeff
            # Для угловой скорости: yaw_vel > 0 (поворот влево) -> angle > 0 (поворот влево) -> angle = yaw_vel * vel_coeff
            if config['stabilization_enabled']['yaw']:
                if abs(yaw_vel) > min_vel:
                    angle_amplitude = yaw_vel * vel_coeffs['yaw']  # Тот же знак, что и для углов
                else:
                    angle_amplitude = yaw_dev * coeffs['yaw']
            else:
                angle_amplitude = 0.0  # Явно устанавливаем 0.0 когда yaw выключен
            
            rospy.loginfo(f"⚡ First step (velocity-based): roll_vel={roll_vel:.3f} (dev={roll_dev:.2f}°), pitch_vel={pitch_vel:.3f} (dev={pitch_dev:.2f}°), yaw_vel={yaw_vel:.3f} (dev={yaw_dev:.2f}°)")
        else:
            # Последующие шаги: используем отклонения углов
            # Roll: отклонение влево (положительное) -> движение влево (положительный Y)
            y_amplitude = roll_dev * coeffs['roll']
            
            # Pitch: отклонение вперед (отрицательное) -> движение вперед (положительный X)
            x_amplitude = pitch_dev * coeffs['pitch']
            
            # YAW: отклонение влево (положительное) -> поворот влево (положительный угол)
            if config['stabilization_enabled']['yaw']:
                angle_amplitude = yaw_dev * coeffs['yaw']
            else:
                angle_amplitude = 0.0  # Явно устанавливаем 0.0 когда yaw выключен
        
        # Ограничиваем амплитуды
        y_amplitude = max(-max_amps['y'], min(max_amps['y'], y_amplitude))
        x_amplitude = max(-max_amps['x'], min(max_amps['x'], x_amplitude))
        # ВАЖНО: Ограничиваем angle_amplitude ТОЛЬКО если yaw стабилизация включена
        # Если выключена, angle_amplitude уже 0.0 и не должен изменяться
        if config['stabilization_enabled']['yaw']:
            angle_amplitude = max(-max_amps['angle'], min(max_amps['angle'], angle_amplitude))
        else:
            # Гарантируем, что angle_amplitude = 0 когда yaw выключен
            angle_amplitude = 0.0
        
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
            
            step_type = "velocity" if first_step else "angle"
            rospy.loginfo(f"🔄 Stabilization ({step_type}): X={x_amplitude:.4f}, Y={y_amplitude:.4f}, "
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
        x_amplitude = pitch_deviation * coeffs['pitch']  # Отрицательный, так как наклон вперед требует шага вперед
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
            period_time: Список [period_time, dsp_ratio, y_swap_amplitude] (может быть изменен)
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
            # ВАЖНО: y_swap_amplitude теперь в period_time[2], а не в gait_param
            if len(period_time) > 2:
                period_time[2] *= (1.0 + amplitude_increase)  # y_swap_amplitude
    
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
    
    def _update_angular_velocity_history(self, roll_vel, pitch_vel, yaw_vel, current_time, config):
        """
        Обновляет историю угловых скоростей для определения качания.
        
        Args:
            roll_vel: Угловая скорость по roll (рад/с)
            pitch_vel: Угловая скорость по pitch (рад/с)
            yaw_vel: Угловая скорость по yaw (рад/с)
            current_time: Текущее время
            config: Конфигурация стабилизации
        """
        state = self.rest_stabilization_state
        history = state['angular_velocity_history']
        osc_config = config.get('oscillation_detection', {})
        min_vel = osc_config.get('min_velocity_for_oscillation', 0.1)
        
        # Определяем направление для каждой оси (1 = положительное, -1 = отрицательное, 0 = слишком мало)
        directions = {}
        if abs(roll_vel) > min_vel:
            directions['roll'] = 1 if roll_vel > 0 else -1
        else:
            directions['roll'] = 0
            
        if abs(pitch_vel) > min_vel:
            directions['pitch'] = 1 if pitch_vel > 0 else -1
        else:
            directions['pitch'] = 0
            
        if abs(yaw_vel) > min_vel:
            directions['yaw'] = 1 if yaw_vel > 0 else -1
        else:
            directions['yaw'] = 0
        
        # Добавляем запись в историю
        history.append({
            'roll': roll_vel,
            'pitch': pitch_vel,
            'yaw': yaw_vel,
            'time': current_time,
            'direction': directions
        })
        
        # Ограничиваем размер истории (последние 50 записей)
        if len(history) > 50:
            history.pop(0)
    
    def _check_oscillation(self, roll_vel, pitch_vel, yaw_vel, current_time, config):
        """
        Проверяет, происходит ли качание (частая смена направления угловой скорости).
        
        Args:
            roll_vel: Угловая скорость по roll (рад/с)
            pitch_vel: Угловая скорость по pitch (рад/с)
            yaw_vel: Угловая скорость по yaw (рад/с)
            current_time: Текущее время
            config: Конфигурация стабилизации
        
        Returns:
            bool: True если обнаружено качание
        """
        state = self.rest_stabilization_state
        history = state['angular_velocity_history']
        osc_config = config.get('oscillation_detection', {})
        threshold = osc_config.get('direction_change_threshold', 0.5)
        
        if len(history) < 3:
            return False  # Недостаточно данных
        
        # Проверяем каждую ось на частую смену направления
        enabled = config['stabilization_enabled']
        
        for axis in ['roll', 'pitch', 'yaw']:
            if not enabled.get(axis, False):
                continue  # Пропускаем выключенные оси
            
            # Подсчитываем смены направления за последние threshold секунд
            direction_changes = 0
            last_direction = None
            last_change_time = None
            
            for entry in history:
                if current_time - entry['time'] > threshold:
                    continue  # Слишком старые записи
                
                direction = entry['direction'].get(axis, 0)
                if direction == 0:
                    continue  # Пропускаем слишком малые скорости
                
                if last_direction is not None and direction != last_direction:
                    # Направление изменилось
                    if last_change_time is None or (entry['time'] - last_change_time) < threshold:
                        direction_changes += 1
                    last_change_time = entry['time']
                
                last_direction = direction
            
            # Если направление меняется слишком часто (более 2 раз за threshold секунд), это качание
            if direction_changes >= 2:
                rospy.logwarn(f"🔄 Oscillation detected on {axis} axis: {direction_changes} direction changes in {threshold}s")
                return True
        
        return False
    
    def _reset_rest_stabilization_state(self):
        """Сбрасывает состояние стабилизации в покое."""
        state = self.rest_stabilization_state
        state['stabilization_active'] = False
        state['first_step'] = True  # Сбрасываем для следующей активации
        state['return_pending'] = False
        state['return_start_time'] = None
        state['movement_history'] = []
        state['last_update_time'] = 0
        state['cooldown_until'] = 0
        state['angular_velocity_history'] = []
    
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
        Применяет PID-корректировки для удержания вертикального положения туловища.
        Меняет только смещения туловища (init_x_offset, init_y_offset, init_roll_offset, init_pitch_offset),
        НЕ трогая базовые параметры походки (period_time, dsp_ratio, y_swap_amplitude, step_fb_ratio и т.д.).
        
        Вызывается из speed_control.process_axes ПОСЛЕ установки базовых параметров.
        
        Args:
            gait_param: Словарь параметров походки (будет изменен)
            period_time: Список [period_time, dsp_ratio, y_swap_amplitude] (не изменяется)
        
        Returns:
            bool: True если были применены корректировки
        """
        config = self.walking_config
        
        # Проверяем, включена ли стабилизация
        if not config.get('enabled', True):
            return False
        
        # Получаем последние данные IMU
        imu_data = self.walking_stabilization_state.get('last_imu_data')
        if not imu_data:
            return False
        
        orientation = imu_data.get('orientation', {})
        angular_velocity = imu_data.get('angular_velocity', {})
        
        if not orientation:
            return False
        
        roll_deg = orientation.get('roll_deg', 0)
        pitch_deg = orientation.get('pitch_deg', 0)
        
        # Применяем фильтрацию, если включена
        if config['filter']['enabled']:
            alpha = config['filter']['alpha']
            state = self.walking_stabilization_state
            state['filtered_orientation']['roll'] = alpha * state['filtered_orientation']['roll'] + (1 - alpha) * roll_deg
            state['filtered_orientation']['pitch'] = alpha * state['filtered_orientation']['pitch'] + (1 - alpha) * pitch_deg
            roll_deg = state['filtered_orientation']['roll']
            pitch_deg = state['filtered_orientation']['pitch']
        
        # Вычисляем ошибки относительно целевых углов
        roll_error = roll_deg - config['pid']['roll']['target']
        pitch_error = pitch_deg - config['pid']['pitch']['target']
        
        # Проверяем пороги активации
        if abs(roll_error) < config['activation_threshold']['roll'] and \
           abs(pitch_error) < config['activation_threshold']['pitch']:
            # Отклонения малы, сбрасываем интегральную составляющую
            self.walking_stabilization_state['integral'] = {'roll': 0.0, 'pitch': 0.0}
            return False
        
        # Вычисляем dt для интегральной и дифференциальной составляющих
        current_time = rospy.get_time()
        last_time = self.walking_stabilization_state.get('last_time')
        dt = current_time - last_time if last_time else 0.02  # ~50Hz по умолчанию
        dt = max(0.001, min(0.1, dt))  # Ограничиваем dt разумными пределами
        self.walking_stabilization_state['last_time'] = current_time
        
        # Получаем угловые скорости для дифференциальной составляющей
        roll_vel = angular_velocity.get('x', 0) * 180.0 / math.pi  # рад/с -> град/с
        pitch_vel = angular_velocity.get('y', 0) * 180.0 / math.pi
        
        # Вычисляем PID выходы
        roll_pid_output = 0.0
        pitch_pid_output = 0.0
        
        if config['pid']['roll']['enabled']:
            roll_pid_output = self._calculate_pid(
                error=roll_error,
                integral=self.walking_stabilization_state['integral']['roll'],
                derivative=roll_vel,
                dt=dt,
                config=config['pid']['roll']
            )
            # Обновляем интегральную составляющую
            self.walking_stabilization_state['integral']['roll'] += roll_error * dt
            # Ограничиваем интегральную составляющую (anti-windup)
            max_integral = config['pid']['roll']['integral_limit']
            self.walking_stabilization_state['integral']['roll'] = max(-max_integral, min(max_integral, self.walking_stabilization_state['integral']['roll']))
        
        if config['pid']['pitch']['enabled']:
            pitch_pid_output = self._calculate_pid(
                error=pitch_error,
                integral=self.walking_stabilization_state['integral']['pitch'],
                derivative=pitch_vel,
                dt=dt,
                config=config['pid']['pitch']
            )
            # Обновляем интегральную составляющую
            self.walking_stabilization_state['integral']['pitch'] += pitch_error * dt
            # Ограничиваем интегральную составляющую (anti-windup)
            max_integral = config['pid']['pitch']['integral_limit']
            self.walking_stabilization_state['integral']['pitch'] = max(-max_integral, min(max_integral, self.walking_stabilization_state['integral']['pitch']))
        
        # Применяем корректировки к gait_param
        # Roll: смещаем туловище влево/вправо и добавляем крен
        limits = config['limits']
        ratios = config['correction_ratios']
        
        # Получаем текущие значения или используем базовые
        current_y_offset = gait_param.get('init_y_offset', 0)
        current_x_offset = gait_param.get('init_x_offset', 0)
        current_roll_offset = gait_param.get('init_roll_offset', 0)
        current_pitch_offset = gait_param.get('init_pitch_offset', 0)
        
        # Применяем корректировки roll
        if config['pid']['roll']['enabled'] and abs(roll_pid_output) > 0.001:
            y_correction = roll_pid_output * ratios['roll_to_y_offset']
            roll_correction = roll_pid_output * ratios['roll_to_roll_offset']
            
            gait_param['init_y_offset'] = max(
                limits['init_y_offset']['min'],
                min(limits['init_y_offset']['max'], current_y_offset + y_correction)
            )
            gait_param['init_roll_offset'] = max(
                limits['init_roll_offset']['min'],
                min(limits['init_roll_offset']['max'], current_roll_offset + roll_correction)
            )
        
        # Применяем корректировки pitch
        if config['pid']['pitch']['enabled'] and abs(pitch_pid_output) > 0.001:
            x_correction = pitch_pid_output * ratios['pitch_to_x_offset']
            pitch_correction = pitch_pid_output * ratios['pitch_to_pitch_offset']
            
            gait_param['init_x_offset'] = max(
                limits['init_x_offset']['min'],
                min(limits['init_x_offset']['max'], current_x_offset + x_correction)
            )
            gait_param['init_pitch_offset'] = max(
                limits['init_pitch_offset']['min'],
                min(limits['init_pitch_offset']['max'], current_pitch_offset + pitch_correction)
            )
        
        return True
    
    def _calculate_pid(self, error, integral, derivative, dt, config):
        """
        Вычисляет выход PID контроллера.
        
        Args:
            error: Текущая ошибка
            integral: Интегральная составляющая
            derivative: Производная (угловая скорость)
            dt: Интервал времени
            config: Конфигурация PID (kp, ki, kd, max_output)
        
        Returns:
            float: Выход PID контроллера (ограничен max_output)
        """
        # P компонента
        p_term = config['kp'] * error
        
        # I компонента
        i_term = config['ki'] * integral
        
        # D компонента (используем производную от угловой скорости)
        d_term = config['kd'] * derivative
        
        # Суммируем и ограничиваем
        output = p_term + i_term + d_term
        return max(-config['max_output'], min(config['max_output'], output))
    
    def reset_walking_corrections(self):
        """
        Сбрасывает состояние PID регулятора стабилизации при ходьбе.
        Вызывается при остановке движения для возврата к базовым параметрам.
        """
        # Сбрасываем интегральную составляющую и состояние
        self.walking_stabilization_state['integral'] = {'roll': 0.0, 'pitch': 0.0}
        self.walking_stabilization_state['last_error'] = {'roll': 0.0, 'pitch': 0.0}
        self.walking_stabilization_state['last_time'] = None
        # Не сбрасываем last_imu_data, так как она может быть полезна при следующем движении
    
    def reset(self):
        """Сбрасывает состояние стабилизации."""
        self._reset_rest_stabilization_state()
        self.reset_walking_corrections()
    
    def set_walking_stabilization_enabled(self, enabled):
        """
        Включает/выключает стабилизацию при ходьбе (PID регулятор).
        
        Args:
            enabled: True для включения, False для выключения
        """
        self.walking_config['enabled'] = enabled
        if not enabled:
            self.reset_walking_corrections()
        rospy.loginfo(f"Walking stabilization PID: {'enabled' if enabled else 'disabled'}")
    
    def set_walking_pid_enabled(self, axis, enabled):
        """
        Включает/выключает PID стабилизацию для конкретной оси.
        
        Args:
            axis: 'roll' или 'pitch'
            enabled: True для включения, False для выключения
        """
        if axis in ['roll', 'pitch']:
            self.walking_config['pid'][axis]['enabled'] = enabled
            if not enabled:
                self.walking_stabilization_state['integral'][axis] = 0.0
            rospy.loginfo(f"Walking PID {axis}: {'enabled' if enabled else 'disabled'}")
    
    def set_walking_pid_params(self, axis, kp=None, ki=None, kd=None, max_output=None):
        """
        Устанавливает параметры PID для конкретной оси.
        
        Args:
            axis: 'roll' или 'pitch'
            kp: Пропорциональный коэффициент (опционально)
            ki: Интегральный коэффициент (опционально)
            kd: Дифференциальный коэффициент (опционально)
            max_output: Максимальный выход (опционально)
        """
        if axis not in ['roll', 'pitch']:
            return
        
        if kp is not None:
            self.walking_config['pid'][axis]['kp'] = kp
        if ki is not None:
            self.walking_config['pid'][axis]['ki'] = ki
        if kd is not None:
            self.walking_config['pid'][axis]['kd'] = kd
        if max_output is not None:
            self.walking_config['pid'][axis]['max_output'] = max_output
        
        rospy.loginfo(f"Walking PID {axis} params updated: kp={self.walking_config['pid'][axis]['kp']}, "
                     f"ki={self.walking_config['pid'][axis]['ki']}, kd={self.walking_config['pid'][axis]['kd']}, "
                     f"max_output={self.walking_config['pid'][axis]['max_output']}")
    
    def get_walking_stabilization_config(self):
        """
        Возвращает текущую конфигурацию стабилизации при ходьбе.
        
        Returns:
            dict: Копия конфигурации
        """
        return self.walking_config.copy()
