#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль автостабилизации робота через PID регулятор.
Управляет только положением корпуса (смещения туловища), без выполнения шагов.
Работает одинаково в покое и при ходьбе.
"""

import rospy
import math
from std_msgs.msg import Header
from ainex_peripherals.msg import PidConfig

from stabilization import UNIFIED_PID_CONFIG, PidController, PidDebugPublisher

# Порог движения джойстика
JOYSTICK_MOVE_THRESHOLD = 0.001


class AutoStabilization:
    """Класс для автоматической стабилизации корпуса робота через PID регулятор."""
    
    def __init__(self, gait_manager, speed_params, speed_mode, init_z_offset):
        """
        Инициализирует модуль стабилизации.
        
        Args:
            gait_manager: Менеджер походки
            speed_params: Параметры скорости
            speed_mode: Режим скорости
            init_z_offset: Начальное смещение по Z
        """
        self.gait_manager = gait_manager
        self.speed_params = speed_params
        self.speed_mode = speed_mode
        self.init_z_offset = init_z_offset
        
        # Единая конфигурация PID (используется для покоя и ходьбы)
        self.pid_config = UNIFIED_PID_CONFIG.copy()
        
        # Кэш для часто используемых значений (обновляется при изменении конфигурации)
        self._config_cache = {}
        self._update_config_cache()
        
        self.enabled = True  # Включаем модуль по умолчанию
        self.rest_enabled = True
        self.walking_enabled = True
        
        # PID контроллеры для покоя и ходьбы (используют единую конфигурацию)
        self.rest_pid_controller = PidController(self.pid_config)
        self.walking_pid_controller = PidController(self.pid_config)
        
        # Состояние для покоя
        self.rest_state = {
            'reference': None,  # Эталонная ориентация {'roll': x, 'pitch': y, 'yaw': z}
            'reference_set': False,  # Установлен ли эталон
            'reference_set_time': None,  # Время установки эталона
            'last_imu_data': None,  # Последние данные IMU
        }
        
        # Состояние для ходьбы
        self.walking_state = {
            'last_imu_data': None,  # Последние данные IMU
        }
        
        # ROS топики для отладки и настройки PID
        self.pid_debug_pub = PidDebugPublisher()
        self.pid_config_sub = rospy.Subscriber('/stabilization/pid_config', PidConfig, self._pid_config_callback)
        
        # Ограничение частоты публикации отладочных данных (для производительности)
        self.debug_pub_last_time = 0
        self.debug_pub_interval = 0.05  # 20 Hz максимум
        
        # Инициализация эталона для покоя
        self._init_reference()
        
        rospy.loginfo("AutoStabilization module initialized (PID-only)")
        rospy.loginfo("  Unified PID configuration enabled")
        rospy.loginfo("  Debug topic: /stabilization/pid_debug")
        rospy.loginfo("  Config topic: /stabilization/pid_config")
    
    def _update_config_cache(self):
        """Обновляет кэш часто используемых значений конфигурации."""
        self._config_cache = {
            'roll_threshold': self.pid_config.get('roll_activation_threshold', 2.0),
            'pitch_threshold': self.pid_config.get('pitch_activation_threshold', 2.0),
            'y_min': self.pid_config.get('limit_y_offset_min', -0.03),
            'y_max': self.pid_config.get('limit_y_offset_max', 0.03),
            'x_min': self.pid_config.get('limit_x_offset_min', -0.03),
            'x_max': self.pid_config.get('limit_x_offset_max', 0.03),
            'roll_min': self.pid_config.get('limit_roll_offset_min', -5.0),
            'roll_max': self.pid_config.get('limit_roll_offset_max', 5.0),
            'pitch_min': self.pid_config.get('limit_pitch_offset_min', -5.0),
            'pitch_max': self.pid_config.get('limit_pitch_offset_max', 5.0),
            'roll_to_y_ratio': self.pid_config.get('roll_to_y_offset_ratio', 1.0),
            'roll_to_roll_ratio': self.pid_config.get('roll_to_roll_offset_ratio', 0.5),
            'pitch_to_x_ratio': self.pid_config.get('pitch_to_x_offset_ratio', 1.0),
            'pitch_to_pitch_ratio': self.pid_config.get('pitch_to_pitch_offset_ratio', 0.5),
            'update_interval': self.pid_config.get('update_interval', 0.05),
        }
    
    def _init_reference(self):
        """Инициализирует эталонную ориентацию для покоя."""
        # Используем абсолютные углы (0, 90, 0) как эталон
        self.rest_state['reference'] = {'roll': 0.0, 'pitch': 90.0, 'yaw': 0.0}
        self.rest_state['reference_set'] = True
        rospy.loginfo("📐 Reference orientation: absolute angles (roll=0°, pitch=90°, yaw=0°)")
    
    def _apply_base_offsets(self):
        """
        Применяет базовые смещения из конфигурации к текущей позе робота.
        Вызывается автоматически при обновлении базовых смещений через топик.
        """
        try:
            if self.gait_manager is None:
                rospy.logwarn("⚠️ Cannot apply base offsets: gait_manager is None")
                return
            
            gait_param = self.gait_manager.get_gait_param()
            
            # Получаем базовые смещения из конфигурации
            base_x = self.pid_config.get('base_x_offset', 0.0)
            base_y = self.pid_config.get('base_y_offset', 0.0)
            base_roll = self.pid_config.get('base_roll_offset', 0.0)
            base_pitch = self.pid_config.get('base_pitch_offset', 0.0)
            
            # Сохраняем старые значения для логирования
            old_x = gait_param.get('init_x_offset', 0.0)
            old_y = gait_param.get('init_y_offset', 0.0)
            old_roll = gait_param.get('init_roll_offset', 0.0)
            old_pitch = gait_param.get('init_pitch_offset', 0.0)
            
            # Применяем базовые смещения к позе
            gait_param['init_x_offset'] = base_x
            gait_param['init_y_offset'] = base_y
            gait_param['init_roll_offset'] = base_roll
            gait_param['init_pitch_offset'] = base_pitch
            
            # Получаем параметры периода из текущего режима скорости
            speed_param = self.speed_params.get(self.speed_mode, {})
            period_time = speed_param.get('period_time', [400, 0.2, 0.02])
            
            # Применяем изменения через update_param (без движения)
            self.gait_manager.update_param(
                period_time,
                0, 0, 0,  # Без движения
                gait_param,
                step_num=0
            )
            
            rospy.loginfo(f"✅ Base offsets applied:")
            rospy.loginfo(f"   X: {old_x:.4f} → {base_x:.4f} m")
            rospy.loginfo(f"   Y: {old_y:.4f} → {base_y:.4f} m")
            rospy.loginfo(f"   Roll: {old_roll:.2f} → {base_roll:.2f}°")
            rospy.loginfo(f"   Pitch: {old_pitch:.2f} → {base_pitch:.2f}°")
        except Exception as e:
            rospy.logerr(f"❌ Error applying base offsets: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def _pid_config_callback(self, msg):
        """
        Обработчик сообщения конфигурации PID из топика.
        Обновляет единую конфигурацию PID в реальном времени.
        Оптимизированная версия с упрощенной логикой обновления.
        """
        try:
            updates = {}
            EPSILON = 1e-6  # Порог для сравнения float значений
            
            # Список всех параметров для обновления (имя_поля, ключ_конфига, значение_по_умолчанию, проверка_на_ноль)
            # проверка_на_ноль: True = пропускать нулевые значения, False = обновлять всегда
            param_list = [
                # Bool параметры
                ('pid_enabled', 'pid_enabled', True, False),
                ('roll_enabled', 'roll_enabled', True, False),
                ('pitch_enabled', 'pitch_enabled', True, False),
                ('filter_enabled', 'filter_enabled', True, False),
                # Roll коэффициенты
                ('roll_kp', 'roll_kp', 0.0, True),
                ('roll_ki', 'roll_ki', 0.0, True),
                ('roll_kd', 'roll_kd', 0.0, True),
                ('roll_max_output', 'roll_max_output', 0.0, True),
                ('roll_target', 'roll_target', 0.0, False),
                ('roll_integral_limit', 'roll_integral_limit', 10.0, True),
                ('roll_activation_threshold', 'roll_activation_threshold', 2.0, True),
                # Pitch коэффициенты
                ('pitch_kp', 'pitch_kp', 0.0, True),
                ('pitch_ki', 'pitch_ki', 0.0, True),
                ('pitch_kd', 'pitch_kd', 0.0, True),
                ('pitch_max_output', 'pitch_max_output', 0.0, True),
                ('pitch_target', 'pitch_target', 90.0, False),
                ('pitch_integral_limit', 'pitch_integral_limit', 10.0, True),
                ('pitch_activation_threshold', 'pitch_activation_threshold', 2.0, True),
                # Базовые смещения
                ('base_x_offset', 'base_x_offset', 0.0, False),
                ('base_y_offset', 'base_y_offset', 0.0, False),
                ('base_roll_offset', 'base_roll_offset', 0.0, False),
                ('base_pitch_offset', 'base_pitch_offset', 0.0, False),
                # Пределы смещений
                ('limit_x_offset_min', 'limit_x_offset_min', -0.03, False),
                ('limit_x_offset_max', 'limit_x_offset_max', 0.03, False),
                ('limit_y_offset_min', 'limit_y_offset_min', -0.03, False),
                ('limit_y_offset_max', 'limit_y_offset_max', 0.03, False),
                ('limit_roll_offset_min', 'limit_roll_offset_min', -5.0, False),
                ('limit_roll_offset_max', 'limit_roll_offset_max', 5.0, False),
                ('limit_pitch_offset_min', 'limit_pitch_offset_min', -5.0, False),
                ('limit_pitch_offset_max', 'limit_pitch_offset_max', 5.0, False),
                # Коэффициенты преобразования
                ('roll_to_y_offset_ratio', 'roll_to_y_offset_ratio', 1.0, True),
                ('roll_to_roll_offset_ratio', 'roll_to_roll_offset_ratio', 0.5, True),
                ('pitch_to_x_offset_ratio', 'pitch_to_x_offset_ratio', 1.0, True),
                ('pitch_to_pitch_offset_ratio', 'pitch_to_pitch_offset_ratio', 0.5, True),
                # Настройки фильтра
                ('filter_alpha', 'filter_alpha', 0.7, True),
                ('update_interval', 'update_interval', 0.05, True),
            ]
            
            # Обновляем параметры через цикл (более эффективно)
            for msg_attr, config_key, default_val, skip_zero in param_list:
                if not hasattr(msg, msg_attr):
                    continue
                    
                msg_val = getattr(msg, msg_attr)
                current_val = self.pid_config.get(config_key, default_val)
                
                # Для bool параметров - простое сравнение
                if isinstance(msg_val, bool):
                    if msg_val != current_val:
                        updates[config_key] = msg_val
                # Для float параметров
                elif isinstance(msg_val, (int, float)):
                    # Для базовых смещений всегда обновляем (даже если значение равно текущему)
                    # Это позволяет применять смещения даже если они уже установлены
                    if config_key.startswith('base_'):
                        updates[config_key] = float(msg_val)
                    # Для остальных параметров: пропускаем нулевые значения, если требуется
                    elif skip_zero and abs(msg_val) < EPSILON:
                        continue
                    # Обновляем, если значение отличается
                    elif abs(msg_val - current_val) > EPSILON:
                        updates[config_key] = float(msg_val)
            
            # Применяем обновления только если есть изменения
            if updates:
                self.pid_config.update(updates)
                self.rest_pid_controller.update_config(updates)
                self.walking_pid_controller.update_config(updates)
                # Обновляем кэш
                self._update_config_cache()
                rospy.loginfo(f"🔧 PID config updated: {len(updates)} parameters changed")
                if rospy.get_param('~debug', False):
                    rospy.loginfo(f"   Updated parameters: {list(updates.keys())}")
                
                # Если обновлены базовые смещения, применяем их к позе робота
                base_offset_keys = ['base_x_offset', 'base_y_offset', 'base_roll_offset', 'base_pitch_offset']
                updated_base_offsets = [key for key in base_offset_keys if key in updates]
                if updated_base_offsets:
                    rospy.loginfo(f"📍 Applying base offsets: {updated_base_offsets}")
                    self._apply_base_offsets()
            
            # Сбрасываем интегральные составляющие, если запрошено
            if msg.reset_integral:
                self.rest_pid_controller.reset_integral()
                self.walking_pid_controller.reset_integral()
                rospy.loginfo("🔧 PID integral components reset")
                
        except Exception as e:
            rospy.logwarn(f"Error updating PID config: {e}")
    
    def process(self, imu_data, robot_state, status, x_move_amp, y_move_amp, angle_move_amp):
        """
        Обрабатывает данные IMU и применяет PID стабилизацию.
        
        Args:
            imu_data: Словарь с данными IMU (orientation, angular_velocity, linear_acceleration)
            robot_state: Состояние робота ('stand', 'lie_to_stand', 'recline_to_stand')
            status: Статус движения ('move' или 'stop')
            x_move_amp: Амплитуда движения по X (от джойстика)
            y_move_amp: Амплитуда движения по Y (от джойстика)
            angle_move_amp: Амплитуда поворота (от джойстика)
        
        Returns:
            bool: False (не выполняем шаги)
        """
        if not self.enabled or imu_data is None:
            return False
        
        # Определяем режим стабилизации
        if status == 'stop' and self.rest_enabled:
            self._process_rest_stabilization(imu_data, robot_state, x_move_amp, y_move_amp, angle_move_amp)
        elif status == 'move' and self.walking_enabled:
            # Сохраняем данные IMU для использования в apply_walking_corrections
            self.walking_state['last_imu_data'] = imu_data
        
        return False  # Не выполняем шаги, только управляем корпусом
    
    def _process_rest_stabilization(self, imu_data, robot_state, x_move_amp, y_move_amp, angle_move_amp):
        """
        Применяет PID стабилизацию в покое.
        
        Args:
            imu_data: Словарь с данными IMU
            robot_state: Состояние робота
            x_move_amp: Амплитуда движения по X (от джойстика)
            y_move_amp: Амплитуда движения по Y (от джойстика)
            angle_move_amp: Амплитуда поворота (от джойстика)
        """
        # Проверяем, включен ли PID
        if not self.pid_config.get('pid_enabled', False):
            return
        
        # Проверяем состояние робота
        if robot_state != 'stand':
            return
        
        # Проверяем, нет ли движения от джойстика
        joystick_moving = (abs(x_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(y_move_amp) > JOYSTICK_MOVE_THRESHOLD or 
                          abs(angle_move_amp) > JOYSTICK_MOVE_THRESHOLD)
        if joystick_moving:
            return
        
        # Применяем PID корректировки
        self._apply_rest_corrections(imu_data)
    
    def _apply_rest_corrections(self, imu_data):
        """
        Применяет PID-корректировки для удержания вертикального положения туловища в покое.
        Корректирует смещения туловища через gait_manager.update_param() без выполнения шагов.
        
        Args:
            imu_data: Словарь с данными IMU
        """
        current_time = rospy.get_time()
        pid_state = self.rest_pid_controller.get_state()
        
        # Проверяем интервал обновления
        last_time = pid_state.get('last_time')
        if last_time is not None:
            update_interval = self._config_cache.get('update_interval', 0.05)
            if current_time - last_time < update_interval:
                return  # Слишком рано для обновления
        
        orientation = imu_data.get('orientation', {})
        angular_velocity = imu_data.get('angular_velocity', {})
        
        if not orientation:
            return
            
        roll_deg = orientation.get('roll_deg', 0)
        pitch_deg = orientation.get('pitch_deg', 0)
        roll_vel = angular_velocity.get('x', 0) * 180.0 / math.pi  # рад/с -> град/с
        pitch_vel = angular_velocity.get('y', 0) * 180.0 / math.pi
        
        # Вычисляем PID выходы
        pid_outputs = self.rest_pid_controller.calculate(
            roll_deg, pitch_deg, roll_vel, pitch_vel, current_time
        )
        
        roll_result = pid_outputs['roll']
        pitch_result = pid_outputs['pitch']
        
        # Проверяем пороги активации (используем кэш)
        roll_threshold = self._config_cache.get('roll_threshold', 2.0)
        pitch_threshold = self._config_cache.get('pitch_threshold', 2.0)
        
        if abs(roll_result['error']) < roll_threshold and abs(pitch_result['error']) < pitch_threshold:
            # Отклонения малы, сбрасываем интегральную составляющую
            self.rest_pid_controller.reset_integral()
            return
        
        # Применяем корректировки к gait_param
        # ВАЖНО: Получаем gait_param и сохраняем текущие значения, чтобы не перезаписать изменения от других модулей
        gait_param = self.gait_manager.get_gait_param()
        current_y_offset = gait_param.get('init_y_offset', 0)
        current_x_offset = gait_param.get('init_x_offset', 0)
        current_roll_offset = gait_param.get('init_roll_offset', 0)
        current_pitch_offset = gait_param.get('init_pitch_offset', 0)
        
        # Пределы и коэффициенты (используем кэш)
        y_min = self._config_cache.get('y_min', -0.03)
        y_max = self._config_cache.get('y_max', 0.03)
        x_min = self._config_cache.get('x_min', -0.03)
        x_max = self._config_cache.get('x_max', 0.03)
        roll_min = self._config_cache.get('roll_min', -5.0)
        roll_max = self._config_cache.get('roll_max', 5.0)
        pitch_min = self._config_cache.get('pitch_min', -5.0)
        pitch_max = self._config_cache.get('pitch_max', 5.0)
        
        roll_to_y_ratio = self._config_cache.get('roll_to_y_ratio', 1.0)
        roll_to_roll_ratio = self._config_cache.get('roll_to_roll_ratio', 0.5)
        pitch_to_x_ratio = self._config_cache.get('pitch_to_x_ratio', 1.0)
        pitch_to_pitch_ratio = self._config_cache.get('pitch_to_pitch_ratio', 0.5)
        
        # Применяем корректировки
        applied_y_correction = 0.0
        applied_roll_correction = 0.0
        applied_x_correction = 0.0
        applied_pitch_correction = 0.0
        
        if self.pid_config.get('roll_enabled', False) and abs(roll_result['output']) > 0.001:
            y_correction = roll_result['output'] * roll_to_y_ratio
            roll_correction = roll_result['output'] * roll_to_roll_ratio
            
            new_y_offset = max(y_min, min(y_max, current_y_offset + y_correction))
            new_roll_offset = max(roll_min, min(roll_max, current_roll_offset + roll_correction))
            
            applied_y_correction = new_y_offset - current_y_offset
            applied_roll_correction = new_roll_offset - current_roll_offset
            
            gait_param['init_y_offset'] = new_y_offset
            gait_param['init_roll_offset'] = new_roll_offset
        
        if self.pid_config.get('pitch_enabled', False) and abs(pitch_result['output']) > 0.001:
            x_correction = pitch_result['output'] * pitch_to_x_ratio
            pitch_correction = pitch_result['output'] * pitch_to_pitch_ratio
            
            new_x_offset = max(x_min, min(x_max, current_x_offset + x_correction))
            new_pitch_offset = max(pitch_min, min(pitch_max, current_pitch_offset + pitch_correction))
            
            applied_x_correction = new_x_offset - current_x_offset
            applied_pitch_correction = new_pitch_offset - current_pitch_offset
            
            gait_param['init_x_offset'] = new_x_offset
            gait_param['init_pitch_offset'] = new_pitch_offset
        
        # Применяем корректировки через update_param (без движения)
        if applied_y_correction != 0 or applied_roll_correction != 0 or \
           applied_x_correction != 0 or applied_pitch_correction != 0:
            try:
                # Используем параметры из текущего режима скорости
                speed_param = self.speed_params.get(self.speed_mode, {})
                period_time = speed_param.get('period_time', [400, 0.2, 0.02])
                
                self.gait_manager.update_param(
                period_time,
                    0, 0, 0,  # Без движения
                gait_param,
                    step_num=0
                )
            except Exception as e:
                rospy.logwarn(f"Error applying rest PID corrections: {e}")
        
        # Публикуем отладочные данные (с ограничением частоты)
        current_time = rospy.get_time()
        if current_time - self.debug_pub_last_time >= self.debug_pub_interval:
            applied_corrections = {
                'x': applied_x_correction,
                'y': applied_y_correction,
                'roll': applied_roll_correction,
                'pitch': applied_pitch_correction,
                'current_x': gait_param.get('init_x_offset', 0),
                'current_y': gait_param.get('init_y_offset', 0),
                'current_roll': gait_param.get('init_roll_offset', 0),
                'current_pitch': gait_param.get('init_pitch_offset', 0),
            }
            self.pid_debug_pub.publish(
                imu_data, is_walking=False,
                pid_state=self.rest_pid_controller.get_state(),
                pid_outputs=pid_outputs,
                applied_corrections=applied_corrections,
                config=self.pid_config
            )
            self.debug_pub_last_time = current_time
    
    def apply_walking_corrections(self, gait_param, period_time):
        """
        Применяет PID-корректировки для удержания вертикального положения туловища при ходьбе.
        Использует единую конфигурацию PID.
        Меняет только смещения туловища (init_x_offset, init_y_offset, init_roll_offset, init_pitch_offset),
        НЕ трогая базовые параметры походки.
        
        Args:
            gait_param: Словарь параметров походки (будет изменен)
            period_time: Список [period_time, dsp_ratio, y_swap_amplitude] (не изменяется)
        
        Returns:
            bool: True если были применены корректировки
        """
        # Проверяем, включен ли PID
        if not self.pid_config.get('pid_enabled', False):
            return False
        
        # Получаем последние данные IMU
        imu_data = self.walking_state.get('last_imu_data')
        if not imu_data:
            return False
        
        orientation = imu_data.get('orientation', {})
        angular_velocity = imu_data.get('angular_velocity', {})
        
        if not orientation:
            return False
        
        roll_deg = orientation.get('roll_deg', 0)
        pitch_deg = orientation.get('pitch_deg', 0)
        roll_vel = angular_velocity.get('x', 0) * 180.0 / math.pi  # рад/с -> град/с
        pitch_vel = angular_velocity.get('y', 0) * 180.0 / math.pi
        
        current_time = rospy.get_time()
        
        # Вычисляем PID выходы
        pid_outputs = self.walking_pid_controller.calculate(
            roll_deg, pitch_deg, roll_vel, pitch_vel, current_time
        )
        
        roll_result = pid_outputs['roll']
        pitch_result = pid_outputs['pitch']
        
        # Проверяем пороги активации (используем кэш)
        roll_threshold = self._config_cache.get('roll_threshold', 2.0)
        pitch_threshold = self._config_cache.get('pitch_threshold', 2.0)
        
        if abs(roll_result['error']) < roll_threshold and abs(pitch_result['error']) < pitch_threshold:
            # Отклонения малы, сбрасываем интегральную составляющую
            self.walking_pid_controller.reset_integral()
            return False
        
        # Получаем текущие значения из gait_param (уже переданного, не перезаписываем)
        current_y_offset = gait_param.get('init_y_offset', 0)
        current_x_offset = gait_param.get('init_x_offset', 0)
        current_roll_offset = gait_param.get('init_roll_offset', 0)
        current_pitch_offset = gait_param.get('init_pitch_offset', 0)
        
        # Пределы и коэффициенты (используем кэш)
        y_min = self._config_cache.get('y_min', -0.03)
        y_max = self._config_cache.get('y_max', 0.03)
        x_min = self._config_cache.get('x_min', -0.03)
        x_max = self._config_cache.get('x_max', 0.03)
        roll_min = self._config_cache.get('roll_min', -5.0)
        roll_max = self._config_cache.get('roll_max', 5.0)
        pitch_min = self._config_cache.get('pitch_min', -5.0)
        pitch_max = self._config_cache.get('pitch_max', 5.0)
        
        roll_to_y_ratio = self._config_cache.get('roll_to_y_ratio', 1.0)
        roll_to_roll_ratio = self._config_cache.get('roll_to_roll_ratio', 0.5)
        pitch_to_x_ratio = self._config_cache.get('pitch_to_x_ratio', 1.0)
        pitch_to_pitch_ratio = self._config_cache.get('pitch_to_pitch_ratio', 0.5)
        
        # Применяемые корректировки
        applied_y_correction = 0.0
        applied_roll_correction = 0.0
        applied_x_correction = 0.0
        applied_pitch_correction = 0.0
        
        # Применяем корректировки roll
        if self.pid_config.get('roll_enabled', False) and abs(roll_result['output']) > 0.001:
            y_correction = roll_result['output'] * roll_to_y_ratio
            roll_correction = roll_result['output'] * roll_to_roll_ratio
            
            new_y_offset = max(y_min, min(y_max, current_y_offset + y_correction))
            new_roll_offset = max(roll_min, min(roll_max, current_roll_offset + roll_correction))
            
            applied_y_correction = new_y_offset - current_y_offset
            applied_roll_correction = new_roll_offset - current_roll_offset
            
            gait_param['init_y_offset'] = new_y_offset
            gait_param['init_roll_offset'] = new_roll_offset
        
        # Применяем корректировки pitch
        if self.pid_config.get('pitch_enabled', False) and abs(pitch_result['output']) > 0.001:
            x_correction = pitch_result['output'] * pitch_to_x_ratio
            pitch_correction = pitch_result['output'] * pitch_to_pitch_ratio
            
            new_x_offset = max(x_min, min(x_max, current_x_offset + x_correction))
            new_pitch_offset = max(pitch_min, min(pitch_max, current_pitch_offset + pitch_correction))
            
            applied_x_correction = new_x_offset - current_x_offset
            applied_pitch_correction = new_pitch_offset - current_pitch_offset
            
            gait_param['init_x_offset'] = new_x_offset
            gait_param['init_pitch_offset'] = new_pitch_offset
        
        # Публикуем отладочные данные (с ограничением частоты)
        current_time = rospy.get_time()
        if current_time - self.debug_pub_last_time >= self.debug_pub_interval:
            applied_corrections = {
                'x': applied_x_correction,
                'y': applied_y_correction,
                'roll': applied_roll_correction,
                'pitch': applied_pitch_correction,
                'current_x': gait_param.get('init_x_offset', 0),
                'current_y': gait_param.get('init_y_offset', 0),
                'current_roll': gait_param.get('init_roll_offset', 0),
                'current_pitch': gait_param.get('init_pitch_offset', 0),
            }
            self.pid_debug_pub.publish(
                imu_data, is_walking=True,
                pid_state=self.walking_pid_controller.get_state(),
                pid_outputs=pid_outputs,
                applied_corrections=applied_corrections,
                config=self.pid_config
            )
            self.debug_pub_last_time = current_time
        
        return True
    
    def reset_walking_corrections(self):
        """Сбрасывает состояние PID регулятора стабилизации при ходьбе."""
        self.walking_pid_controller.reset_integral()
    
    def reset(self):
        """Сбрасывает состояние стабилизации."""
        self.rest_pid_controller.reset_integral()
        self.walking_pid_controller.reset_integral()
    
    def set_walking_stabilization_enabled(self, enabled):
        """Включает/выключает стабилизацию при ходьбе."""
        self.walking_enabled = enabled
        rospy.loginfo(f"Walking stabilization: {'ENABLED' if enabled else 'DISABLED'}")
    
    def set_rest_stabilization_enabled(self, enabled):
        """Включает/выключает стабилизацию в покое."""
        self.rest_enabled = enabled
        rospy.loginfo(f"Rest stabilization: {'ENABLED' if enabled else 'DISABLED'}")
