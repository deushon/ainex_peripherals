#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль обработки данных IMU: обнаружение падений через ориентацию.
"""

# ========== КОНФИГУРАЦИЯ ОБНАРУЖЕНИЯ ПАДЕНИЙ ==========
FALL_DETECTION_CONFIG = {
    # Критические углы по каждой оси (в градусах)
    'critical_angle_roll': 45.0,   # Критический угол крена (наклон влево/вправо)
    'critical_angle_pitch': 45.0,  # Критический угол тангажа (наклон вперед/назад)
    
    # Время, в течение которого робот должен быть под критическим углом (сек)
    'critical_angle_duration': 0.5,
    
    # Опциональная проверка покоя (изменения ускорений ниже порога)
    'check_rest_enabled': True,   # Включить проверку покоя
    'rest_accel_threshold': 0.5,  # Порог изменения ускорения для определения покоя (м/с²)
    'rest_check_duration': 0.3,   # Время для проверки покоя (сек)
}

# Автоматический подъем
AUTO_GETUP_ENABLED = True  # Включен ли автоматический подъем
FALL_TIME_THRESHOLD = 3.0  # Время падения для автоматического подъема (сек)
AUTO_GETUP_INTERVAL = 5.0  # Минимальный интервал между попытками подъема (сек)
MAX_GETUP_ATTEMPTS = 2  # Максимальное количество попыток подъема
GETUP_ACTION_TIMEOUT = 10.0  # Таймаут выполнения действия подъема (сек)
FALL_CHECK_COOLDOWN_DURATION = 10.0  # Длительность cooldown после подъема (сек)

# ===================================

import rospy
import math
import threading
from sensor_msgs.msg import Imu


class IMUHandler:
    """
    Класс для обработки данных IMU: обнаружение падений через ориентацию.
    """
    
    def __init__(self):
        """Инициализация обработчика IMU."""
        self.fall_config = FALL_DETECTION_CONFIG.copy()
        
        # Состояние робота
        self.robot_state = 'stand'
        self.fall_check_cooldown = 0
        
        # История ориентации для определения падения
        self.orientation_history = {
            'roll': [],
            'pitch': []
        }
        
        # История ускорений для проверки покоя
        self.accel_history = {
            'x': [],
            'y': [],
            'z': []
        }
        
        # Время начала критического угла
        self.critical_angle_start_time = None
        self.critical_angle_axis = None
        
        # Параметры для автоматического подъема
        self.auto_getup_enabled = AUTO_GETUP_ENABLED
        self.fall_time_threshold = FALL_TIME_THRESHOLD
        self.fall_start_time = None
        self.last_auto_getup_time = 0
        self.auto_getup_interval = AUTO_GETUP_INTERVAL
        self.getup_action_in_progress = False
        self.getup_attempt_count = 0
        self.max_getup_attempts = MAX_GETUP_ATTEMPTS
        
        # Блокировка для thread-safe доступа
        self.imu_lock = threading.Lock()
        
        # Логирование
        self.imu_data_received = False
        
        # Хранение RAW данных IMU для вывода при падении
        self.last_raw_data = None
        self.last_processed_data = None
        self.fall_data_logged = False  # Флаг для однократного вывода данных при падении
        
        rospy.loginfo("IMUHandler module initialized")
        rospy.loginfo(f"  Fall detection config: {self.fall_config}")
    
    def process_imu(self, msg: Imu):
        """
        Обрабатывает сообщение IMU.
        
        Args:
            msg: Сообщение IMU
        
        Returns:
            dict: Словарь с обработанными данными и состоянием
        """
        try:
            with self.imu_lock:
                # Получаем сырые данные из IMU
                qx_raw = msg.orientation.x
                qy_raw = msg.orientation.y
                qz_raw = msg.orientation.z
                qw_raw = msg.orientation.w
                
                # Используем quaternion как есть
                qx, qy, qz, qw = qx_raw, qy_raw, qz_raw, qw_raw
                
                # Преобразование quaternion в углы Эйлера (в радианах)
                sinr_cosp = 2 * (qw * qx + qy * qz)
                cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                roll_raw = math.atan2(sinr_cosp, cosr_cosp)
                
                sinp = 2 * (qw * qy - qz * qx)
                if abs(sinp) >= 1:
                    pitch_raw = math.copysign(math.pi / 2, sinp)
                else:
                    pitch_raw = math.asin(sinp)
                
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                # МЕНЯЕМ МЕСТАМИ ROLL И PITCH (так как X и Y переставлены)
                roll = pitch_raw  # roll теперь из pitch
                pitch = roll_raw  # pitch теперь из roll
                
                # Конвертируем в градусы для удобства
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)
                
                # Получаем угловую скорость
                gx_raw = msg.angular_velocity.x
                gy_raw = msg.angular_velocity.y
                gz_raw = msg.angular_velocity.z
                
                # МЕНЯЕМ МЕСТАМИ X И Y для угловой скорости
                gx = gy_raw  # gx теперь из gy
                gy = gx_raw  # gy теперь из gx
                gz = gz_raw  # gz остается как есть
                
                # Получаем линейное ускорение
                ax_raw = msg.linear_acceleration.x
                ay_raw = msg.linear_acceleration.y
                az_raw = msg.linear_acceleration.z
                
                # МЕНЯЕМ МЕСТАМИ X И Y для линейного ускорения
                ax = ay_raw  # ax теперь из ay
                ay = ax_raw  # ay теперь из ax
                az = az_raw  # az остается как есть
                
                # Сохраняем RAW данные для вывода при падении
                self.last_raw_data = {
                    'orientation': {
                        'x': qx_raw,
                        'y': qy_raw,
                        'z': qz_raw,
                        'w': qw_raw
                    },
                    'angular_velocity': {
                        'x': gx_raw,
                        'y': gy_raw,
                        'z': gz_raw
                    },
                    'linear_acceleration': {
                        'x': ax_raw,
                        'y': ay_raw,
                        'z': az_raw
                    }
                }
                
                # Сохраняем обработанные данные для вывода при падении
                self.last_processed_data = {
                    'orientation': {
                        'x': qx,
                        'y': qy,
                        'z': qz,
                        'w': qw,
                        'roll': roll,
                        'pitch': pitch,
                        'yaw': yaw,
                        'roll_deg': roll_deg,
                        'pitch_deg': pitch_deg,
                        'yaw_deg': yaw_deg
                    },
                    'angular_velocity': {
                        'x': gx,
                        'y': gy,
                        'z': gz
                    },
                    'linear_acceleration': {
                        'x': ax,
                        'y': ay,
                        'z': az
                    }
                }
                
                # Логирование первого получения IMU данных
                if not self.imu_data_received:
                    self.imu_data_received = True
                    rospy.loginfo("✅ IMU data received!")
            
            # Обработка падения через ориентацию
            current_time = rospy.get_time()
            self._update_fall_detection(roll_deg, pitch_deg, ax, ay, az, current_time)
            
            # Возвращаем обработанные данные
            return {
                'orientation': {
                    'x': qx,
                    'y': qy,
                    'z': qz,
                    'w': qw,
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'roll_deg': roll_deg,
                    'pitch_deg': pitch_deg,
                    'yaw_deg': yaw_deg,
                },
                'angular_velocity': {
                    'x': gx,
                    'y': gy,
                    'z': gz,
                },
                'linear_acceleration': {
                    'x': ax,
                    'y': ay,
                    'z': az,
                },
                'robot_state': self.robot_state,
                'fall_detected': self.robot_state != 'stand'
            }
            
        except Exception as e:
            rospy.logwarn(f"Error processing IMU data: {e}")
            return None
    
    def _update_fall_detection(self, roll_deg, pitch_deg, ax, ay, az, current_time):
        """
        Обновляет логику обнаружения падения на основе ориентации.
        
        Args:
            roll_deg: Угол крена в градусах
            pitch_deg: Угол тангажа в градусах
            ax, ay, az: Линейные ускорения
            current_time: Текущее время
        """
        # Проверяем cooldown
        if current_time < self.fall_check_cooldown:
            return
        
        # Добавляем текущие значения в историю
        self.orientation_history['roll'].append(roll_deg)
        self.orientation_history['pitch'].append(pitch_deg)
        self.accel_history['x'].append(ax)
        self.accel_history['y'].append(ay)
        self.accel_history['z'].append(az)
        
        # Ограничиваем размер истории
        max_history_size = 100
        for key in self.orientation_history:
            if len(self.orientation_history[key]) > max_history_size:
                self.orientation_history[key].pop(0)
        for key in self.accel_history:
            if len(self.accel_history[key]) > max_history_size:
                self.accel_history[key].pop(0)
        
        # Проверяем критические углы (yaw не проверяем)
        critical_roll = self.fall_config['critical_angle_roll']
        critical_pitch = self.fall_config['critical_angle_pitch']
        
        # Roll: нормальное положение около 0°, падение при отклонении
        roll_exceeded = abs(roll_deg) > critical_roll
        
        # Pitch: нормальное положение около 90°, падение при отклонении к 0° или 180°
        # Проверяем отклонение от 90°
        pitch_deviation_from_90 = abs(pitch_deg - 90.0)
        # Если отклонение больше критического угла, значит pitch вышел за допустимые пределы
        pitch_exceeded = pitch_deviation_from_90 > critical_pitch
        
        critical_angle_exceeded = roll_exceeded or pitch_exceeded
        
        # Определяем, какая ось превышена
        exceeded_axis = None
        if roll_exceeded:
            exceeded_axis = 'roll'
        elif pitch_exceeded:
            exceeded_axis = 'pitch'
        
        # Проверяем условие покоя (если включено)
        is_at_rest = True
        if self.fall_config['check_rest_enabled']:
            is_at_rest = self._check_robot_at_rest()
        
        # Обновляем время начала критического угла
        if critical_angle_exceeded and is_at_rest:
            if self.critical_angle_start_time is None:
                self.critical_angle_start_time = current_time
                self.critical_angle_axis = exceeded_axis
            elif self.critical_angle_axis != exceeded_axis:
                # Изменилась ось критического угла - сбрасываем таймер
                self.critical_angle_start_time = current_time
                self.critical_angle_axis = exceeded_axis
        else:
            # Угол не критичен или робот не в покое - сбрасываем таймер
            self.critical_angle_start_time = None
            self.critical_angle_axis = None
        
        # Проверяем, прошло ли достаточно времени под критическим углом
        old_state = self.robot_state
        if (critical_angle_exceeded and 
            is_at_rest and 
            self.critical_angle_start_time is not None and
            (current_time - self.critical_angle_start_time) >= self.fall_config['critical_angle_duration']):
            
            # Определяем тип падения на основе оси и направления
            if exceeded_axis == 'roll':
                if roll_deg > 0:
                    self.robot_state = 'fall_left'   # Упал влево
                else:
                    self.robot_state = 'fall_right'  # Упал вправо
            elif exceeded_axis == 'pitch':
                # Pitch: нормальное положение около 90°
                # Падение вперед: pitch близок к 0° (или 360°)
                # Падение назад: pitch близок к 180°
                # Нормализуем угол к диапазону [0, 360)
                pitch_normalized = pitch_deg % 360
                if pitch_normalized < 0:
                    pitch_normalized += 360
                
                # Определяем направление падения:
                # Если pitch в диапазоне [0, 90) или (270, 360), то ближе к 0° → fall_forward
                # Если pitch в диапазоне (90, 270], то ближе к 180° → fall_backward
                if (pitch_normalized >= 0 and pitch_normalized < 90) or (pitch_normalized > 270 and pitch_normalized < 360):
                    self.robot_state = 'fall_forward'   # Упал вперед (ближе к 0°)
                else:
                    self.robot_state = 'fall_backward'  # Упал назад (ближе к 180°)
        else:
            # Робот не под критическим углом достаточно долго
            self.robot_state = 'stand'
        
        # Логируем изменение состояния
        if old_state != self.robot_state:
            rospy.loginfo(f"🔄 IMU detected robot state change: '{old_state}' -> '{self.robot_state}'")
            if self.robot_state != 'stand' and old_state == 'stand':
                if self.fall_start_time is None:
                    self.fall_start_time = current_time
                    rospy.logwarn(f"⚠️ Robot fell! State: {self.robot_state}, Axis: {exceeded_axis}")
                    
                    # Выводим RAW данные IMU и рассчитанные значения один раз при детекции падения
                    if self.last_raw_data is not None and self.last_processed_data is not None:
                        rospy.logwarn("=" * 80)
                        rospy.logwarn("📊 IMU DATA AT FALL DETECTION:")
                        rospy.logwarn("=" * 80)
                        rospy.logwarn("RAW IMU DATA:")
                        rospy.logwarn(f"  Orientation (quaternion):")
                        rospy.logwarn(f"    x: {self.last_raw_data['orientation']['x']:.6f}")
                        rospy.logwarn(f"    y: {self.last_raw_data['orientation']['y']:.6f}")
                        rospy.logwarn(f"    z: {self.last_raw_data['orientation']['z']:.6f}")
                        rospy.logwarn(f"    w: {self.last_raw_data['orientation']['w']:.6f}")
                        rospy.logwarn(f"  Angular velocity (rad/s):")
                        rospy.logwarn(f"    x: {self.last_raw_data['angular_velocity']['x']:.6f}")
                        rospy.logwarn(f"    y: {self.last_raw_data['angular_velocity']['y']:.6f}")
                        rospy.logwarn(f"    z: {self.last_raw_data['angular_velocity']['z']:.6f}")
                        rospy.logwarn(f"  Linear acceleration (m/s²):")
                        rospy.logwarn(f"    x: {self.last_raw_data['linear_acceleration']['x']:.6f}")
                        rospy.logwarn(f"    y: {self.last_raw_data['linear_acceleration']['y']:.6f}")
                        rospy.logwarn(f"    z: {self.last_raw_data['linear_acceleration']['z']:.6f}")
                        rospy.logwarn("")
                        rospy.logwarn("PROCESSED DATA (after x/y swap):")
                        rospy.logwarn(f"  Orientation (quaternion):")
                        rospy.logwarn(f"    x: {self.last_processed_data['orientation']['x']:.6f}")
                        rospy.logwarn(f"    y: {self.last_processed_data['orientation']['y']:.6f}")
                        rospy.logwarn(f"    z: {self.last_processed_data['orientation']['z']:.6f}")
                        rospy.logwarn(f"    w: {self.last_processed_data['orientation']['w']:.6f}")
                        rospy.logwarn(f"  Euler angles:")
                        rospy.logwarn(f"    roll:  {self.last_processed_data['orientation']['roll_deg']:.2f}° ({self.last_processed_data['orientation']['roll']:.6f} rad)")
                        rospy.logwarn(f"    pitch: {self.last_processed_data['orientation']['pitch_deg']:.2f}° ({self.last_processed_data['orientation']['pitch']:.6f} rad)")
                        rospy.logwarn(f"    yaw:   {self.last_processed_data['orientation']['yaw_deg']:.2f}° ({self.last_processed_data['orientation']['yaw']:.6f} rad)")
                        rospy.logwarn(f"  Angular velocity (rad/s):")
                        rospy.logwarn(f"    x: {self.last_processed_data['angular_velocity']['x']:.6f}")
                        rospy.logwarn(f"    y: {self.last_processed_data['angular_velocity']['y']:.6f}")
                        rospy.logwarn(f"    z: {self.last_processed_data['angular_velocity']['z']:.6f}")
                        rospy.logwarn(f"  Linear acceleration (m/s²):")
                        rospy.logwarn(f"    x: {self.last_processed_data['linear_acceleration']['x']:.6f}")
                        rospy.logwarn(f"    y: {self.last_processed_data['linear_acceleration']['y']:.6f}")
                        rospy.logwarn(f"    z: {self.last_processed_data['linear_acceleration']['z']:.6f}")
                        rospy.logwarn("=" * 80)
                        self.fall_data_logged = True
            elif self.robot_state == 'stand' and old_state != 'stand':
                self.fall_start_time = None
                self.last_auto_getup_time = 0
                self.getup_action_in_progress = False
                self.getup_attempt_count = 0
                self.fall_check_cooldown = current_time + FALL_CHECK_COOLDOWN_DURATION
                self.fall_data_logged = False  # Сбрасываем флаг для следующего падения
                rospy.loginfo(f"✅ Robot recovered to stand position")
    
    def _check_robot_at_rest(self):
        """
        Проверяет, находится ли робот в покое (изменения ускорений ниже порога).
        
        Returns:
            bool: True если робот в покое
        """
        if not self.fall_config['check_rest_enabled']:
            return True
        
        threshold = self.fall_config['rest_accel_threshold']
        check_duration = self.fall_config['rest_check_duration']
        
        # Нужно достаточно данных для проверки
        if len(self.accel_history['x']) < 2:
            return False
        
        # Вычисляем изменения ускорений за последние данные
        # Используем последние N значений, соответствующих check_duration
        # Предполагаем частоту обновления ~50Hz, значит check_duration * 50 значений
        sample_rate = 50.0  # Hz
        num_samples = int(check_duration * sample_rate)
        num_samples = min(num_samples, len(self.accel_history['x']))
        
        if num_samples < 2:
            return False
        
        # Вычисляем максимальное изменение ускорения
        max_change = 0.0
        for i in range(len(self.accel_history['x']) - num_samples, len(self.accel_history['x']) - 1):
            change_x = abs(self.accel_history['x'][i+1] - self.accel_history['x'][i])
            change_y = abs(self.accel_history['y'][i+1] - self.accel_history['y'][i])
            change_z = abs(self.accel_history['z'][i+1] - self.accel_history['z'][i])
            max_change = max(max_change, change_x, change_y, change_z)
        
        return max_change < threshold
    
    def check_auto_getup(self, motion_manager, can_move_func, 
                         fall_forward_action, fall_backward_action,
                         fall_left_action=None, fall_right_action=None):
        """
        Проверяет необходимость автоматического подъема.
        
        Args:
            motion_manager: Экземпляр MotionManager
            can_move_func: Функция проверки разрешения на движение
            fall_forward_action: Название действия для подъема из падения вперед
            fall_backward_action: Название действия для подъема из падения назад
            fall_left_action: Название действия для подъема из падения влево (заглушка)
            fall_right_action: Название действия для подъема из падения вправо (заглушка)
        
        Returns:
            bool: True если была выполнена попытка подъема
        """
        if not self.auto_getup_enabled:
            return False
        
        if not can_move_func():
            return False
        
        current_time = rospy.get_time()
        
        if current_time < self.fall_check_cooldown:
            return False
        
        if self.robot_state == 'stand':
            self.fall_start_time = None
            self.getup_action_in_progress = False
            return False
        
        if self.fall_start_time is None:
            self.fall_start_time = current_time
            return False
        
        if current_time - self.last_auto_getup_time < self.auto_getup_interval:
            return False
        
        fall_duration = current_time - self.fall_start_time
        if fall_duration < self.fall_time_threshold:
            return False
        
        if self.getup_attempt_count >= self.max_getup_attempts:
            rospy.logerr(f"🚨 Auto-getup: Max attempts reached. Calling for help!")
            return False
        
        if self.getup_action_in_progress:
            if current_time - self.last_auto_getup_time > GETUP_ACTION_TIMEOUT:
                self.getup_action_in_progress = False
                self.getup_attempt_count += 1
            else:
                return False
        
        rospy.logwarn(f"🚨 Auto-getup: Robot has been down for {fall_duration:.1f} seconds")
        rospy.logwarn(f"   Attempt {self.getup_attempt_count + 1}/{self.max_getup_attempts}: {self.robot_state}")
        
        try:
            action_to_run = None
            if self.robot_state == 'fall_forward':
                action_to_run = fall_forward_action
            elif self.robot_state == 'fall_backward':
                action_to_run = fall_backward_action
            elif self.robot_state == 'fall_left':
                # Заглушка для падения влево
                if fall_left_action:
                    action_to_run = fall_left_action
                else:
                    rospy.logwarn(f"⚠️ Fall left action not implemented yet")
                    return False
            elif self.robot_state == 'fall_right':
                # Заглушка для падения вправо
                if fall_right_action:
                    action_to_run = fall_right_action
                else:
                    rospy.logwarn(f"⚠️ Fall right action not implemented yet")
                    return False
            
            if action_to_run and motion_manager is not None:
                self.getup_action_in_progress = True
                self.getup_attempt_count += 1
                rospy.loginfo(f"🤖 Executing auto-getup action: {action_to_run}")
                motion_manager.run_action(action_to_run)
                self.last_auto_getup_time = current_time
                return True
        except Exception as e:
            rospy.logerr(f"❌ Error in auto-getup: {e}")
            self.getup_action_in_progress = False
        
        return False
    
    def get_robot_state(self):
        """Возвращает текущее состояние робота."""
        return self.robot_state
