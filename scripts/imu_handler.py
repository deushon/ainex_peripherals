#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль обработки данных IMU: обнаружение падений, калибровка, резонанс.
"""

# ========== КОНФИГУРАЦИЯ ==========
# Обнаружение падений
FALL_COUNT_THRESHOLD = 50  # Порог счетчика для определения падения
FALL_ACCEL_THRESHOLD = 7.0  # Порог ускорения для падения (м/с²)
FALL_ANGLE_THRESHOLD = 30.0  # Порог угла для падения (градусы)
FALL_COUNT_INCREMENT = 1  # Приращение счетчика при падении
FALL_COUNT_DECREMENT = 2  # Уменьшение счетчика при нормальном состоянии
FALL_COUNT_COOLDOWN_DECREMENT = 15  # Уменьшение счетчика в период cooldown
FALL_CHECK_COOLDOWN_DURATION = 10.0  # Длительность cooldown после подъема (сек)

# Автоматический подъем
AUTO_GETUP_ENABLED = True  # Включен ли автоматический подъем
FALL_TIME_THRESHOLD = 3.0  # Время падения для автоматического подъема (сек)
AUTO_GETUP_INTERVAL = 5.0  # Минимальный интервал между попытками подъема (сек)
MAX_GETUP_ATTEMPTS = 2  # Максимальное количество попыток подъема
GETUP_ACTION_TIMEOUT = 10.0  # Таймаут выполнения действия подъема (сек)

# Обнаружение резонанса
RESONANCE_DETECTION_ENABLED = True  # Включено ли обнаружение резонанса
MAX_SAFE_AMPLITUDE = 0.3  # Максимальная безопасная амплитуда качания (м/с²)
CRITICAL_AMPLITUDE = 0.5  # Критическая амплитуда (м/с²)
RESONANCE_ADAPTATION_STEP = 0.05  # Шаг изменения фактора адаптации
RESONANCE_MIN_FACTOR = 0.5  # Минимальный фактор адаптации
RESONANCE_REDUCTION_RATIO = 0.4  # Коэффициент уменьшения при резонансе
RESONANCE_PITCH_MULTIPLIER = 2.0  # Множитель для амплитуды pitch
RESONANCE_HISTORY_MIN = 10  # Минимальный размер истории для анализа резонанса

# История IMU данных
IMU_HISTORY_SIZE = 50  # Размер истории IMU данных

# Логирование
LOG_INTERVAL = 2.0  # Интервал логирования (сек)
# ===================================

import rospy
import math
import threading
import numpy as np
from sensor_msgs.msg import Imu


# Матрица преобразования осей (как в imu_visualizer.py)
AXIS_TRANSFORM_MATRIX = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, -1.0, 0.0],
])


def transform_axes_vector(vec):
    """Преобразование вектора согласно матрице преобразования осей"""
    return AXIS_TRANSFORM_MATRIX.dot(vec)


class IMUHandler:
    """
    Класс для обработки данных IMU: обнаружение падений, калибровка.
    """
    
    def __init__(self):
        """Инициализация обработчика IMU."""
        # Состояние робота
        self.robot_state = 'stand'
        self.count_lie = 0
        self.count_recline = 0
        self.fall_check_cooldown = 0
        
        # История IMU данных для анализа резонанса
        self.imu_history_size = IMU_HISTORY_SIZE
        self.imu_history = {
            'ax': [],
            'ay': [],
            'az': [],
            'roll': [],
            'pitch': [],
            'yaw': []
        }
        
        # Параметры для автоматического подъема
        self.auto_getup_enabled = AUTO_GETUP_ENABLED
        self.fall_time_threshold = FALL_TIME_THRESHOLD
        self.fall_start_time = None
        self.last_auto_getup_time = 0
        self.auto_getup_interval = AUTO_GETUP_INTERVAL
        self.getup_action_in_progress = False
        self.getup_attempt_count = 0
        self.max_getup_attempts = MAX_GETUP_ATTEMPTS
        
        # Параметры для обнаружения резонанса
        self.resonance_detection_enabled = RESONANCE_DETECTION_ENABLED
        self.max_safe_amplitude = MAX_SAFE_AMPLITUDE
        self.critical_amplitude = CRITICAL_AMPLITUDE
        self.current_adaptation_factor = 1.0
        
        # Блокировка для thread-safe доступа
        self.imu_lock = threading.Lock()
        
        # Логирование
        self.last_log_time = 0
        self.log_interval = LOG_INTERVAL
        self.imu_data_received = False
        
        rospy.loginfo("IMUHandler module initialized")
    
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
                # Получаем сырые данные
                accel_raw = np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])
                
                # Сохраняем исходные значения для логики падения
                ay_original = msg.linear_acceleration.y
                az_original = msg.linear_acceleration.z
                
                # Применяем преобразование осей
                accel_transformed = transform_axes_vector(accel_raw)
                ax = accel_transformed[0]
                ay = accel_transformed[1]
                az = accel_transformed[2]
                
                # Гироскоп тоже преобразуем
                gyro_raw = np.array([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ])
                gyro_transformed = transform_axes_vector(gyro_raw)
                gx = gyro_transformed[0]
                gy = gyro_transformed[1]
                gz = gyro_transformed[2]
                
                # Извлечение углов ориентации из quaternion
                qx = msg.orientation.x
                qy = msg.orientation.y
                qz = msg.orientation.z
                qw = msg.orientation.w
                
                # Преобразование quaternion в углы Эйлера
                sinr_cosp = 2 * (qw * qx + qy * qz)
                cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                roll = math.atan2(sinr_cosp, cosr_cosp)
                
                sinp = 2 * (qw * qy - qz * qx)
                if abs(sinp) >= 1:
                    pitch = math.copysign(math.pi / 2, sinp)
                else:
                    pitch = math.asin(sinp)
                
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                # Обновление истории IMU данных
                self.imu_history['ax'].append(ax)
                self.imu_history['ay'].append(ay)
                self.imu_history['az'].append(az)
                self.imu_history['roll'].append(roll)
                self.imu_history['pitch'].append(pitch)
                self.imu_history['yaw'].append(yaw)
                
                # Ограничение размера истории
                for key in self.imu_history:
                    if len(self.imu_history[key]) > self.imu_history_size:
                        self.imu_history[key].pop(0)
                
                # Логирование первого получения IMU данных
                if not self.imu_data_received:
                    self.imu_data_received = True
                    rospy.loginfo("✅ IMU data received!")
            
            # Обработка падения (используем исходные значения)
            current_time = rospy.get_time()
            
            if current_time >= self.fall_check_cooldown:
                if abs(az_original) > 1e-6:
                    angle_rad = math.atan2(abs(ay_original), abs(az_original))
                    angle_deg = math.degrees(angle_rad)
                else:
                    angle_deg = 90.0
                
                # Логика падения
                if angle_deg < FALL_ANGLE_THRESHOLD:
                    if az_original > FALL_ACCEL_THRESHOLD:
                        self.count_lie += FALL_COUNT_INCREMENT
                        self.count_recline = max(0, self.count_recline - FALL_COUNT_DECREMENT)
                    elif az_original < -FALL_ACCEL_THRESHOLD:
                        self.count_recline += FALL_COUNT_INCREMENT
                        self.count_lie = max(0, self.count_lie - FALL_COUNT_DECREMENT)
                    else:
                        self.count_lie = max(0, self.count_lie - FALL_COUNT_DECREMENT)
                        self.count_recline = max(0, self.count_recline - FALL_COUNT_DECREMENT)
                else:
                    self.count_lie = max(0, self.count_lie - FALL_COUNT_DECREMENT)
                    self.count_recline = max(0, self.count_recline - FALL_COUNT_DECREMENT)
            else:
                # В период cooldown - агрессивно сбрасываем счетчики
                self.count_lie = max(0, self.count_lie - FALL_COUNT_COOLDOWN_DECREMENT)
                self.count_recline = max(0, self.count_recline - FALL_COUNT_COOLDOWN_DECREMENT)
            
            old_state = self.robot_state
            
            if self.count_lie > FALL_COUNT_THRESHOLD:
                self.robot_state = 'lie_to_stand'
            elif self.count_recline > FALL_COUNT_THRESHOLD:
                self.robot_state = 'recline_to_stand'
            else:
                self.robot_state = 'stand'
            
            if old_state != self.robot_state:
                rospy.loginfo(f"🔄 IMU detected robot state change: '{old_state}' -> '{self.robot_state}'")
                if self.robot_state != 'stand' and old_state == 'stand':
                    if self.fall_start_time is None:
                        self.fall_start_time = current_time
                        rospy.logwarn(f"⚠️ Robot fell! State: {self.robot_state}")
                elif self.robot_state == 'stand' and old_state != 'stand':
                    self.fall_start_time = None
                    self.last_auto_getup_time = 0
                    self.getup_action_in_progress = False
                    self.getup_attempt_count = 0
                    self.count_lie = 0
                    self.count_recline = 0
                    self.fall_check_cooldown = current_time + FALL_CHECK_COOLDOWN_DURATION
                    rospy.loginfo(f"✅ Robot recovered to stand position")
            
            # Возвращаем обработанные данные
            return {
                'ax': ax,
                'ay': ay,
                'az': az,
                'gx': gx,
                'gy': gy,
                'gz': gz,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'robot_state': self.robot_state,
                'fall_detected': self.robot_state != 'stand'
            }
            
        except Exception as e:
            rospy.logwarn(f"Error processing IMU data: {e}")
            return None
    
    def check_auto_getup(self, motion_manager, can_move_func, lie_action, recline_action):
        """
        Проверяет необходимость автоматического подъема.
        
        Args:
            motion_manager: Экземпляр MotionManager
            can_move_func: Функция проверки разрешения на движение
            lie_action: Название действия для подъема из положения лежа
            recline_action: Название действия для подъема из положения на спине
        
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
            return False  # Нужно вызвать функцию помощи отдельно
        
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
            if self.robot_state == 'lie_to_stand':
                action_to_run = lie_action
            elif self.robot_state == 'recline_to_stand':
                action_to_run = recline_action
            
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
    
    def check_resonance(self, status):
        """
        Проверяет резонанс и возвращает фактор адаптации.
        
        Args:
            status: Статус движения ('move' или 'stop')
        
        Returns:
            float: Фактор адаптации (1.0 = без изменений, <1.0 = уменьшение)
        """
        if not self.resonance_detection_enabled or status != 'move':
            return 1.0
        
        try:
            with self.imu_lock:
                if len(self.imu_history['ax']) < RESONANCE_HISTORY_MIN:
                    return self.current_adaptation_factor
                
                # Расчет амплитуды качания
                def calculate_amplitude(data_list):
                    if len(data_list) < 2:
                        return 0.0
                    mean = sum(data_list) / len(data_list)
                    variance = sum((x - mean) ** 2 for x in data_list) / len(data_list)
                    return math.sqrt(variance)
                
                amplitude_x = calculate_amplitude(self.imu_history['ax'])
                amplitude_y = calculate_amplitude(self.imu_history['ay'])
                amplitude_z = calculate_amplitude(self.imu_history['az'])
                amplitude_pitch = calculate_amplitude(self.imu_history['pitch'])
                
                max_amplitude = max(amplitude_x, amplitude_y, amplitude_z)
                combined_amplitude = max(max_amplitude, abs(amplitude_pitch) * RESONANCE_PITCH_MULTIPLIER)
            
            old_factor = self.current_adaptation_factor
            
            if combined_amplitude > self.critical_amplitude:
                self.current_adaptation_factor = RESONANCE_MIN_FACTOR
                rospy.logwarn(f"⚠️ CRITICAL resonance detected! (amplitude={combined_amplitude:.3f} m/s²)")
            elif combined_amplitude > self.max_safe_amplitude:
                ratio = (combined_amplitude - self.max_safe_amplitude) / (self.critical_amplitude - self.max_safe_amplitude)
                self.current_adaptation_factor = 1.0 - ratio * RESONANCE_REDUCTION_RATIO
                rospy.loginfo(f"📊 High amplitude detected (amplitude={combined_amplitude:.3f} m/s²)")
            else:
                if self.current_adaptation_factor < 1.0:
                    self.current_adaptation_factor = min(1.0, self.current_adaptation_factor + RESONANCE_ADAPTATION_STEP)
            
            return self.current_adaptation_factor
            
        except Exception as e:
            rospy.logwarn(f"❌ Error in resonance detection: {e}")
            return self.current_adaptation_factor
    
    def get_robot_state(self):
        """Возвращает текущее состояние робота."""
        return self.robot_state
    
    def reset_adaptation(self):
        """Сбрасывает фактор адаптации."""
        self.current_adaptation_factor = 1.0

