#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Приложение для визуализации данных IMU сенсора робота через ROS Bridge
Подключается к топику /imu и отображает:
- Графики ориентации (наклон по осям)
- Графики акселерометра
- 3D визуализацию положения робота
"""

import roslibpy
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import threading
import time
from scipy.spatial.transform import Rotation as R
try:
    import tkinter as tk
    from tkinter import ttk
    TKINTER_AVAILABLE = True
except ImportError:
    TKINTER_AVAILABLE = False
    print("ВНИМАНИЕ: tkinter не доступен. GUI для настроек будет отключен.")


AXIS_TRANSFORM_MATRIX = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
    [0.0, -1.0, 0.0],
])


def transform_axes_vector(vec):
    """Инвертировать все оси и поменять местами Y и Z"""
    return AXIS_TRANSFORM_MATRIX.dot(vec)


def transform_orientation_quat(quat):
    """Применить преобразование осей к quaternion"""
    rotation = safe_quat_to_rotation(quat)
    if rotation is None:
        return quat
    transformed_matrix = rotation.as_matrix().dot(AXIS_TRANSFORM_MATRIX)
    transformed_rotation = R.from_matrix(transformed_matrix)
    return transformed_rotation.as_quat()


def validate_quaternion(quat):
    """
    Проверка и нормализация quaternion
    
    Args:
        quat: quaternion [x, y, z, w]
    
    Returns:
        Валидный quaternion [x, y, z, w] или единичный quaternion [0, 0, 0, 1] если входной невалиден
    """
    quat = np.array(quat)
    norm = np.linalg.norm(quat)
    
    # Если quaternion нулевой или очень маленький, возвращаем единичный
    if norm < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    
    # Нормализуем quaternion
    quat_normalized = quat / norm
    
    # Проверяем, что w не слишком мал (признак некорректного quaternion)
    if abs(quat_normalized[3]) < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    
    return quat_normalized


def safe_quat_to_rotation(quat):
    """
    Безопасное преобразование quaternion в Rotation объект
    
    Args:
        quat: quaternion [x, y, z, w]
    
    Returns:
        Rotation объект или None если quaternion невалиден
    """
    try:
        quat_valid = validate_quaternion(quat)
        return R.from_quat(quat_valid)
    except (ValueError, TypeError) as e:
        # Если все равно ошибка, возвращаем единичную ориентацию
        return R.identity()


class IMUVisualizer:
    def __init__(self, max_data_points=500, ros_host='localhost', ros_port=9090, calibration_samples=10,
                 friction_coefficient=0.95, accel_threshold=0.15, velocity_threshold=0.03, topic_name='/imu'):
        """
        Инициализация визуализатора IMU
        
        Args:
            max_data_points: Максимальное количество точек данных для отображения
            ros_host: IP адрес ROS bridge сервера (по умолчанию: localhost)
            ros_port: Порт ROS bridge сервера (по умолчанию: 9090)
            calibration_samples: Количество измерений для калибровки (по умолчанию: 10)
        """
        self.max_data_points = max_data_points
        self.calibration_samples = calibration_samples
        self.topic_name = topic_name
        
        # Калибровка IMU
        self.calibration_data_accel = []  # Буфер для калибровочных данных акселерометра
        self.calibration_data_gyro = []   # Буфер для калибровочных данных гироскопа
        self.calibration_data_orientation = []  # Буфер для ориентации при калибровке
        self.accel_bias = np.array([0.0, 0.0, 0.0])  # Смещение акселерометра
        self.gyro_bias = np.array([0.0, 0.0, 0.0])    # Смещение гироскопа
        self.calibration_complete = False  # Флаг завершения калибровки
        
        # Буферы данных
        self.time_data = deque(maxlen=max_data_points)
        self.orientation_euler = {'roll': deque(maxlen=max_data_points),
                                  'pitch': deque(maxlen=max_data_points),
                                  'yaw': deque(maxlen=max_data_points)}
        self.angular_velocity = {'x': deque(maxlen=max_data_points),
                                'y': deque(maxlen=max_data_points),
                                'z': deque(maxlen=max_data_points)}
        self.linear_acceleration = {'x': deque(maxlen=max_data_points),
                                   'y': deque(maxlen=max_data_points),
                                   'z': deque(maxlen=max_data_points)}
        
        # Данные для 3D визуализации (положение и ориентация)
        self.position = {'x': deque(maxlen=max_data_points),
                        'y': deque(maxlen=max_data_points),
                        'z': deque(maxlen=max_data_points)}
        self.orientation_quat = {'x': deque(maxlen=max_data_points),
                                'y': deque(maxlen=max_data_points),
                                'z': deque(maxlen=max_data_points),
                                'w': deque(maxlen=max_data_points)}
        
        # Переменные для вычисления положения
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.last_time = None
        self.initial_orientation = None
        self.orientation_reference = None
        
        # Коэффициент трения для затухания остаточных ускорений
        self.friction_coefficient = friction_coefficient  # Коэффициент затухания скорости (0.95 = 5% потерь)
        self.accel_threshold = accel_threshold  # Порог ускорения для определения покоя (м/с²)
        self.velocity_threshold = velocity_threshold  # Порог скорости для определения покоя (м/с)
        
        # Флаги
        self.data_lock = threading.Lock()
        self.running = True
        self.data_received = False
        
        # Настройка ROS Bridge через roslibpy
        print(f"Подключение к ROS Bridge: {ros_host}:{ros_port}")
        self.ros_client = roslibpy.Ros(host=ros_host, port=ros_port)
        self.ros_client.run()
        
        # Проверка подключения
        if not self.ros_client.is_connected:
            print("Ожидание подключения к ROS Bridge...")
            timeout = 5
            start_time = time.time()
            while not self.ros_client.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self.ros_client.is_connected:
                raise ConnectionError(f"Не удалось подключиться к ROS Bridge на {ros_host}:{ros_port}")
        
        print("✓ Подключено к ROS Bridge")
        
        # Подписка на топик IMU
        print(f"Подписка на топик {self.topic_name}...")
        self.imu_topic = roslibpy.Topic(self.ros_client, self.topic_name, 'sensor_msgs/Imu')
        self.imu_topic.subscribe(self.imu_callback)
        
        print(f"Ожидание данных из топика {self.topic_name}...")
        
        # Ждем получения первых данных
        timeout = 10
        start_time = time.time()
        while not self.data_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if not self.data_received:
            print("ВНИМАНИЕ: Данные не получены. Проверьте подключение к ROS Bridge и наличие топика /imu.")
        else:
            print("Данные получены! Запуск визуализации...")
            print(f"⚠ ВАЖНО: Убедитесь, что робот находится в покое для калибровки!")
            print(f"Калибровка будет выполнена по первым {self.calibration_samples} измерениям...")
        
        # Создание графиков
        self.setup_plots()
    
    def perform_calibration(self, accel_data, gyro_data, orientation_quat):
        """
        Выполнение калибровки IMU по первым N измерениям в покое
        Учитывает гравитацию при вычислении смещения
        
        Args:
            accel_data: Данные акселерометра [x, y, z]
            gyro_data: Данные гироскопа [x, y, z]
            orientation_quat: Ориентация в виде quaternion [x, y, z, w]
        """
        if self.calibration_complete:
            return
        
        # Добавляем данные в буфер калибровки
        self.calibration_data_accel.append(accel_data)
        self.calibration_data_gyro.append(gyro_data)
        self.calibration_data_orientation.append(orientation_quat)
        
        # Когда собрано достаточно данных, вычисляем смещение
        if len(self.calibration_data_accel) >= self.calibration_samples:
            # Гравитация в глобальной системе координат (направлена вниз по оси -Z)
            gravity_global = np.array([0.0, 0.0, -9.81])
            
            # Вычисляем смещение акселерометра
            # При калибровке робот стоит на месте, поэтому акселерометр показывает:
            # смещение + гравитация в локальной системе координат
            # Для вычисления смещения вычитаем гравитацию из каждого измерения
            accel_without_gravity = []
            orientations_valid = 0
            for accel, quat in zip(self.calibration_data_accel, self.calibration_data_orientation):
                # Безопасное преобразование quaternion в объект Rotation
                r = safe_quat_to_rotation(quat)
                if r is not None:
                    # Преобразуем гравитацию из глобальной системы в локальную
                    gravity_local = r.inv().apply(gravity_global)
                    # Вычитаем гравитацию из показаний акселерометра
                    # Остается только смещение (если робот стоит на месте)
                    accel_no_gravity = accel - gravity_local
                    accel_without_gravity.append(accel_no_gravity)
                    orientations_valid += 1
                else:
                    # Если quaternion некорректный, используем данные без вычитания гравитации
                    accel_without_gravity.append(accel)
            
            if orientations_valid < len(self.calibration_data_accel):
                print(f"  ⚠ ВНИМАНИЕ: {len(self.calibration_data_accel) - orientations_valid} измерений имели некорректную ориентацию")
            
            # Вычисляем среднее значение (смещение) для акселерометра
            # Это должно быть близко к нулю, если калибровка правильная
            accel_array = np.array(accel_without_gravity)
            self.accel_bias = np.mean(accel_array, axis=0)
            
            # Вычисляем стандартное отклонение для проверки стабильности данных
            accel_std = np.std(accel_array, axis=0)
            
            # Вычисляем среднее значение (смещение) для гироскопа
            gyro_array = np.array(self.calibration_data_gyro)
            self.gyro_bias = np.mean(gyro_array, axis=0)
            gyro_std = np.std(gyro_array, axis=0)
            
            self.calibration_complete = True
            
            # Сбрасываем состояние вычисления положения для начала с чистого листа
            self.velocity = np.array([0.0, 0.0, 0.0])
            self.current_position = np.array([0.0, 0.0, 0.0])
            self.last_time = None
            self.initial_orientation = None
            last_calib_quat = self.calibration_data_orientation[-1] if self.calibration_data_orientation else [0.0, 0.0, 0.0, 1.0]
            last_calib_quat = validate_quaternion(last_calib_quat)
            reference_rotation = safe_quat_to_rotation(last_calib_quat)
            self.orientation_reference = reference_rotation if reference_rotation is not None else R.identity()
            
            bias_magnitude = np.linalg.norm(self.accel_bias)
            print(f"\n✓ Калибровка завершена!")
            print(f"  Смещение акселерометра (без гравитации): X={self.accel_bias[0]:.4f}, Y={self.accel_bias[1]:.4f}, Z={self.accel_bias[2]:.4f} м/с²")
            print(f"  Стандартное отклонение: X={accel_std[0]:.4f}, Y={accel_std[1]:.4f}, Z={accel_std[2]:.4f} м/с²")
            print(f"  Величина смещения: {bias_magnitude:.4f} м/с² (должно быть близко к 0)")
            if bias_magnitude > 0.5:
                print(f"  ⚠ ВНИМАНИЕ: Смещение велико! Убедитесь, что робот стоит на месте во время калибровки.")
            print(f"  Смещение гироскопа: X={self.gyro_bias[0]:.4f}, Y={self.gyro_bias[1]:.4f}, Z={self.gyro_bias[2]:.4f} рад/с")
            print(f"  Стандартное отклонение гироскопа: X={gyro_std[0]:.4f}, Y={gyro_std[1]:.4f}, Z={gyro_std[2]:.4f} рад/с")
            print(f"  Гравитация будет вычитаться при каждом измерении с учетом ориентации.")
            print(f"  На графиках будут отображаться данные БЕЗ гравитации (должны быть близки к 0 при покое).\n")
        
    def imu_callback(self, msg):
        """Обработчик сообщений IMU"""
        with self.data_lock:
            # roslibpy передает данные как словарь
            header = msg.get('header', {})
            stamp = header.get('stamp', {})
            current_time = stamp.get('secs', 0) + stamp.get('nsecs', 0) * 1e-9
            if current_time == 0:
                current_time = time.time()  # Используем системное время, если нет timestamp
            
            # Ориентация (quaternion -> euler)
            orientation = msg.get('orientation', {})
            quat = [orientation.get('x', 0.0), orientation.get('y', 0.0), 
                   orientation.get('z', 0.0), orientation.get('w', 1.0)]
            quat = transform_orientation_quat(quat)
            
            # Угловая скорость (сырые данные)
            angular_velocity = msg.get('angular_velocity', {})
            gyro_raw = np.array([angular_velocity.get('x', 0.0),
                                angular_velocity.get('y', 0.0),
                                angular_velocity.get('z', 0.0)])
            gyro_raw = transform_axes_vector(gyro_raw)
            
            # Линейное ускорение (сырые данные)
            linear_acceleration = msg.get('linear_acceleration', {})
            accel_raw = np.array([linear_acceleration.get('x', 0.0),
                                 linear_acceleration.get('y', 0.0),
                                 linear_acceleration.get('z', 0.0)])
            accel_raw = transform_axes_vector(accel_raw)
            
            # Калибровка: собираем данные для вычисления смещения
            if not self.calibration_complete:
                self.perform_calibration(accel_raw, gyro_raw, quat)
                # До завершения калибровки НЕ сохраняем данные и НЕ вычисляем положение
                self.data_received = True  # Помечаем, что данные получены, но не обрабатываем их
                return  # Выходим из функции до завершения калибровки
            
            # После завершения калибровки начинаем обработку данных
            # Применяем калибровку (вычитаем смещение)
            accel_calibrated = accel_raw - self.accel_bias
            gyro_calibrated = gyro_raw - self.gyro_bias
            
            # Сохраняем время только после калибровки
            self.time_data.append(current_time)
            
            # Нормализуем quaternion перед использованием и сохранением
            quat = validate_quaternion(quat)
            
            # Сохраняем quaternion для 3D визуализации (уже нормализованный)
            reference_rotation = self.orientation_reference
            r = safe_quat_to_rotation(quat)
            if r is not None:
                relative_rotation = r if reference_rotation is None else reference_rotation.inv() * r
                relative_quat = relative_rotation.as_quat()
                self.orientation_quat['x'].append(relative_quat[0])
                self.orientation_quat['y'].append(relative_quat[1])
                self.orientation_quat['z'].append(relative_quat[2])
                self.orientation_quat['w'].append(relative_quat[3])
            else:
                self.orientation_quat['x'].append(0.0)
                self.orientation_quat['y'].append(0.0)
                self.orientation_quat['z'].append(0.0)
                self.orientation_quat['w'].append(1.0)
            
            # Преобразуем quaternion в углы Эйлера (roll, pitch, yaw)
            if r is not None:
                try:
                    relative_rotation = r if reference_rotation is None else reference_rotation.inv() * r
                    euler = relative_rotation.as_euler('xyz', degrees=True)
                    self.orientation_euler['roll'].append(euler[0])
                    self.orientation_euler['pitch'].append(euler[1])
                    self.orientation_euler['yaw'].append(euler[2])
                except:
                    # Если преобразование не удалось, используем нули
                    self.orientation_euler['roll'].append(0.0)
                    self.orientation_euler['pitch'].append(0.0)
                    self.orientation_euler['yaw'].append(0.0)
            else:
                # Если quaternion некорректный, используем нули
                self.orientation_euler['roll'].append(0.0)
                self.orientation_euler['pitch'].append(0.0)
                self.orientation_euler['yaw'].append(0.0)
            
            # Вычитаем гравитацию из откалиброванных данных для отображения на графиках
            # Чтобы при покое значения были близки к нулю
            current_orientation = safe_quat_to_rotation(quat)
            if current_orientation is not None:
                try:
                    gravity_global = np.array([0.0, 0.0, -9.81])
                    gravity_local = current_orientation.inv().apply(gravity_global)
                    accel_for_display = accel_calibrated - gravity_local
                except:
                    # Если не удалось вычислить гравитацию, используем данные без гравитации
                    accel_for_display = accel_calibrated
            else:
                # Если quaternion некорректный, используем данные без гравитации
                accel_for_display = accel_calibrated
            
            # Сохраняем откалиброванные данные (без гравитации для графиков)
            self.angular_velocity['x'].append(gyro_calibrated[0])
            self.angular_velocity['y'].append(gyro_calibrated[1])
            self.angular_velocity['z'].append(gyro_calibrated[2])
            
            self.linear_acceleration['x'].append(accel_for_display[0])
            self.linear_acceleration['y'].append(accel_for_display[1])
            self.linear_acceleration['z'].append(accel_for_display[2])
            
            # Вычисление положения (интеграция акселерометра) - только после калибровки
            if self.last_time is not None:
                dt = current_time - self.last_time
                if dt > 0 and dt < 1.0:  # Защита от больших скачков времени
                    # Сохраняем начальную ориентацию при первом измерении после калибровки
                    if self.initial_orientation is None:
                        self.initial_orientation = safe_quat_to_rotation(quat)
                        if self.initial_orientation is None:
                            self.initial_orientation = R.identity()
                    
                    # Текущая ориентация робота
                    current_orientation = safe_quat_to_rotation(quat)
                    if current_orientation is None:
                        # Если quaternion некорректный, используем предыдущую ориентацию или единичную
                        current_orientation = self.initial_orientation if self.initial_orientation is not None else R.identity()
                    
                    # Ускорение в локальной системе координат робота (используем откалиброванные данные)
                    # accel_calibrated = accel_raw - accel_bias (уже вычли смещение)
                    accel_local = accel_calibrated
                    
                    # Гравитация в глобальной системе координат (направлена вниз по оси -Z)
                    gravity_global = np.array([0.0, 0.0, -9.81])
                    
                    # Преобразуем гравитацию из глобальной системы в локальную систему координат робота
                    # Используем обратное преобразование ориентации
                    gravity_local = current_orientation.inv().apply(gravity_global)
                    
                    # Вычитаем гравитацию в локальной системе координат, чтобы получить реальное ускорение
                    # Если робот стоит на месте, accel_real_local должно быть близко к нулю
                    accel_real_local = accel_local - gravity_local
                    
                    # Применяем коэффициент трения для затухания остаточных ускорений
                    # Малые ускорения затухают быстрее
                    accel_magnitude = np.linalg.norm(accel_real_local)
                    if accel_magnitude < self.accel_threshold:
                        # Если ускорение очень мало, применяем сильное затухание
                        accel_real_local *= 0.3  # Уменьшаем остаточные ускорения
                    
                    # Проверка: если ускорение и скорость очень малы (робот стоит на месте), не интегрируем
                    velocity_magnitude = np.linalg.norm(self.velocity)
                    
                    if accel_magnitude < self.accel_threshold and velocity_magnitude < self.velocity_threshold:
                        # Если робот стоит на месте, применяем сильное затухание скорости
                        self.velocity *= 0.3  # Очень сильное затухание скорости (коэффициент трения)
                        # Не интегрируем положение при покое
                    else:
                        # Преобразуем реальное ускорение из локальной системы координат в глобальную
                        accel_global = current_orientation.apply(accel_real_local)
                        
                        # Интегрируем для получения скорости
                        self.velocity += accel_global * dt
                        
                        # Интегрируем для получения положения
                        self.current_position += self.velocity * dt + 0.5 * accel_global * dt * dt
                        
                        # Применяем коэффициент трения для затухания скорости
                        self.velocity *= self.friction_coefficient
            else:
                # Инициализация при первом сообщении после калибровки
                self.initial_orientation = safe_quat_to_rotation(quat)
                if self.initial_orientation is None:
                    self.initial_orientation = R.identity()
            
            self.last_time = current_time
            
            # Сохраняем положение
            self.position['x'].append(self.current_position[0])
            self.position['y'].append(self.current_position[1])
            self.position['z'].append(self.current_position[2])
            
            self.data_received = True
    
    
    def setup_plots(self):
        """Настройка графиков"""
        # Окно 1: Графики ориентации и акселерометра
        self.fig1 = plt.figure(figsize=(14, 8))
        self.fig1.suptitle('IMU Данные - Ориентация и Акселерометр', fontsize=14, fontweight='bold')
        
        # Графики ориентации
        self.ax_orientation = plt.subplot(2, 1, 1)
        self.ax_orientation.set_title('Ориентация (Roll, Pitch, Yaw)')
        self.ax_orientation.set_xlabel('Время (сек)')
        self.ax_orientation.set_ylabel('Угол (градусы)')
        self.ax_orientation.grid(True)
        self.line_roll, = self.ax_orientation.plot([], [], 'r-', label='Roll', linewidth=2)
        self.line_pitch, = self.ax_orientation.plot([], [], 'g-', label='Pitch', linewidth=2)
        self.line_yaw, = self.ax_orientation.plot([], [], 'b-', label='Yaw', linewidth=2)
        self.ax_orientation.legend(loc='upper right')
        
        # Графики акселерометра
        self.ax_accel = plt.subplot(2, 1, 2)
        self.ax_accel.set_title('Линейное ускорение')
        self.ax_accel.set_xlabel('Время (сек)')
        self.ax_accel.set_ylabel('Ускорение (м/с²)')
        self.ax_accel.grid(True)
        self.line_accel_x, = self.ax_accel.plot([], [], 'r-', label='X', linewidth=2)
        self.line_accel_y, = self.ax_accel.plot([], [], 'g-', label='Y', linewidth=2)
        self.line_accel_z, = self.ax_accel.plot([], [], 'b-', label='Z', linewidth=2)
        self.ax_accel.legend(loc='upper right')
        
        # Окно 2: 3D визуализация положения
        self.fig2 = plt.figure(figsize=(10, 8))
        self.ax_3d = self.fig2.add_subplot(111, projection='3d')
        self.ax_3d.set_title('Положение робота в пространстве (ИНС)', fontsize=14, fontweight='bold')
        self.ax_3d.set_xlabel('X (м)')
        self.ax_3d.set_ylabel('Y (м)')
        self.ax_3d.set_zlabel('Z (м)')
        
        # Линия траектории
        self.line_3d, = self.ax_3d.plot([], [], [], 'b-', linewidth=2, label='Траектория')
        
        # Текущая позиция с ориентацией
        self.robot_marker = None
        self.orientation_arrows = []  # Список для хранения стрелок ориентации
        
        self.ax_3d.legend()
        self.ax_3d.grid(True)
        
        # Настройка равных осей для 3D
        self.ax_3d.set_box_aspect([1, 1, 1])
        
        # Анимация
        self.ani1 = FuncAnimation(self.fig1, self.update_plot1, interval=50, blit=False)
        self.ani2 = FuncAnimation(self.fig2, self.update_plot2, interval=50, blit=False)
        
        plt.tight_layout()
    
    def update_plot1(self, frame):
        """Обновление первого окна (графики)"""
        with self.data_lock:
            if len(self.time_data) == 0:
                return
            
            time_array = np.array(self.time_data)
            
            # Обновление графиков ориентации
            if len(self.orientation_euler['roll']) > 0:
                self.line_roll.set_data(time_array, list(self.orientation_euler['roll']))
                self.line_pitch.set_data(time_array, list(self.orientation_euler['pitch']))
                self.line_yaw.set_data(time_array, list(self.orientation_euler['yaw']))
                
                self.ax_orientation.relim()
                self.ax_orientation.autoscale_view()
            
            # Обновление графиков акселерометра
            if len(self.linear_acceleration['x']) > 0:
                self.line_accel_x.set_data(time_array, list(self.linear_acceleration['x']))
                self.line_accel_y.set_data(time_array, list(self.linear_acceleration['y']))
                self.line_accel_z.set_data(time_array, list(self.linear_acceleration['z']))
                
                self.ax_accel.relim()
                self.ax_accel.autoscale_view()
    
    def update_plot2(self, frame):
        """Обновление второго окна (3D визуализация)"""
        with self.data_lock:
            if len(self.position['x']) == 0:
                return
            
            x_array = np.array(self.position['x'])
            y_array = np.array(self.position['y'])
            z_array = np.array(self.position['z'])
            
            # Обновление траектории
            self.line_3d.set_data(x_array, y_array)
            self.line_3d.set_3d_properties(z_array)
            
            # Обновление маркера текущей позиции с ориентацией
            if len(x_array) > 0:
                current_x = x_array[-1]
                current_y = y_array[-1]
                current_z = z_array[-1]
                
                # Удаляем старые маркеры и стрелки
                if self.robot_marker is not None:
                    self.robot_marker.remove()
                    self.robot_marker = None
                
                # Удаляем старые стрелки ориентации
                for arrow in self.orientation_arrows:
                    arrow.remove()
                self.orientation_arrows.clear()
                
                # Получаем текущую ориентацию
                if (len(self.orientation_quat['x']) > 0 and 
                    len(self.orientation_quat['y']) > 0 and
                    len(self.orientation_quat['z']) > 0 and
                    len(self.orientation_quat['w']) > 0):
                    
                    quat = [self.orientation_quat['x'][-1],
                           self.orientation_quat['y'][-1],
                           self.orientation_quat['z'][-1],
                           self.orientation_quat['w'][-1]]
                    
                    # Безопасное преобразование quaternion
                    r = safe_quat_to_rotation(quat)
                    if r is not None:
                        try:
                            # Создаем стрелки для осей ориентации
                            axis_length = 0.1
                            x_axis = r.apply([axis_length, 0, 0])
                            y_axis = r.apply([0, axis_length, 0])
                            z_axis = r.apply([0, 0, axis_length])
                            
                            # Рисуем оси ориентации и сохраняем ссылки
                            arrow_x = self.ax_3d.quiver(current_x, current_y, current_z,
                                                        x_axis[0], x_axis[1], x_axis[2],
                                                        color='r', arrow_length_ratio=0.3, linewidth=2)
                            arrow_y = self.ax_3d.quiver(current_x, current_y, current_z,
                                                        y_axis[0], y_axis[1], y_axis[2],
                                                        color='g', arrow_length_ratio=0.3, linewidth=2)
                            arrow_z = self.ax_3d.quiver(current_x, current_y, current_z,
                                                        z_axis[0], z_axis[1], z_axis[2],
                                                        color='b', arrow_length_ratio=0.3, linewidth=2)
                            
                            self.orientation_arrows.extend([arrow_x, arrow_y, arrow_z])
                        except:
                            pass
                
                # Точка текущей позиции
                self.robot_marker = self.ax_3d.scatter([current_x], [current_y], [current_z],
                                                       c='red', s=100, marker='o')
            
            # Автоматическое масштабирование
            if len(x_array) > 1:
                margin = 0.1
                x_range = [x_array.min() - margin, x_array.max() + margin]
                y_range = [y_array.min() - margin, y_array.max() + margin]
                z_range = [z_array.min() - margin, z_array.max() + margin]
                
                self.ax_3d.set_xlim(x_range)
                self.ax_3d.set_ylim(y_range)
                self.ax_3d.set_zlim(z_range)
    
    def switch_topic(self, new_topic_name):
        """Переключение на другой топик IMU"""
        with self.data_lock:
            if self.imu_topic is not None:
                self.imu_topic.unsubscribe()
            print(f"Переключение на топик {new_topic_name}...")
            self.topic_name = new_topic_name
            self.imu_topic = roslibpy.Topic(self.ros_client, self.topic_name, 'sensor_msgs/Imu')
            self.imu_topic.subscribe(self.imu_callback)
            # Сбрасываем калибровку при переключении топика
            self.calibration_complete = False
            self.calibration_data_accel = []
            self.calibration_data_gyro = []
            self.calibration_data_orientation = []
            self.orientation_reference = None
            print(f"✓ Переключено на топик {new_topic_name}")
    
    def update_parameters(self, friction_coefficient=None, accel_threshold=None, velocity_threshold=None):
        """Обновление параметров на лету"""
        if friction_coefficient is not None:
            self.friction_coefficient = friction_coefficient
        if accel_threshold is not None:
            self.accel_threshold = accel_threshold
        if velocity_threshold is not None:
            self.velocity_threshold = velocity_threshold
    
    def recalibrate(self):
        """Повторная калибровка"""
        with self.data_lock:
            self.calibration_complete = False
            self.calibration_data_accel = []
            self.calibration_data_gyro = []
            self.calibration_data_orientation = []
            self.velocity = np.array([0.0, 0.0, 0.0])
            self.current_position = np.array([0.0, 0.0, 0.0])
            self.last_time = None
            self.initial_orientation = None
            self.orientation_reference = None
            print("Калибровка сброшена. Начинается новая калибровка...")
    
    def run(self):
        """Запуск визуализации"""
        try:
            plt.show(block=True)
        except KeyboardInterrupt:
            print("\nОстановка визуализатора...")
        finally:
            self.running = False
            # Закрытие соединения с ROS Bridge
            if self.ros_client.is_connected:
                self.imu_topic.unsubscribe()
                self.ros_client.terminate()
                print("Соединение с ROS Bridge закрыто")


class IMUSettingsGUI:
    """GUI окно для настройки параметров IMU визуализатора"""
    def __init__(self, visualizer):
        self.visualizer = visualizer
        self.root = tk.Tk()
        self.root.title("Настройки IMU Визуализатора")
        self.root.geometry("500x600")
        
        # Переменные для значений
        self.topic_var = tk.StringVar(value=visualizer.topic_name)
        self.friction_var = tk.DoubleVar(value=visualizer.friction_coefficient)
        self.accel_threshold_var = tk.DoubleVar(value=visualizer.accel_threshold)
        self.velocity_threshold_var = tk.DoubleVar(value=visualizer.velocity_threshold)
        self.calibration_samples_var = tk.IntVar(value=visualizer.calibration_samples)
        
        # Информация о калибровке
        self.calibration_info = tk.StringVar(value="Калибровка не выполнена")
        
        self.create_widgets()
        self.update_calibration_info()
        
        # Обновление информации о калибровке каждую секунду
        self.root.after(1000, self.periodic_update)
    
    def create_widgets(self):
        """Создание виджетов GUI"""
        # Заголовок
        title_label = tk.Label(self.root, text="Настройки IMU", font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # Выбор топика
        topic_frame = tk.LabelFrame(self.root, text="Выбор топика IMU", padx=10, pady=10)
        topic_frame.pack(fill=tk.X, padx=10, pady=5)
        
        topics = ['/imu', '/imu_corrected', '/ros_robot_controller/imu_raw']
        topic_combo = ttk.Combobox(topic_frame, textvariable=self.topic_var, values=topics, state="readonly", width=40)
        topic_combo.pack(pady=5)
        topic_combo.bind('<<ComboboxSelected>>', self.on_topic_change)
        
        # Параметры калибровки
        calib_frame = tk.LabelFrame(self.root, text="Параметры калибровки", padx=10, pady=10)
        calib_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(calib_frame, text="Количество измерений для калибровки:").pack(anchor=tk.W)
        calib_spin = tk.Spinbox(calib_frame, from_=5, to=50, textvariable=self.calibration_samples_var, width=10)
        calib_spin.pack(anchor=tk.W, pady=5)
        
        recalib_btn = tk.Button(calib_frame, text="Повторная калибровка", command=self.on_recalibrate, bg="#4CAF50", fg="white")
        recalib_btn.pack(pady=5)
        
        # Информация о калибровке
        info_frame = tk.LabelFrame(self.root, text="Информация о калибровке", padx=10, pady=10)
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.info_label = tk.Label(info_frame, textvariable=self.calibration_info, justify=tk.LEFT, wraplength=450)
        self.info_label.pack(anchor=tk.W, fill=tk.BOTH, expand=True)
        
        # Параметры фильтрации
        filter_frame = tk.LabelFrame(self.root, text="Параметры фильтрации", padx=10, pady=10)
        filter_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Коэффициент трения
        tk.Label(filter_frame, text="Коэффициент трения (0.0-1.0):").pack(anchor=tk.W)
        friction_scale = tk.Scale(filter_frame, from_=0.8, to=0.99, resolution=0.01, 
                                  orient=tk.HORIZONTAL, variable=self.friction_var, 
                                  command=self.on_friction_change)
        friction_scale.pack(fill=tk.X, pady=5)
        friction_value_label = tk.Label(filter_frame, text=f"Текущее: {self.friction_var.get():.2f}")
        friction_value_label.pack(anchor=tk.W)
        
        def update_friction_label(val):
            friction_value_label.config(text=f"Текущее: {float(val):.2f}")
        friction_scale.config(command=lambda v: (update_friction_label(v), self.on_friction_change(v)))
        
        # Порог ускорения
        tk.Label(filter_frame, text="Порог ускорения для покоя (м/с²):").pack(anchor=tk.W)
        accel_scale = tk.Scale(filter_frame, from_=0.05, to=0.5, resolution=0.01,
                              orient=tk.HORIZONTAL, variable=self.accel_threshold_var,
                              command=self.on_accel_threshold_change)
        accel_scale.pack(fill=tk.X, pady=5)
        
        # Порог скорости
        tk.Label(filter_frame, text="Порог скорости для покоя (м/с):").pack(anchor=tk.W)
        velocity_scale = tk.Scale(filter_frame, from_=0.01, to=0.1, resolution=0.001,
                                 orient=tk.HORIZONTAL, variable=self.velocity_threshold_var,
                                 command=self.on_velocity_threshold_change)
        velocity_scale.pack(fill=tk.X, pady=5)
        
        # Кнопка закрытия
        close_btn = tk.Button(self.root, text="Закрыть", command=self.root.destroy, bg="#f44336", fg="white")
        close_btn.pack(pady=10)
    
    def on_topic_change(self, event=None):
        """Обработчик изменения топика"""
        new_topic = self.topic_var.get()
        if new_topic != self.visualizer.topic_name:
            self.visualizer.switch_topic(new_topic)
            self.update_calibration_info()
    
    def on_friction_change(self, value):
        """Обработчик изменения коэффициента трения"""
        self.visualizer.update_parameters(friction_coefficient=float(value))
    
    def on_accel_threshold_change(self, value):
        """Обработчик изменения порога ускорения"""
        self.visualizer.update_parameters(accel_threshold=float(value))
    
    def on_velocity_threshold_change(self, value):
        """Обработчик изменения порога скорости"""
        self.visualizer.update_parameters(velocity_threshold=float(value))
    
    def on_recalibrate(self):
        """Обработчик повторной калибровки"""
        self.visualizer.calibration_samples = self.calibration_samples_var.get()
        self.visualizer.recalibrate()
        self.update_calibration_info()
    
    def update_calibration_info(self):
        """Обновление информации о калибровке"""
        with self.visualizer.data_lock:
            if self.visualizer.calibration_complete:
                bias = self.visualizer.accel_bias
                bias_mag = np.linalg.norm(bias)
                info = f"✓ Калибровка выполнена\n"
                info += f"Смещение акселерометра:\n"
                info += f"  X: {bias[0]:.4f} м/с²\n"
                info += f"  Y: {bias[1]:.4f} м/с²\n"
                info += f"  Z: {bias[2]:.4f} м/с²\n"
                info += f"Величина: {bias_mag:.4f} м/с²\n\n"
                info += f"Смещение гироскопа:\n"
                gyro = self.visualizer.gyro_bias
                info += f"  X: {gyro[0]:.4f} рад/с\n"
                info += f"  Y: {gyro[1]:.4f} рад/с\n"
                info += f"  Z: {gyro[2]:.4f} рад/с"
            else:
                samples = len(self.visualizer.calibration_data_accel)
                total = self.visualizer.calibration_samples
                info = f"⏳ Калибровка в процессе...\n"
                info += f"Собрано измерений: {samples}/{total}\n"
                info += f"Убедитесь, что робот стоит на месте!"
            self.calibration_info.set(info)
    
    def periodic_update(self):
        """Периодическое обновление информации"""
        self.update_calibration_info()
        self.root.after(1000, self.periodic_update)
    
    def run(self):
        """Запуск GUI"""
        self.root.mainloop()


def main():
    """Главная функция"""
    # Парсинг аргументов командной строки
    parser = argparse.ArgumentParser(description='Визуализатор данных IMU через ROS Bridge')
    parser.add_argument('--host', default='localhost', 
                       help='IP адрес ROS Bridge сервера (по умолчанию: localhost)')
    parser.add_argument('--port', type=int, default=9090, 
                       help='Порт ROS Bridge сервера (по умолчанию: 9090)')
    parser.add_argument('--calibration-samples', type=int, default=10,
                       help='Количество измерений для калибровки IMU в покое (по умолчанию: 10)')
    parser.add_argument('--friction', type=float, default=0.95,
                       help='Коэффициент трения для затухания скорости (0.0-1.0, по умолчанию: 0.95)')
    parser.add_argument('--accel-threshold', type=float, default=0.15,
                       help='Порог ускорения для определения покоя в м/с² (по умолчанию: 0.15)')
    parser.add_argument('--velocity-threshold', type=float, default=0.03,
                       help='Порог скорости для определения покоя в м/с (по умолчанию: 0.03)')
    parser.add_argument('--topic', default='/imu',
                       help='Название топика IMU (по умолчанию: /imu)')
    parser.add_argument('--no-gui', action='store_true',
                       help='Отключить GUI окно настроек')
    
    args = parser.parse_args()
    
    print(f"Подключение к ROS Bridge: {args.host}:{args.port}")
    print("Для указания другого адреса: python imu_visualizer.py --host IP --port 9090")
    print(f"Топик: {args.topic}")
    print(f"Калибровка будет выполнена по {args.calibration_samples} первым измерениям")
    print(f"Коэффициент трения: {args.friction}, Пороги: ускорение={args.accel_threshold} м/с², скорость={args.velocity_threshold} м/с")
    
    # Создание визуализатора
    try:
        visualizer = IMUVisualizer(ros_host=args.host, ros_port=args.port, 
                                   calibration_samples=args.calibration_samples,
                                   friction_coefficient=args.friction,
                                   accel_threshold=args.accel_threshold,
                                   velocity_threshold=args.velocity_threshold,
                                   topic_name=args.topic)
        
        # Запуск GUI в отдельном потоке, если доступен
        if TKINTER_AVAILABLE and not args.no_gui:
            def run_gui():
                gui = IMUSettingsGUI(visualizer)
                gui.run()
            
            gui_thread = threading.Thread(target=run_gui, daemon=True)
            gui_thread.start()
            print("GUI окно настроек запущено")
        
        visualizer.run()
    except KeyboardInterrupt:
        print("\nОстановка визуализатора...")
    except Exception as e:
        print(f"Ошибка: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

