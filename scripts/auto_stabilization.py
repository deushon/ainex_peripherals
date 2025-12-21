#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль для получения параметров ориентации и throttle для переопределения параметров ходьбы.
Вся логика стабилизации удалена - пользователь будет писать стабилизацию самостоятельно.
"""

import rospy


class AutoStabilization:
    """Класс для получения параметров ориентации и throttle."""
    
    def __init__(self, gait_manager, speed_params, speed_mode, init_z_offset):
        """
        Инициализирует модуль.
        
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
        
        # Последние данные IMU (для доступа к ориентации)
        self.last_imu_data = None
        
        # Параметры движения от джойстика (для доступа к throttle)
        self.x_move_amplitude = 0.0
        self.y_move_amplitude = 0.0
        self.angle_move_amplitude = 0.0
        
        rospy.loginfo("AutoStabilization module initialized (orientation and throttle data only)")
    
    def process(self, imu_data, robot_state, status, x_move_amp, y_move_amp, angle_move_amp):
        """
        Сохраняет данные IMU и параметры движения для доступа извне.
        
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
        # Сохраняем данные для доступа извне
        self.last_imu_data = imu_data
        self.x_move_amplitude = x_move_amp
        self.y_move_amplitude = y_move_amp
        self.angle_move_amplitude = angle_move_amp
        
        return False  # Не выполняем шаги
    
    def get_orientation(self):
        """
        Возвращает текущую ориентацию робота.
        
        Returns:
            dict: Словарь с ориентацией {'roll': float, 'pitch': float, 'yaw': float} в градусах,
                  или None если данных нет
        """
        if self.last_imu_data is None:
            return None
        
        orientation = self.last_imu_data.get('orientation', {})
        if not orientation:
            return None
        
        return {
            'roll': orientation.get('roll_deg', 0.0),
            'pitch': orientation.get('pitch_deg', 0.0),
            'yaw': orientation.get('yaw_deg', 0.0),
        }
    
    def get_angular_velocity(self):
        """
        Возвращает текущую угловую скорость робота.
        
        Returns:
            dict: Словарь с угловой скоростью {'x': float, 'y': float, 'z': float} в rad/s,
                  или None если данных нет
        """
        if self.last_imu_data is None:
            return None
        
        angular_velocity = self.last_imu_data.get('angular_velocity', {})
        if not angular_velocity:
            return None
        
        return {
            'x': angular_velocity.get('x', 0.0),
            'y': angular_velocity.get('y', 0.0),
            'z': angular_velocity.get('z', 0.0),
        }
    
    def get_linear_acceleration(self):
        """
        Возвращает текущее линейное ускорение робота.
        
        Returns:
            dict: Словарь с линейным ускорением {'x': float, 'y': float, 'z': float} в m/s²,
                  или None если данных нет
        """
        if self.last_imu_data is None:
            return None
        
        linear_acceleration = self.last_imu_data.get('linear_acceleration', {})
        if not linear_acceleration:
            return None
        
        return {
            'x': linear_acceleration.get('x', 0.0),
            'y': linear_acceleration.get('y', 0.0),
            'z': linear_acceleration.get('z', 0.0),
        }
    
    def get_throttle(self):
        """
        Возвращает текущие параметры throttle (управление от джойстика).
        
        Returns:
            dict: Словарь с параметрами throttle {'x': float, 'y': float, 'angle': float}
        """
        return {
            'x': self.x_move_amplitude,
            'y': self.y_move_amplitude,
            'angle': self.angle_move_amplitude,
        }
    
    def get_all_data(self):
        """
        Возвращает все доступные данные для переопределения параметров ходьбы.
        
        Returns:
            dict: Словарь со всеми данными:
                - 'orientation': {'roll': float, 'pitch': float, 'yaw': float} в градусах
                - 'angular_velocity': {'x': float, 'y': float, 'z': float} в rad/s
                - 'linear_acceleration': {'x': float, 'y': float, 'z': float} в m/s²
                - 'throttle': {'x': float, 'y': float, 'angle': float}
                - 'speed_mode': int (режим скорости 1-4)
                - 'speed_params': dict (параметры скорости)
        """
        return {
            'orientation': self.get_orientation(),
            'angular_velocity': self.get_angular_velocity(),
            'linear_acceleration': self.get_linear_acceleration(),
            'throttle': self.get_throttle(),
            'speed_mode': self.speed_mode,
            'speed_params': self.speed_params,
        }
    
    def apply_walking_corrections(self, gait_param, period_time):
        """
        Заглушка для совместимости. Не выполняет никаких корректировок.
        
        Args:
            gait_param: Словарь параметров походки
            period_time: Список [period_time, dsp_ratio, y_swap_amplitude]
        
        Returns:
            bool: False (корректировки не применяются)
        """
        return False
    
    def reset_walking_corrections(self):
        """Заглушка для совместимости. Не выполняет никаких действий."""
        pass
    
    def reset(self):
        """Заглушка для совместимости. Не выполняет никаких действий."""
        pass
    
    def set_walking_stabilization_enabled(self, enabled):
        """Заглушка для совместимости. Не выполняет никаких действий."""
        pass
    
    def set_rest_stabilization_enabled(self, enabled):
        """Заглушка для совместимости. Не выполняет никаких действий."""
        pass
