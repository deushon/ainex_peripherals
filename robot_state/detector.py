#!/usr/bin/env python3
# encoding: utf-8
"""
Обнаружение падений и состояний робота на основе данных IMU.
"""

import rospy
from collections import deque
from config import ConfigLoader


class FallDetector:
    """
    Обнаружение падений на основе ориентации робота.
    """
    
    def __init__(self, config_loader=None):
        """
        Инициализация детектора падений.
        
        Args:
            config_loader: Экземпляр ConfigLoader (опционально)
        """
        self.config = config_loader if config_loader else ConfigLoader()
        imu_config = self.config.load('imu')
        fall_config = imu_config.get('imu', {}).get('fall_detection', {})
        
        self.critical_angle_roll = fall_config.get('critical_angle_roll', 45.0)
        self.critical_angle_pitch = fall_config.get('critical_angle_pitch', 45.0)
        self.critical_angle_duration = fall_config.get('critical_angle_duration', 0.5)
        self.check_rest_enabled = fall_config.get('check_rest_enabled', True)
        self.rest_accel_threshold = fall_config.get('rest_accel_threshold', 0.5)
        self.rest_check_duration = fall_config.get('rest_check_duration', 0.3)
        self.max_history_size = fall_config.get('max_history_size', 100)
        self.sample_rate = fall_config.get('sample_rate', 50.0)
        
        self.critical_angle_start_time = None
        self.critical_angle_axis = None
        
        self.orientation_history = {
            'roll': deque(maxlen=self.max_history_size),
            'pitch': deque(maxlen=self.max_history_size)
        }
        
        self.accel_history = {
            'x': deque(maxlen=self.max_history_size),
            'y': deque(maxlen=self.max_history_size),
            'z': deque(maxlen=self.max_history_size)
        }
    
    def detect(self, roll_deg, pitch_deg, ax, ay, az, current_time, fall_check_cooldown=0.0):
        """
        Обнаруживает падение на основе ориентации.
        
        Args:
            roll_deg: Угол крена в градусах
            pitch_deg: Угол тангажа в градусах
            ax, ay, az: Линейные ускорения
            current_time: Текущее время
            fall_check_cooldown: Время cooldown после подъема
        
        Returns:
            str: Состояние робота ('stand', 'fall_forward', 'fall_backward', 'fall_left', 'fall_right')
        """
        if current_time < fall_check_cooldown:
            return 'stand'
        
        self.orientation_history['roll'].append(roll_deg)
        self.orientation_history['pitch'].append(pitch_deg)
        self.accel_history['x'].append(ax)
        self.accel_history['y'].append(ay)
        self.accel_history['z'].append(az)
        
        roll_exceeded = abs(roll_deg) > self.critical_angle_roll
        pitch_deviation_from_90 = abs(pitch_deg - 90.0)
        pitch_exceeded = pitch_deviation_from_90 > self.critical_angle_pitch
        
        critical_angle_exceeded = roll_exceeded or pitch_exceeded
        
        exceeded_axis = None
        if roll_exceeded:
            exceeded_axis = 'roll'
        elif pitch_exceeded:
            exceeded_axis = 'pitch'
        
        is_at_rest = True
        if self.check_rest_enabled:
            is_at_rest = self._check_at_rest()
        
        if critical_angle_exceeded and is_at_rest:
            if self.critical_angle_start_time is None:
                self.critical_angle_start_time = current_time
                self.critical_angle_axis = exceeded_axis
            elif self.critical_angle_axis != exceeded_axis:
                self.critical_angle_start_time = current_time
                self.critical_angle_axis = exceeded_axis
        else:
            self.critical_angle_start_time = None
            self.critical_angle_axis = None
        
        if (critical_angle_exceeded and 
            is_at_rest and 
            self.critical_angle_start_time is not None and
            (current_time - self.critical_angle_start_time) >= self.critical_angle_duration):
            
            if exceeded_axis == 'roll':
                return 'fall_left' if roll_deg > 0 else 'fall_right'
            elif exceeded_axis == 'pitch':
                pitch_normalized = pitch_deg % 360.0
                if pitch_normalized < 0:
                    pitch_normalized += 360.0
                
                if (0 <= pitch_normalized < 90) or (270 < pitch_normalized < 360):
                    return 'fall_forward'
                else:
                    return 'fall_backward'
        
        return 'stand'
    
    def _check_at_rest(self):
        """Проверяет, находится ли робот в покое."""
        if len(self.accel_history['x']) < 2:
            return False
        
        num_samples = min(int(self.rest_check_duration * self.sample_rate), len(self.accel_history['x']))
        
        if num_samples < 2:
            return False
        
        max_change = 0.0
        accel_x = list(self.accel_history['x'])
        accel_y = list(self.accel_history['y'])
        accel_z = list(self.accel_history['z'])
        
        start_idx = len(accel_x) - num_samples
        for i in range(start_idx, len(accel_x) - 1):
            change_x = abs(accel_x[i+1] - accel_x[i])
            change_y = abs(accel_y[i+1] - accel_y[i])
            change_z = abs(accel_z[i+1] - accel_z[i])
            max_change = max(max_change, change_x, change_y, change_z)
        
        return max_change < self.rest_accel_threshold

