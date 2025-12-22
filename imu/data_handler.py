#!/usr/bin/env python3
# encoding: utf-8
"""
Простой обработчик данных IMU.
Только преобразование данных, без логики состояний.
"""

import rospy
import math
from sensor_msgs.msg import Imu


class IMUDataHandler:
    """
    Простой обработчик данных IMU.
    Преобразует сырые данные IMU в структурированный формат.
    """
    
    def __init__(self):
        """Инициализация обработчика данных IMU."""
        self.data_received = False
    
    def process(self, msg: Imu):
        """
        Обрабатывает сообщение IMU и возвращает структурированные данные.
        
        Args:
            msg: Сообщение IMU из ROS
        
        Returns:
            dict: Словарь с обработанными данными или None при ошибке
        """
        try:
            qx_raw = msg.orientation.x
            qy_raw = msg.orientation.y
            qz_raw = msg.orientation.z
            qw_raw = msg.orientation.w
            
            roll_raw, pitch_raw, yaw = self._quaternion_to_euler(qx_raw, qy_raw, qz_raw, qw_raw)
            
            roll = pitch_raw
            pitch = roll_raw
            
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            gx_raw = msg.angular_velocity.x
            gy_raw = msg.angular_velocity.y
            gz_raw = msg.angular_velocity.z
            
            gx = gy_raw
            gy = gx_raw
            gz = gz_raw
            
            ax_raw = msg.linear_acceleration.x
            ay_raw = msg.linear_acceleration.y
            az_raw = msg.linear_acceleration.z
            
            ax = ay_raw
            ay = ax_raw
            az = az_raw
            
            if not self.data_received:
                self.data_received = True
                rospy.loginfo("IMU data received")
            
            return {
                'orientation': {
                    'x': qx_raw,
                    'y': qy_raw,
                    'z': qz_raw,
                    'w': qw_raw,
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
                }
            }
        except Exception as e:
            rospy.logwarn(f"Error processing IMU data: {e}")
            return None
    
    def _quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Преобразует quaternion в углы Эйлера.
        
        Args:
            qx, qy, qz, qw: Компоненты quaternion
        
        Returns:
            tuple: (roll, pitch, yaw) в радианах
        """
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll_raw = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch_raw = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch_raw = math.asin(sinp)
        
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll_raw, pitch_raw, yaw

