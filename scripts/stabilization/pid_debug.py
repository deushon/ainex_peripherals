#!/usr/bin/env python3
# encoding: utf-8
"""
Публикация отладочных данных PID регулятора.
"""

import rospy
from std_msgs.msg import Header
from ainex_peripherals.msg import PidDebug


class PidDebugPublisher:
    """Публикует отладочные данные PID регулятора."""
    
    def __init__(self):
        """Инициализирует публикатор отладочных данных."""
        self.pub = rospy.Publisher('/stabilization/pid_debug', PidDebug, queue_size=10)
        self.last_debug_data = None
    
    def publish(self, imu_data, is_walking, pid_state, pid_outputs, applied_corrections, config):
        """
        Публикует отладочные данные PID регулятора.
        
        Args:
            imu_data: Словарь с данными IMU
            is_walking: True если робот идет, False если в покое
            pid_state: Состояние PID контроллера
            pid_outputs: Словарь с выходами PID {'roll': {...}, 'pitch': {...}}
            applied_corrections: Словарь с примененными корректировками
            config: Конфигурация PID
        """
        try:
            msg = PidDebug()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"
            
            # Ориентация робота
            orientation = imu_data.get('orientation', {})
            msg.roll_deg = orientation.get('roll_deg', 0)
            msg.pitch_deg = orientation.get('pitch_deg', 0)
            msg.yaw_deg = orientation.get('yaw_deg', 0)
            
            # Угловые скорости
            angular_velocity = imu_data.get('angular_velocity', {})
            msg.roll_velocity = angular_velocity.get('x', 0)
            msg.pitch_velocity = angular_velocity.get('y', 0)
            msg.yaw_velocity = angular_velocity.get('z', 0)
            
            # Линейные ускорения
            linear_accel = imu_data.get('linear_acceleration', {})
            msg.linear_accel_x = linear_accel.get('x', 0)
            msg.linear_accel_y = linear_accel.get('y', 0)
            msg.linear_accel_z = linear_accel.get('z', 0)
            
            # Текущие значения позы (нужно получить из gait_manager, но здесь передаем через applied_corrections)
            msg.init_x_offset = applied_corrections.get('current_x', 0)
            msg.init_y_offset = applied_corrections.get('current_y', 0)
            msg.init_roll_offset = applied_corrections.get('current_roll', 0)
            msg.init_pitch_offset = applied_corrections.get('current_pitch', 0)
            
            # Управляющие воздействия от PID
            roll_output = pid_outputs.get('roll', {})
            pitch_output = pid_outputs.get('pitch', {})
            
            msg.roll_error = roll_output.get('error', 0)
            msg.pitch_error = pitch_output.get('error', 0)
            
            msg.roll_p_term = roll_output.get('p_term', 0)
            msg.roll_i_term = roll_output.get('i_term', 0)
            msg.roll_d_term = roll_output.get('d_term', 0)
            msg.roll_pid_output = roll_output.get('output', 0)
            
            msg.pitch_p_term = pitch_output.get('p_term', 0)
            msg.pitch_i_term = pitch_output.get('i_term', 0)
            msg.pitch_d_term = pitch_output.get('d_term', 0)
            msg.pitch_pid_output = pitch_output.get('output', 0)
            
            # Применяемые корректировки
            msg.applied_x_correction = applied_corrections.get('x', 0)
            msg.applied_y_correction = applied_corrections.get('y', 0)
            msg.applied_roll_correction = applied_corrections.get('roll', 0)
            msg.applied_pitch_correction = applied_corrections.get('pitch', 0)
            
            # Состояние регулятора
            msg.pid_enabled = config.get('pid_enabled', False)
            msg.roll_enabled = config.get('roll_enabled', False)
            msg.pitch_enabled = config.get('pitch_enabled', False)
            msg.is_walking = is_walking
            
            # Интегральные составляющие
            integral = pid_state.get('integral', {})
            msg.roll_integral = integral.get('roll', 0)
            msg.pitch_integral = integral.get('pitch', 0)
            
            # Время с последнего обновления
            last_time = pid_state.get('last_time')
            current_time = rospy.get_time()
            msg.dt = current_time - last_time if last_time else 0
            
            self.pub.publish(msg)
            self.last_debug_data = msg
        except Exception as e:
            rospy.logwarn(f"Error publishing PID debug data: {e}")

