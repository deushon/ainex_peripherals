#!/usr/bin/env python3
# encoding: utf-8
"""
Автоматический подъем робота после падения.
"""

import rospy
from config import ConfigLoader


class AutoGetup:
    """
    Автоматический подъем робота после падения.
    """
    
    def __init__(self, config_loader=None):
        """
        Инициализация автоматического подъема.
        
        Args:
            config_loader: Экземпляр ConfigLoader (опционально)
        """
        self.config = config_loader if config_loader else ConfigLoader()
        imu_config = self.config.load('imu')
        auto_getup_config = imu_config.get('imu', {}).get('auto_getup', {})
        
        self.enabled = auto_getup_config.get('enabled', True)
        self.fall_time_threshold = auto_getup_config.get('fall_time_threshold', 2.0)
        self.interval = auto_getup_config.get('interval', 4.0)
        self.max_attempts = auto_getup_config.get('max_attempts', 2)
        self.action_timeout = auto_getup_config.get('action_timeout', 10.0)
        
        actions_config = auto_getup_config.get('actions', {})
        self.actions = {
            'fall_forward': actions_config.get('fall_forward', 'lie_to_stand'),
            'fall_backward': actions_config.get('fall_backward', 'BACK_UP'),
            'fall_left': actions_config.get('fall_left', 'LEFT_UP'),
            'fall_right': actions_config.get('fall_right', 'RIGHT_UP')
        }
        
        self.fall_start_time = None
        self.last_attempt_time = 0.0
        self.action_in_progress = False
        self.attempt_count = 0
    
    def check(self, robot_state, motion_manager, can_move_func, current_time, fall_check_cooldown=0.0):
        """
        Проверяет необходимость автоматического подъема.
        
        Args:
            robot_state: Текущее состояние робота
            motion_manager: Экземпляр MotionManager
            can_move_func: Функция проверки разрешения на движение
            current_time: Текущее время
            fall_check_cooldown: Время cooldown после подъема
        
        Returns:
            bool: True если была выполнена попытка подъема
        """
        if not self.enabled:
            return False
        
        if current_time < fall_check_cooldown:
            return False
        
        if not can_move_func():
            return False
        
        if robot_state == 'stand':
            self.fall_start_time = None
            self.action_in_progress = False
            return False
        
        if self.fall_start_time is None:
            self.fall_start_time = current_time
            return False
        
        if current_time - self.last_attempt_time < self.interval:
            return False
        
        fall_duration = current_time - self.fall_start_time
        if fall_duration < self.fall_time_threshold:
            return False
        
        if self.attempt_count >= self.max_attempts:
            rospy.logerr("Auto-getup: Max attempts reached")
            return False
        
        if self.action_in_progress:
            if current_time - self.last_attempt_time > self.action_timeout:
                self.action_in_progress = False
                self.attempt_count += 1
            else:
                return False
        
        rospy.logwarn(f"Auto-getup: Robot has been down for {fall_duration:.1f} seconds")
        rospy.logwarn(f"   Attempt {self.attempt_count + 1}/{self.max_attempts}: {robot_state}")
        
        try:
            action_to_run = self.actions.get(robot_state)
            if not action_to_run:
                rospy.logwarn(f"Auto-getup action not configured for state: {robot_state}")
                return False
            
            if motion_manager is not None:
                self.action_in_progress = True
                self.attempt_count += 1
                rospy.loginfo(f"Executing auto-getup action: {action_to_run}")
                motion_manager.run_action(action_to_run)
                self.last_attempt_time = current_time
                return True
        except Exception as e:
            rospy.logerr(f"Error in auto-getup: {e}")
            self.action_in_progress = False
        
        return False
    
    def reset(self):
        """Сбрасывает состояние автоматического подъема."""
        self.fall_start_time = None
        self.last_attempt_time = 0.0
        self.action_in_progress = False
        self.attempt_count = 0

