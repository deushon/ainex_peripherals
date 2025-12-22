#!/usr/bin/env python3
# encoding: utf-8
"""
Управление состоянием робота.
Централизованное место для доступа к состоянию робота во всем пакете.
"""

import rospy


class RobotStateManager:
    """
    Менеджер состояния робота.
    Централизованное хранилище состояния робота.
    """
    
    STATE_STAND = 'stand'
    STATE_MOVING = 'moving'
    STATE_FALL_FORWARD = 'fall_forward'
    STATE_FALL_BACKWARD = 'fall_backward'
    STATE_FALL_LEFT = 'fall_left'
    STATE_FALL_RIGHT = 'fall_right'
    STATE_LIE_TO_STAND = 'lie_to_stand'
    STATE_RECLINE_TO_STAND = 'recline_to_stand'
    
    def __init__(self):
        """Инициализация менеджера состояния."""
        self._state = self.STATE_STAND
        self._previous_state = self.STATE_STAND
    
    def get_state(self):
        """
        Возвращает текущее состояние робота.
        
        Returns:
            str: Текущее состояние
        """
        return self._state
    
    def set_state(self, new_state):
        """
        Устанавливает новое состояние робота.
        
        Args:
            new_state: Новое состояние робота
        """
        if new_state != self._state:
            self._previous_state = self._state
            self._state = new_state
            rospy.loginfo(f"Robot state changed: '{self._previous_state}' -> '{self._state}'")
    
    def get_previous_state(self):
        """
        Возвращает предыдущее состояние робота.
        
        Returns:
            str: Предыдущее состояние
        """
        return self._previous_state
    
    def is_fallen(self):
        """
        Проверяет, упал ли робот.
        
        Returns:
            bool: True если робот упал
        """
        return self._state in [
            self.STATE_FALL_FORWARD,
            self.STATE_FALL_BACKWARD,
            self.STATE_FALL_LEFT,
            self.STATE_FALL_RIGHT
        ]
    
    def is_standing(self):
        """
        Проверяет, стоит ли робот.
        
        Returns:
            bool: True если робот стоит
        """
        return self._state == self.STATE_STAND
    
    def is_moving(self):
        """
        Проверяет, движется ли робот.
        
        Returns:
            bool: True если робот движется
        """
        return self._state == self.STATE_MOVING

