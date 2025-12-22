#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль управления состоянием робота.
"""

from robot_state.manager import RobotStateManager
from robot_state.detector import FallDetector
from robot_state.auto_getup import AutoGetup

__all__ = ['RobotStateManager', 'FallDetector', 'AutoGetup']

