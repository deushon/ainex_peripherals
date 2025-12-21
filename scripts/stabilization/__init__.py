"""
Модуль стабилизации корпуса робота через PID регулятор.
Используется для удержания вертикального положения как в покое, так и при ходьбе.
"""

from .pid_config import UNIFIED_PID_CONFIG
from .pid_controller import PidController
from .pid_debug import PidDebugPublisher

__all__ = ['UNIFIED_PID_CONFIG', 'PidController', 'PidDebugPublisher']

