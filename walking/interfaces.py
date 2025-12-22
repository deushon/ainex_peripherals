#!/usr/bin/env python3
# encoding: utf-8
"""
Интерфейсы данных для модуля стабилизации.
Определяет четкую структуру данных для обмена между модулями.
"""

from dataclasses import dataclass
from typing import Optional, Dict, Any, List


@dataclass
class IMUData:
    """Структура данных IMU."""
    orientation: Dict[str, float]  # {'roll': float, 'pitch': float, 'yaw': float} в градусах
    angular_velocity: Dict[str, float]  # {'x': float, 'y': float, 'z': float} в rad/s
    linear_acceleration: Dict[str, float]  # {'x': float, 'y': float, 'z': float} в m/s²


@dataclass
class ThrottleData:
    """Структура данных управления от джойстика."""
    x: float  # Амплитуда движения по X
    y: float  # Амплитуда движения по Y
    angle: float  # Амплитуда поворота


@dataclass
class WalkingPeriodParams:
    """Параметры периода ходьбы."""
    period_ms: int  # Период шага в миллисекундах
    dsp_ratio: float  # Доля двойной опоры (0.0-1.0)
    y_swap_amplitude: float  # Амплитуда обмена по Y


@dataclass
class RobotPoseParams:
    """Параметры позы робота (смещения)."""
    init_x_offset: float = 0.0  # Смещение по X
    init_y_offset: float = 0.0  # Смещение по Y
    init_roll_offset: float = 0.0  # Смещение по крену (градусы)
    init_pitch_offset: float = 0.0  # Смещение по тангажу (градусы)


@dataclass
class GaitBaseParams:
    """Базовые параметры походки."""
    body_height: float
    step_fb_ratio: float
    z_swap_amplitude: float
    hip_pitch_offset: float  # градусы
    pelvis_offset: float  # градусы


@dataclass
class WalkingParams:
    """Полные параметры ходьбы."""
    period: WalkingPeriodParams
    pose: RobotPoseParams
    gait_base: GaitBaseParams
    move_amplitudes: Dict[str, float]  # {'x': float, 'y': float, 'angle': float}
    extra_params: Dict[str, Any]  # Дополнительные параметры из gait_param


@dataclass
class StabilizationResult:
    """Результат работы модуля стабилизации."""
    period_override: Optional[WalkingPeriodParams] = None  # Переопределение периода (None = использовать из конфига)
    pose_override: Optional[RobotPoseParams] = None  # Переопределение позы (None = использовать из конфига)
    gait_base_override: Optional[GaitBaseParams] = None  # Переопределение базовых параметров (None = использовать из конфига)
    modified: bool = False  # Флаг, что были внесены изменения


