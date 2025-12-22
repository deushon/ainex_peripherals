#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль стабилизации - единственное место переопределения параметров ходьбы.
Принимает данные IMU и может переопределить параметры позы и ходьбы.
Если не переопределяет - используются значения из конфигов.
"""

from typing import Optional
from walking.interfaces import (
    IMUData,
    ThrottleData,
    WalkingParams,
    StabilizationResult,
    WalkingPeriodParams,
    RobotPoseParams,
    GaitBaseParams
)


class StabilizationModule:
    """
    Модуль стабилизации.
    Единственное место переопределения параметров ходьбы и позы робота.
    """
    
    def __init__(self):
        """Инициализация модуля стабилизации."""
        self.enabled = False
    
    def process(
        self,
        imu_data: IMUData,
        throttle_data: ThrottleData,
        current_walking_params: WalkingParams
    ) -> StabilizationResult:
        """
        Обрабатывает данные IMU и throttle, возвращает переопределенные параметры.
        
        Args:
            imu_data: Данные IMU (ориентация, угловая скорость, ускорение)
            throttle_data: Данные управления от джойстика
            current_walking_params: Текущие параметры ходьбы из конфига
        
        Returns:
            StabilizationResult: Результат стабилизации (переопределения или None для использования конфига)
        """
        if not self.enabled:
            return StabilizationResult(modified=False)
        
        # Здесь будет логика стабилизации пользователя
        # Пока возвращаем исходные параметры (используем конфиг)
        
        result = StabilizationResult(modified=False)
        
        # Пример: если нужно переопределить позу:
        # result.pose_override = RobotPoseParams(
        #     init_roll_offset=some_correction,
        #     init_pitch_offset=another_correction
        # )
        # result.modified = True
        
        # Пример: если нужно переопределить период:
        # result.period_override = WalkingPeriodParams(
        #     period_ms=current_walking_params.period.period_ms,
        #     dsp_ratio=adjusted_dsp_ratio,
        #     y_swap_amplitude=current_walking_params.period.y_swap_amplitude
        # )
        # result.modified = True
        
        return result
    
    def set_enabled(self, enabled: bool):
        """Включает/выключает модуль стабилизации."""
        self.enabled = enabled
    
    def is_enabled(self) -> bool:
        """Возвращает статус модуля стабилизации."""
        return self.enabled
    
    def reset(self):
        """Сбрасывает состояние модуля стабилизации."""
        pass

