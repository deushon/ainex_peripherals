#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль стабилизации - единственное место переопределения параметров ходьбы.
Принимает данные IMU и может переопределить параметры позы и ходьбы.
Если не переопределяет - используются значения из конфигов.
"""

import rospy
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
        self.enabled = True
    
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
            rospy.logwarn("Stabilization module is disabled!")
            return StabilizationResult(modified=False)
        
        # Проверка данных
        if not imu_data:
            rospy.logwarn("Stabilization: imu_data is None!")
            return StabilizationResult(modified=False)
        
        if not imu_data.orientation:
            rospy.logwarn("Stabilization: imu_data.orientation is None or empty!")
            return StabilizationResult(modified=False)
        
        result = StabilizationResult()
        
        # Пример: прямая привязка позы к ориентации IMU
        pitch_deg = imu_data.orientation.get('pitch', 0.0)
        roll_deg = imu_data.orientation.get('roll', 0.0)
        
        # Привязка init_x_offset к pitch (тангаж)
        # pitch в градусах, делим на 100 для получения разумного диапазона
        init_x_offset = max(-0.05, min(0.05, -1*(pitch_deg - 90) / 1000.0))
        
        result.pose_override = RobotPoseParams(
            init_x_offset=init_x_offset,
            #init_roll_offset=roll_deg  # roll в градусах
        )
        result.modified = True
        
        # Логирование для отладки - используем loginfo чтобы точно видеть
        rospy.loginfo(f"[STABILIZATION] pitch={pitch_deg:.2f}°, roll={roll_deg:.2f}° -> "
                     f"init_x_offset={init_x_offset:.4f}, init_roll_offset={roll_deg:.2f}°")
        
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

