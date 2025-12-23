#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль стабилизации - единственное место переопределения параметров ходьбы.
Принимает данные IMU и может переопределить параметры позы и ходьбы.
Если не переопределяет - используются значения из конфигов.
"""

import rospy
import time
from typing import Optional
from simple_pid import PID
from config import ConfigLoader
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
    
    def __init__(self, config_loader=None):
        """
        Инициализация модуля стабилизации.
        
        Args:
            config_loader: Экземпляр ConfigLoader (опционально)
        """
        self.enabled = True
        self.config = config_loader if config_loader else ConfigLoader()
        
        # Загружаем конфигурацию PID
        walking_config = self.config.load('walking')
        pid_config = walking_config.get('walking', {}).get('stabilization_pid', {})
        
        self.pid_enabled = pid_config.get('enabled', True)
        self.pid_setpoint = pid_config.get('setpoint', 90.0)
        
        # Параметры PID
        kp = pid_config.get('kp', 0.001)
        ki = pid_config.get('ki', 0.0)
        kd = pid_config.get('kd', 0.0)
        
        # Пределы выходного сигнала
        self.output_limits = (
            pid_config.get('lower_limit', -0.05),
            pid_config.get('upper_limit', 0.05)
        )
        
        
        # Инициализируем PID контроллер
        if self.pid_enabled:
            try:
                self.pid_controller = PID(
                    Kp=kp,
                    Ki=ki,
                    Kd=kd,
                    setpoint=self.pid_setpoint,
                    output_limits=self.output_limits,
                    sample_time=None  # Обновляем при каждом вызове
                )
                self.last_time = None
                rospy.loginfo(f"PID stabilization enabled: Kp={kp}, Ki={ki}, Kd={kd}, "
                            f"setpoint={self.pid_setpoint}°, limits={self.output_limits}, ")
            except ImportError:
                rospy.logerr("simple-pid library not installed! Install with: pip install simple-pid")
                self.pid_enabled = False
                self.pid_controller = None
        else:
            self.pid_controller = None
            rospy.loginfo("PID stabilization disabled")
    
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
        
        pitch_deg = imu_data.orientation.get('pitch', 0.0)
        roll_deg = imu_data.orientation.get('roll', 0.0)
        
        # Используем PID контроллер для стабилизации по pitch
        if self.pid_enabled and self.pid_controller is not None:
            # Вычисляем управляющее воздействие от PID
            current_time = time.time()
            if self.last_time is None:
                self.last_time = current_time
            
            # Обновляем sample_time для PID (время с последнего вызова)
            dt = current_time - self.last_time
            if dt > 0:
                self.pid_controller.sample_time = dt
            
            # Вычисляем control_effort от PID
            pid_output = self.pid_controller(pitch_deg)
            self.last_time = current_time
            
            # Обновляем hip_pitch_offset в gait_base
            hip_pitch_offset = 15 - pid_output
            
            result.gait_base_override = GaitBaseParams(
                body_height=current_walking_params.gait_base.body_height,
                step_fb_ratio=current_walking_params.gait_base.step_fb_ratio,
                z_swap_amplitude=current_walking_params.gait_base.z_swap_amplitude,
                hip_pitch_offset=hip_pitch_offset,
                pelvis_offset=current_walking_params.gait_base.pelvis_offset
            )
            result.modified = True
            
            # Логирование отключено для максимальной скорости
            # rospy.logdebug(f"[PID STABILIZATION] pitch={pitch_deg:.2f}°, roll={roll_deg:.2f}°, "
            #              f"setpoint={self.pid_setpoint}°, init_x_offset={init_x_offset:.4f}, "
            #              f"init_y_offset={init_y_offset:.4f}")
        else:
            # Если PID отключен, возвращаем без изменений
            result.modified = False
        
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
        if self.pid_controller is not None:
            self.pid_controller.reset()
        self.last_time = None
    
    def update_pid_parameters(self, kp=None, ki=None, kd=None, setpoint=None):
        """
        Обновляет параметры PID контроллера во время работы.
        
        Args:
            kp: Пропорциональный коэффициент
            ki: Интегральный коэффициент
            kd: Дифференциальный коэффициент
            setpoint: Целевое значение pitch
        """
        if self.pid_controller is not None:
            if kp is not None:
                self.pid_controller.Kp = kp
            if ki is not None:
                self.pid_controller.Ki = ki
            if kd is not None:
                self.pid_controller.Kd = kd
            if setpoint is not None:
                self.pid_controller.setpoint = setpoint
                self.pid_setpoint = setpoint

