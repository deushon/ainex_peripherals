#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль стабилизации - единственное место переопределения параметров ходьбы.
Принимает данные IMU и может переопределить параметры позы и ходьбы.
Если не переопределяет - используются значения из конфигов.
"""

import rospy
import time
import threading
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

try:
    from walking.pid_calibrator import PIDCalibrator
    CALIBRATION_AVAILABLE = True
except ImportError as e:
    CALIBRATION_AVAILABLE = False
    try:
        rospy.logwarn(f"PID calibration not available. Install scipy: pip install scipy. Error: {e}")
    except:
        pass  # rospy может быть не инициализирован при импорте


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
        
        # Параметры калибровки
        calibration_config = pid_config.get('calibration', {})
        self.calibration_enabled = calibration_config.get('enabled', False)
        self.calibration_delay = calibration_config.get('start_delay', 5.0)  # Задержка перед началом калибровки
        self.calibration_duration = calibration_config.get('duration', 60.0)  # Общая длительность сбора данных
        self.calibration_angle_range = calibration_config.get('angle_range', 10.0)  # Диапазон углов ±10°
        self.calibration_target_angle = calibration_config.get('target_angle', 85.0)  # Целевой угол для калибровки
        self.calibration_setpoint_duration = calibration_config.get('setpoint_duration', 10.0)  # Время удержания каждого угла
        
        # Параметры PID (используем откалиброванные значения, если есть)
        calibrated_kp = pid_config.get('calibrated_kp')
        calibrated_ki = pid_config.get('calibrated_ki')
        calibrated_kd = pid_config.get('calibrated_kd')
        
        if calibrated_kp is not None and not self.calibration_enabled:
            # Используем откалиброванные значения, если калибровка не включена
            kp = calibrated_kp
            ki = calibrated_ki if calibrated_ki is not None else 0.0
            kd = calibrated_kd if calibrated_kd is not None else 0.0
            rospy.loginfo(f"Using calibrated PID parameters: Kp={kp:.6f}, Ki={ki:.6f}, Kd={kd:.6f}")
        else:
            # Используем значения из конфига
            kp = pid_config.get('kp', 0.001)
            ki = pid_config.get('ki', 0.0)
            kd = pid_config.get('kd', 0.0)
        
        # Пределы выходного сигнала
        self.output_limits = (
            pid_config.get('lower_limit', -0.05),
            pid_config.get('upper_limit', 0.05)
        )
        
        # Состояние калибровки
        self.calibration_state = 'idle'  # 'idle', 'waiting', 'collecting', 'optimizing', 'completed', 'failed'
        self.calibration_start_time = None
        self.calibrator = None
        self.init_time = time.time()
        self.calibration_current_setpoint = None  # Текущий setpoint для калибровки
        self.calibration_setpoint_index = 0  # Индекс текущего setpoint в последовательности
        self.calibration_setpoints = []  # Список setpoints для тестирования
        self.calibration_setpoint_duration = 10.0  # Время удержания каждого setpoint (секунды)
        self.calibration_setpoint_start_time = None
        
        
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
        
        # Запускаем калибровку, если включена
        if self.calibration_enabled and CALIBRATION_AVAILABLE and self.pid_enabled:
            self._start_calibration_sequence()
    
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
            # Обработка калибровки - автоматическое переключение setpoints
            if self.calibration_state == 'collecting' and self.calibrator is not None:
                current_time = time.time()
                
                # Проверяем, нужно ли переключить setpoint
                if self.calibration_setpoint_start_time is None:
                    self.calibration_setpoint_start_time = current_time
                
                # Если прошло достаточно времени, переключаемся на следующий setpoint
                if (current_time - self.calibration_setpoint_start_time) >= self.calibration_setpoint_duration:
                    self.calibration_setpoint_index += 1
                    if self.calibration_setpoint_index < len(self.calibration_setpoints):
                        self.calibration_current_setpoint = self.calibration_setpoints[self.calibration_setpoint_index]
                        self.pid_controller.setpoint = self.calibration_current_setpoint
                        self.calibration_setpoint_start_time = current_time
                        rospy.loginfo(f"Calibration: Switching to setpoint {self.calibration_setpoint_index + 1}/{len(self.calibration_setpoints)}: {self.calibration_current_setpoint:.1f}°")
                    else:
                        # Все setpoints протестированы, завершаем сбор данных
                        self.calibrator.finish_data_collection()
                        self.calibration_state = 'optimizing'
                        rospy.loginfo("Calibration: All setpoints tested. Starting optimization...")
                        # Запускаем оптимизацию в отдельном потоке
                        threading.Thread(target=self._run_calibration_optimization, daemon=True).start()
            
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
            
            # Собираем данные для калибровки (если идет калибровка)
            if self.calibration_state == 'collecting' and self.calibrator is not None:
                self.calibrator.collect_data_point(pitch_deg, pid_output)
            
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
    
    def _start_calibration_sequence(self):
        """Запускает последовательность автоматической калибровки."""
        if not CALIBRATION_AVAILABLE:
            rospy.logerr("PID calibration not available. Install scipy: pip install scipy")
            return
        
        def calibration_thread():
            # Ждем задержку перед началом калибровки
            time.sleep(self.calibration_delay)
            
            if rospy.is_shutdown():
                return
            
            rospy.loginfo("=" * 60)
            rospy.loginfo("PID CALIBRATION MODE - AUTOMATIC")
            rospy.loginfo("=" * 60)
            rospy.loginfo(f"Target angle: {self.calibration_target_angle}°")
            rospy.loginfo(f"Angle range: ±{self.calibration_angle_range}°")
            rospy.loginfo("")
            rospy.loginfo("The system will automatically test different angles:")
            rospy.loginfo("  1. Minimum angle (target - range)")
            rospy.loginfo("  2. Target angle (center)")
            rospy.loginfo("  3. Maximum angle (target + range)")
            rospy.loginfo("")
            rospy.loginfo("The robot will try to maintain each angle while collecting data.")
            rospy.loginfo("Starting calibration in 3 seconds...")
            time.sleep(3)
            
            # Формируем последовательность setpoints для тестирования
            # Тестируем: минимум -> центр -> максимум, повторяем несколько раз
            min_angle = self.calibration_target_angle - self.calibration_angle_range
            max_angle = self.calibration_target_angle + self.calibration_angle_range
            
            # Количество циклов зависит от длительности калибровки
            cycles = max(2, int(self.calibration_duration / (self.calibration_setpoint_duration * 3)))
            
            self.calibration_setpoints = []
            for cycle in range(cycles):
                self.calibration_setpoints.extend([min_angle, self.calibration_target_angle, max_angle])
            
            rospy.loginfo(f"Will test {len(self.calibration_setpoints)} setpoints in {cycles} cycles")
            rospy.loginfo(f"Duration per setpoint: {self.calibration_setpoint_duration} seconds")
            rospy.loginfo(f"Total calibration time: ~{len(self.calibration_setpoints) * self.calibration_setpoint_duration:.0f} seconds")
            
            # Инициализируем калибровщик
            self.calibrator = PIDCalibrator(
                setpoint=self.calibration_target_angle,
                output_limits=self.output_limits,
                initial_kp=self.pid_controller.Kp if self.pid_controller else 0.001,
                initial_ki=self.pid_controller.Ki if self.pid_controller else 0.0,
                initial_kd=self.pid_controller.Kd if self.pid_controller else 0.0
            )
            
            # Начинаем сбор данных
            self.calibration_state = 'collecting'
            self.calibration_start_time = time.time()
            self.calibration_setpoint_index = 0
            self.calibration_current_setpoint = self.calibration_setpoints[0]
            self.calibration_setpoint_start_time = None
            
            # Устанавливаем первый setpoint
            if self.pid_controller is not None:
                self.pid_controller.setpoint = self.calibration_current_setpoint
                rospy.loginfo(f"Calibration: Starting with setpoint 1/{len(self.calibration_setpoints)}: {self.calibration_current_setpoint:.1f}°")
            
            self.calibrator.start_data_collection()
            rospy.loginfo("Data collection started! System will automatically test different angles...")
        
        # Запускаем калибровку в отдельном потоке
        thread = threading.Thread(target=calibration_thread, daemon=True)
        thread.start()
    
    def _run_calibration_optimization(self):
        """Выполняет оптимизацию параметров PID после сбора данных."""
        if self.calibrator is None:
            return
        
        rospy.loginfo("Optimizing PID parameters...")
        
        # Выполняем калибровку
        result = self.calibrator.calibrate()
        
        if result.success:
            # Сохраняем результаты в конфиг
            self._save_calibrated_parameters(result)
            
            # Обновляем PID контроллер с новыми параметрами
            self.update_pid_parameters(
                kp=result.kp,
                ki=result.ki,
                kd=result.kd
            )
            
            # Возвращаем setpoint к нормальному значению
            if self.pid_controller is not None:
                self.pid_controller.setpoint = self.pid_setpoint
            
            self.calibration_state = 'completed'
            rospy.loginfo("=" * 60)
            rospy.loginfo("PID CALIBRATION COMPLETED SUCCESSFULLY!")
            rospy.loginfo(f"Optimized parameters:")
            rospy.loginfo(f"  Kp = {result.kp:.6f}")
            rospy.loginfo(f"  Ki = {result.ki:.6f}")
            rospy.loginfo(f"  Kd = {result.kd:.6f}")
            rospy.loginfo(f"  Fitness = {result.fitness:.6f}")
            rospy.loginfo("Parameters saved to config file.")
            rospy.loginfo("Switching to normal operation mode...")
            rospy.loginfo("=" * 60)
        else:
            self.calibration_state = 'failed'
            # Возвращаем setpoint к нормальному значению
            if self.pid_controller is not None:
                self.pid_controller.setpoint = self.pid_setpoint
            rospy.logerr("=" * 60)
            rospy.logerr("PID CALIBRATION FAILED!")
            rospy.logerr(f"Error: {result.message}")
            rospy.logerr("Using default parameters from config.")
            rospy.logerr("=" * 60)
    
    def _save_calibrated_parameters(self, result):
        """
        Сохраняет откалиброванные параметры в конфиг.
        
        Args:
            result: CalibrationResult с откалиброванными параметрами
        """
        try:
            # Загружаем текущий конфиг
            walking_config = self.config.load('walking')
            if 'walking' not in walking_config:
                walking_config['walking'] = {}
            if 'stabilization_pid' not in walking_config['walking']:
                walking_config['walking']['stabilization_pid'] = {}
            
            # Устанавливаем откалиброванные параметры
            walking_config['walking']['stabilization_pid']['calibrated_kp'] = result.kp
            walking_config['walking']['stabilization_pid']['calibrated_ki'] = result.ki
            walking_config['walking']['stabilization_pid']['calibrated_kd'] = result.kd
            
            # Сохраняем конфиг
            self.config.save('walking', walking_config)
            
            rospy.loginfo("Calibrated PID parameters saved to config file")
        except Exception as e:
            rospy.logerr(f"Failed to save calibrated parameters: {e}")
    
    def get_calibration_status(self):
        """
        Возвращает статус калибровки.
        
        Returns:
            str: Текущий статус калибровки
        """
        return self.calibration_state

