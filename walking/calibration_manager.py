#!/usr/bin/env python3
# encoding: utf-8
"""
Менеджер автоматической калибровки PID регулятора.
Управляет процессом калибровки: переключение setpoints, сбор данных, оптимизация.
"""

import rospy
import time
import threading
from typing import Optional, Callable
from simple_pid import PID
from config import ConfigLoader

try:
    from walking.pid_calibrator import PIDCalibrator, CalibrationResult
    CALIBRATION_AVAILABLE = True
except ImportError:
    CALIBRATION_AVAILABLE = False
    CalibrationResult = None


class CalibrationManager:
    """
    Менеджер автоматической калибровки PID регулятора.
    Управляет всем процессом калибровки.
    """
    
    def __init__(self, config_loader: ConfigLoader, pid_controller: PID,
                 target_angle: float, angle_range: float, output_limits: tuple,
                 start_delay: float = 5.0, duration: float = 60.0, 
                 setpoint_duration: float = 10.0,
                 calibrated_config_name: str = 'walking_pid'):
        """
        Инициализация менеджера калибровки.
        
        Args:
            config_loader: Экземпляр ConfigLoader для сохранения результатов
            pid_controller: PID контроллер для управления
            target_angle: Целевой угол для калибровки
            angle_range: Диапазон углов ±range
            output_limits: Пределы выходного сигнала (lower, upper)
            start_delay: Задержка перед началом калибровки
            duration: Общая длительность сбора данных
            setpoint_duration: Время удержания каждого setpoint
        """
        if not CALIBRATION_AVAILABLE:
            raise ImportError("PID calibration not available. Install scipy: pip install scipy")
        
        self.config = config_loader
        self.pid_controller = pid_controller
        self.target_angle = target_angle
        self.angle_range = angle_range
        self.output_limits = output_limits
        self.start_delay = start_delay
        self.duration = duration
        self.setpoint_duration = setpoint_duration
        self.calibrated_config_name = calibrated_config_name
        
        # Состояние калибровки
        self.state = 'idle'  # 'idle', 'waiting', 'collecting', 'optimizing', 'completed', 'failed'
        self.calibrator: Optional[PIDCalibrator] = None
        self.setpoints = []
        self.current_setpoint_index = 0
        self.setpoint_start_time = None
        self.start_time = None
        
        # Callbacks
        self.on_state_changed: Optional[Callable[[str], None]] = None
        self.on_completed: Optional[Callable[[CalibrationResult], None]] = None
        self.on_failed: Optional[Callable[[str], None]] = None
    
    def start(self):
        """Запускает процесс калибровки в отдельном потоке."""
        if self.state != 'idle':
            rospy.logwarn(f"Calibration already in progress. State: {self.state}")
            return
        
        thread = threading.Thread(target=self._calibration_thread, daemon=True)
        thread.start()
    
    def collect_data_point(self, pitch: float, pid_output: float):
        """
        Собирает точку данных для калибровки.
        Вызывается из основного цикла обработки.
        
        Args:
            pitch: Текущий угол pitch в градусах
            pid_output: Выход PID контроллера
        """
        if self.state == 'collecting' and self.calibrator is not None:
            self.calibrator.collect_data_point(pitch, pid_output)
    
    def update_setpoint(self, current_time: float) -> bool:
        """
        Обновляет setpoint для текущего этапа калибровки.
        Вызывается из основного цикла обработки.
        
        Args:
            current_time: Текущее время
        
        Returns:
            bool: True если нужно продолжить калибровку, False если завершена
        """
        if self.state != 'collecting':
            return False
        
        if self.setpoint_start_time is None:
            self.setpoint_start_time = current_time
        
        # Проверяем, нужно ли переключить setpoint
        if (current_time - self.setpoint_start_time) >= self.setpoint_duration:
            self.current_setpoint_index += 1
            if self.current_setpoint_index < len(self.setpoints):
                new_setpoint = self.setpoints[self.current_setpoint_index]
                self.pid_controller.setpoint = new_setpoint
                self.setpoint_start_time = current_time
                rospy.loginfo(f"Calibration: Switching to setpoint {self.current_setpoint_index + 1}/{len(self.setpoints)}: {new_setpoint:.1f}°")
            else:
                # Все setpoints протестированы
                self.calibrator.finish_data_collection()
                self.state = 'optimizing'
                self._set_state('optimizing')
                rospy.loginfo("Calibration: All setpoints tested. Starting optimization...")
                # Запускаем оптимизацию в отдельном потоке
                threading.Thread(target=self._run_optimization, daemon=True).start()
                return False
        
        return True
    
    def get_state(self) -> str:
        """Возвращает текущее состояние калибровки."""
        return self.state
    
    def get_current_setpoint(self) -> Optional[float]:
        """Возвращает текущий setpoint или None если калибровка не активна."""
        if self.state == 'collecting' and self.current_setpoint_index < len(self.setpoints):
            return self.setpoints[self.current_setpoint_index]
        return None
    
    def _calibration_thread(self):
        """Основной поток калибровки."""
        # Ждем задержку
        time.sleep(self.start_delay)
        
        if rospy.is_shutdown():
            return
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("PID CALIBRATION MODE - AUTOMATIC")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Target angle: {self.target_angle}°")
        rospy.loginfo(f"Angle range: ±{self.angle_range}°")
        rospy.loginfo("")
        rospy.loginfo("The system will automatically test different angles:")
        rospy.loginfo("  1. Minimum angle (target - range)")
        rospy.loginfo("  2. Target angle (center)")
        rospy.loginfo("  3. Maximum angle (target + range)")
        rospy.loginfo("")
        rospy.loginfo("The robot will try to maintain each angle while collecting data.")
        rospy.loginfo("Starting calibration in 3 seconds...")
        time.sleep(3)
        
        # Формируем последовательность setpoints
        min_angle = self.target_angle - self.angle_range
        max_angle = self.target_angle + self.angle_range
        
        cycles = max(2, int(self.duration / (self.setpoint_duration * 3)))
        
        self.setpoints = []
        for cycle in range(cycles):
            self.setpoints.extend([min_angle, self.target_angle, max_angle])
        
        rospy.loginfo(f"Will test {len(self.setpoints)} setpoints in {cycles} cycles")
        rospy.loginfo(f"Duration per setpoint: {self.setpoint_duration} seconds")
        rospy.loginfo(f"Total calibration time: ~{len(self.setpoints) * self.setpoint_duration:.0f} seconds")
        
        # Инициализируем калибровщик
        self.calibrator = PIDCalibrator(
            setpoint=self.target_angle,
            output_limits=self.output_limits,
            initial_kp=self.pid_controller.Kp if self.pid_controller else 0.001,
            initial_ki=self.pid_controller.Ki if self.pid_controller else 0.0,
            initial_kd=self.pid_controller.Kd if self.pid_controller else 0.0
        )
        
        # Начинаем сбор данных
        self.state = 'collecting'
        self._set_state('collecting')
        self.start_time = time.time()
        self.current_setpoint_index = 0
        self.setpoint_start_time = None
        
        # Устанавливаем первый setpoint
        if self.pid_controller is not None and len(self.setpoints) > 0:
            self.pid_controller.setpoint = self.setpoints[0]
            rospy.loginfo(f"Calibration: Starting with setpoint 1/{len(self.setpoints)}: {self.setpoints[0]:.1f}°")
        
        self.calibrator.start_data_collection()
        rospy.loginfo("Data collection started! System will automatically test different angles...")
    
    def _run_optimization(self):
        """Выполняет оптимизацию параметров PID."""
        if self.calibrator is None:
            return
        
        rospy.loginfo("Optimizing PID parameters...")
        
        result = self.calibrator.calibrate()
        
        if result.success:
            # Сохраняем результаты
            self._save_calibrated_parameters(result)
            
            # Вызываем callback
            if self.on_completed:
                self.on_completed(result)
            
            self.state = 'completed'
            self._set_state('completed')
            
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
            self.state = 'failed'
            self._set_state('failed')
            
            error_msg = result.message
            if self.on_failed:
                self.on_failed(error_msg)
            
            rospy.logerr("=" * 60)
            rospy.logerr("PID CALIBRATION FAILED!")
            rospy.logerr(f"Error: {error_msg}")
            rospy.logerr("Using default parameters from config.")
            rospy.logerr("=" * 60)
    
    def _save_calibrated_parameters(self, result: CalibrationResult):
        """
        Сохраняет откалиброванные параметры в конфиг.
        
        Args:
            result: CalibrationResult с откалиброванными параметрами
        """
        try:
            # Загружаем конфиг калибровки (отдельный файл, чтобы не трогать walking.yaml)
            full_config = self.config.load(self.calibrated_config_name)
            
            # Убеждаемся, что структура существует
            if not isinstance(full_config, dict):
                full_config = {}
            if 'stabilization_pid' not in full_config:
                full_config['stabilization_pid'] = {}
            
            # Устанавливаем откалиброванные параметры (конвертируем в стандартные Python типы)
            full_config['stabilization_pid']['calibrated_kp'] = float(result.kp)
            full_config['stabilization_pid']['calibrated_ki'] = float(result.ki)
            full_config['stabilization_pid']['calibrated_kd'] = float(result.kd)
            
            # Сохраняем ВЕСЬ конфиг целиком (включая все ключи верхнего уровня)
            # Метод save делает глубокое слияние, поэтому все существующие данные сохранятся
            self.config.save(self.calibrated_config_name, full_config)
            
            rospy.loginfo("Calibrated PID parameters saved to config file")
        except Exception as e:
            rospy.logerr(f"Failed to save calibrated parameters: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def _set_state(self, new_state: str):
        """Устанавливает новое состояние и вызывает callback."""
        self.state = new_state
        if self.on_state_changed:
            self.on_state_changed(new_state)

