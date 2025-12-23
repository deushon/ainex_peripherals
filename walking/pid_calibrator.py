#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль автоматической калибровки PID регулятора.
Использует scipy.optimize для оптимизации параметров PID на основе реальных данных.
"""

import time
import numpy as np
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass
from simple_pid import PID

try:
    import rospy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # Создаем заглушку для rospy
    class RosPyStub:
        def loginfo(self, msg): print(f"[INFO] {msg}")
        def logwarn(self, msg): print(f"[WARN] {msg}")
        def logerr(self, msg): print(f"[ERROR] {msg}")
    rospy = RosPyStub()

try:
    from scipy.optimize import minimize, differential_evolution
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    if ROS_AVAILABLE:
        rospy.logwarn("scipy not available! Install with: pip install scipy")
    else:
        print("[WARN] scipy not available! Install with: pip install scipy")


@dataclass
class CalibrationDataPoint:
    """Точка данных для калибровки."""
    timestamp: float
    pitch: float  # Текущий угол pitch в градусах
    setpoint: float  # Целевой угол в градусах
    pid_output: float  # Выход PID контроллера
    error: float  # Ошибка (setpoint - pitch)


@dataclass
class CalibrationResult:
    """Результат калибровки."""
    kp: float
    ki: float
    kd: float
    fitness: float  # Значение функции качества (чем меньше, тем лучше)
    success: bool
    message: str


class PIDCalibrator:
    """
    Класс для автоматической калибровки PID регулятора.
    Использует оптимизацию для поиска оптимальных параметров.
    """
    
    def __init__(self, setpoint: float, output_limits: Tuple[float, float], 
                 initial_kp: float = 0.001, initial_ki: float = 0.0, initial_kd: float = 0.0):
        """
        Инициализация калибровщика.
        
        Args:
            setpoint: Целевое значение pitch в градусах
            output_limits: Пределы выходного сигнала (lower, upper)
            initial_kp: Начальное значение Kp
            initial_ki: Начальное значение Ki
            initial_kd: Начальное значение Kd
        """
        if not SCIPY_AVAILABLE:
            raise ImportError("scipy is required for PID calibration. Install with: pip install scipy")
        
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.initial_params = (initial_kp, initial_ki, initial_kd)
        
        # Диапазоны поиска параметров (можно настроить)
        # Kp: обычно от 0.0001 до 0.1
        # Ki: обычно от 0.0 до 0.5
        # Kd: обычно от 0.0 до 0.01
        self.param_bounds = [
            (0.0001, 0.1),   # Kp
            (0.0, 0.5),      # Ki
            (0.0, 0.01)      # Kd
        ]
        
        self.calibration_data: List[CalibrationDataPoint] = []
        self.data_collection_started = False
        self.data_collection_finished = False
    
    def start_data_collection(self):
        """Начинает сбор данных для калибровки."""
        self.calibration_data = []
        self.data_collection_started = True
        self.data_collection_finished = False
        rospy.loginfo("PID calibration: Started data collection")
    
    def collect_data_point(self, pitch: float, pid_output: float):
        """
        Собирает точку данных для калибровки.
        
        Args:
            pitch: Текущий угол pitch в градусах
            pid_output: Выход PID контроллера
        """
        if not self.data_collection_started:
            return
        
        error = self.setpoint - pitch
        data_point = CalibrationDataPoint(
            timestamp=time.time(),
            pitch=pitch,
            setpoint=self.setpoint,
            pid_output=pid_output,
            error=error
        )
        self.calibration_data.append(data_point)
    
    def finish_data_collection(self):
        """Завершает сбор данных."""
        self.data_collection_finished = True
        rospy.loginfo(f"PID calibration: Finished data collection. Collected {len(self.calibration_data)} points")
    
    def _evaluate_pid_params(self, params: Tuple[float, float, float], 
                            data_points: List[CalibrationDataPoint]) -> float:
        """
        Оценивает качество параметров PID на основе собранных данных.
        Использует интеграл квадрата ошибки (ISE) как метрику качества.
        
        Args:
            params: Параметры (kp, ki, kd)
            data_points: Собранные точки данных
        
        Returns:
            float: Значение функции качества (чем меньше, тем лучше)
        """
        if len(data_points) < 10:
            return 1e6  # Очень плохое значение при недостатке данных
        
        kp, ki, kd = params
        
        # Создаем временный PID контроллер с тестируемыми параметрами
        try:
            pid = PID(
                Kp=kp,
                Ki=ki,
                Kd=kd,
                setpoint=self.setpoint,
                output_limits=self.output_limits,
                sample_time=None
            )
        except Exception:
            return 1e6
        
        # Симулируем работу PID на собранных данных
        total_error_squared = 0.0
        last_time = None
        
        for i, point in enumerate(data_points):
            if last_time is not None:
                dt = point.timestamp - last_time
                if dt > 0:
                    pid.sample_time = dt
            
            # Вычисляем выход PID для этого момента
            pid_output = pid(point.pitch)
            
            # Вычисляем ошибку
            error = point.setpoint - point.pitch
            
            # Добавляем к интегралу квадрата ошибки
            if i > 0:
                dt = point.timestamp - data_points[i-1].timestamp
                total_error_squared += error * error * dt
            
            last_time = point.timestamp
        
        # Также учитываем перерегулирование и колебания
        # Добавляем штраф за большие изменения выходного сигнала
        output_variance = 0.0
        if len(data_points) > 1:
            outputs = [p.pid_output for p in data_points]
            output_variance = np.var(outputs) if len(outputs) > 1 else 0.0
        
        # Функция качества: ISE + штраф за колебания
        fitness = total_error_squared + 0.1 * output_variance
        
        return fitness
    
    def calibrate(self, method: str = 'differential_evolution') -> CalibrationResult:
        """
        Выполняет калибровку PID регулятора.
        
        Args:
            method: Метод оптимизации ('differential_evolution' или 'minimize')
        
        Returns:
            CalibrationResult: Результат калибровки
        """
        if not self.data_collection_finished or len(self.calibration_data) < 50:
            return CalibrationResult(
                kp=self.initial_params[0],
                ki=self.initial_params[1],
                kd=self.initial_params[2],
                fitness=1e6,
                success=False,
                message=f"Insufficient data for calibration: {len(self.calibration_data)} points (need at least 50)"
            )
        
        rospy.loginfo(f"Starting PID calibration with {len(self.calibration_data)} data points...")
        
        # Функция для минимизации
        def objective(params):
            return self._evaluate_pid_params(params, self.calibration_data)
        
        try:
            if method == 'differential_evolution':
                # Дифференциальная эволюция - более надежный метод для глобальной оптимизации
                result = differential_evolution(
                    objective,
                    bounds=self.param_bounds,
                    seed=42,
                    maxiter=100,
                    popsize=15,
                    tol=1e-6,
                    atol=1e-6
                )
                optimal_params = result.x
                fitness = result.fun
                success = result.success
                message = result.message
            else:
                # Локальная оптимизация с начальными параметрами
                result = minimize(
                    objective,
                    x0=self.initial_params,
                    method='L-BFGS-B',
                    bounds=self.param_bounds,
                    options={'maxiter': 100}
                )
                optimal_params = result.x
                fitness = result.fun
                success = result.success
                message = result.message if hasattr(result, 'message') else 'Optimization completed'
            
            kp, ki, kd = optimal_params
            
            rospy.loginfo(f"PID calibration completed: Kp={kp:.6f}, Ki={ki:.6f}, Kd={kd:.6f}, "
                         f"fitness={fitness:.6f}, success={success}")
            
            return CalibrationResult(
                kp=kp,
                ki=ki,
                kd=kd,
                fitness=fitness,
                success=success,
                message=str(message)
            )
        
        except Exception as e:
            rospy.logerr(f"PID calibration failed: {e}")
            return CalibrationResult(
                kp=self.initial_params[0],
                ki=self.initial_params[1],
                kd=self.initial_params[2],
                fitness=1e6,
                success=False,
                message=f"Calibration error: {str(e)}"
            )
    
    def set_param_bounds(self, kp_range: Tuple[float, float], 
                        ki_range: Tuple[float, float], 
                        kd_range: Tuple[float, float]):
        """
        Устанавливает диапазоны поиска параметров.
        
        Args:
            kp_range: Диапазон для Kp (min, max)
            ki_range: Диапазон для Ki (min, max)
            kd_range: Диапазон для Kd (min, max)
        """
        self.param_bounds = [kp_range, ki_range, kd_range]

