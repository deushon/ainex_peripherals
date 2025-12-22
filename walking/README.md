# Модуль стабилизации

Единственное место для переопределения параметров ходьбы и позы робота на основе данных IMU.

## Структура

- `interfaces.py` - определения интерфейсов данных (IMUData, WalkingParams, StabilizationResult и т.д.)
- `stabilization.py` - основной модуль стабилизации (StabilizationModule)
- `converter.py` - утилиты для конвертации между форматами gait_manager и интерфейсами

## Использование

### Базовый пример

```python
from walking import StabilizationModule, IMUData, ThrottleData, WalkingParams
from walking import converter

# Создание модуля
stabilization = StabilizationModule()
stabilization.set_enabled(True)

# Подготовка данных IMU
imu_data = IMUData(
    orientation={'roll': 1.0, 'pitch': 2.0, 'yaw': 0.0},
    angular_velocity={'x': 0.0, 'y': 0.0, 'z': 0.0},
    linear_acceleration={'x': 0.0, 'y': 0.0, 'z': 9.8}
)

# Подготовка данных throttle
throttle_data = ThrottleData(x=0.01, y=0.0, angle=0.0)

# Получение текущих параметров из gait_manager
gait_param = gait_manager.get_gait_param()
period_time = [400, 0.2, 0.02]
move_amplitudes = {'x': 0.01, 'y': 0.0, 'angle': 0.0}

# Конвертация в WalkingParams
current_params = converter.gait_param_to_walking_params(
    gait_param, period_time, move_amplitudes
)

# Обработка стабилизации
result = stabilization.process(imu_data, throttle_data, current_params)

# Применение результата
if result.modified:
    updated_params = converter.apply_stabilization_result(current_params, result)
    # Конвертация обратно для gait_manager
    updated_gait_param = converter.walking_params_to_gait_param(updated_params)
    updated_period_time = converter.walking_params_to_period_time(updated_params)
    # Использование обновленных параметров
    gait_manager.set_step(updated_period_time, ...)
```

### Реализация логики стабилизации

В классе `StabilizationModule.process()` реализуйте свою логику:

```python
def process(self, imu_data, throttle_data, current_walking_params):
    result = StabilizationResult(modified=False)
    
    # Пример: корректировка позы на основе наклона
    roll = imu_data.orientation['roll']
    pitch = imu_data.orientation['pitch']
    
    if abs(roll) > 5.0:  # Если наклон больше 5 градусов
        result.pose_override = RobotPoseParams(
            init_roll_offset=-roll * 0.5,  # Коррекция
            init_pitch_offset=current_walking_params.pose.init_pitch_offset
        )
        result.modified = True
    
    return result
```

## Важные моменты

1. **Единственное место переопределения**: Все изменения параметров ходьбы должны проходить через `StabilizationModule`

2. **Если не переопределяете**: Верните `StabilizationResult(modified=False)` или `None` для override полей - будут использованы значения из конфига

3. **Интерфейсы четко определены**: Используйте типизированные структуры из `interfaces.py` для всех данных

4. **Конвертеры**: Используйте функции из `converter.py` для преобразования между форматами gait_manager и интерфейсами

