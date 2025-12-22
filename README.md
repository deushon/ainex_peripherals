# ainex_peripherals

ROS пакет для управления периферийными устройствами робота через джойстик.

## Структура пакета

```
ainex_peripherals/
├── config/                          # Конфигурационные файлы
│   ├── joystick.yaml               # Конфигурация джойстика
│   ├── walking.yaml                # Параметры ходьбы
│   ├── serial.yaml                 # Параметры Serial порта
│   ├── imu.yaml                    # Параметры IMU
│   ├── buttons.yaml                # Параметры кнопок
│   ├── game.yaml                   # Параметры игры
│   └── config_loader.py            # Загрузчик конфигурации
│
├── game/                           # Игровые сервисы
│   └── services.py                 # GameServices
│
├── walking/                        # Логика ходьбы
│   ├── speed_control.py            # SpeedControl
│   ├── stabilization.py            # StabilizationModule
│   ├── interfaces.py               # Интерфейсы данных
│   └── converter.py                # Конвертеры данных
│
├── imu/                            # Обработка данных IMU
│   └── data_handler.py             # IMUDataHandler (преобразование данных)
│
├── robot_state/                    # Управление состоянием робота
│   ├── manager.py                  # RobotStateManager (централизованное состояние)
│   ├── detector.py                 # FallDetector (обнаружение падений)
│   └── auto_getup.py               # AutoGetup (автоматический подъем)
│
├── control/                        # Управление
│   ├── joystick_controller.py      # JoystickController (главный контроллер)
│   ├── button_handler.py           # ButtonHandler
│   └── serial_handler.py           # SerialHandler
│
├── utils/                          # Утилиты
│   ├── copy_action_groups.sh
│   └── copy_action_groups_from_external.sh
│
├── scripts/                        # Исполняемые скрипты
│   ├── joystick_control.py         # Точка входа
│   └── tf_broadcaster_imu.py
│
├── launch/                         # Launch файлы
├── action_groups/                  # Action groups
└── README.md
```

## Модули

### config/
Все конфигурационные параметры вынесены в отдельные YAML файлы. Используется `ConfigLoader` для загрузки конфигурации.

### game/
Модуль игровых сервисов:
- Управление HP робота
- Система разрешений (движение, стрельба, голова, камера)
- Обработка урона
- Статус робота

### walking/
Модуль управления ходьбой:
- `SpeedControl` - управление скоростью и параметрами ходьбы
- `StabilizationModule` - единственное место переопределения параметров ходьбы на основе IMU данных
- Интерфейсы данных для четкого определения структур
- Конвертеры между форматами gait_manager и интерфейсами

### imu/
Модуль обработки данных IMU:
- `IMUDataHandler` - простой обработчик данных IMU (преобразование quaternion в Euler)
- Только обработка данных, без логики состояний

### robot_state/
Модуль управления состоянием робота:
- `RobotStateManager` - централизованное хранилище состояния робота (доступно для всего пакета)
- `FallDetector` - обнаружение падений на основе данных IMU
- `AutoGetup` - автоматический подъем после падения
- Оптимизирован для производительности (без locks, использование deque)

### control/
Модули управления:
- `JoystickController` - главный контроллер, объединяющий все модули
- `ButtonHandler` - обработка действий кнопок джойстика
- `SerialHandler` - обработка Serial порта для связи с Arduino (оптимизирован без locks)

## Использование

### Запуск

```bash
roslaunch ainex_peripherals joystick_control.launch robot_id:=robot_1 port:=/dev/ttyUSB0 baudrate:=9600
```

### Настройка конфигурации

Все параметры настраиваются через YAML файлы в директории `config/`:
- `config/walking.yaml` - параметры режимов скорости ходьбы
- `config/imu.yaml` - параметры обнаружения падений и автоматического подъема
- `config/buttons.yaml` - параметры обработки кнопок
- `config/game.yaml` - параметры игры
- `config/serial.yaml` - параметры Serial порта

### Модуль стабилизации

Модуль стабилизации (`walking/stabilization.py`) - единственное место переопределения параметров ходьбы на основе данных IMU. 

Для реализации логики стабилизации отредактируйте метод `process()` класса `StabilizationModule`:

```python
def process(self, imu_data, throttle_data, current_walking_params):
    result = StabilizationResult(modified=False)
    
    # Ваша логика стабилизации
    # Пример: корректировка позы
    roll = imu_data.orientation['roll']
    if abs(roll) > 5.0:
        result.pose_override = RobotPoseParams(
            init_roll_offset=-roll * 0.5
        )
        result.modified = True
    
    return result
```

Если `result.modified = False` или override поля равны `None`, используются значения из конфигурации.

## Особенности

- **Чистая архитектура**: четкое разделение ответственности между модулями
- **Производительность**: оптимизирован без использования locks (threading.Lock)
- **Конфигурация**: все параметры вынесены в отдельные YAML файлы
- **Интерфейсы данных**: четко определенные структуры данных для обмена между модулями
- **Единственное место переопределения**: модуль стабилизации - единственное место изменения параметров ходьбы
- **Отсутствие legacy кода**: удалены все заглушки и временные решения

## Зависимости

- ROS (rospy, std_msgs)
- rospkg
- python3-yaml (PyYAML)
- ainex_sdk
- ainex_kinematics
