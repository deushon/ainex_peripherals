# Документация по топикам PID регулятора стабилизации

## Обзор

Система стабилизации корпуса робота использует единый PID регулятор для удержания вертикального положения как в покое, так и при ходьбе. Для отладки и настройки доступны два ROS топика.

## Топики

### 1. `/stabilization/pid_debug` (публикация)

**Тип:** `ainex_peripherals/PidDebug`

**Частота:** ~20 Hz (зависит от `update_interval`)

**Описание:** Отладочные данные для построения графиков и анализа работы PID регулятора.

#### Структура сообщения:

```python
Header header

# Ориентация робота (из IMU)
float64 roll_deg          # Угол roll (градусы)
float64 pitch_deg         # Угол pitch (градусы)
float64 yaw_deg           # Угол yaw (градусы)

# Угловые скорости (рад/с)
float64 roll_velocity
float64 pitch_velocity
float64 yaw_velocity

# Линейные ускорения (м/с²)
float64 linear_accel_x
float64 linear_accel_y
float64 linear_accel_z

# Текущие значения позы для стабилизации
float64 init_x_offset     # Смещение по X (м)
float64 init_y_offset     # Смещение по Y (м)
float64 init_roll_offset  # Смещение по roll (градусы)
float64 init_pitch_offset # Смещение по pitch (градусы)

# Управляющие воздействия от PID
float64 roll_error        # Ошибка по roll (градусы)
float64 pitch_error       # Ошибка по pitch (градусы)

# PID компоненты для roll
float64 roll_p_term       # Пропорциональная составляющая
float64 roll_i_term       # Интегральная составляющая
float64 roll_d_term       # Дифференциальная составляющая
float64 roll_pid_output   # Общий выход PID для roll

# PID компоненты для pitch
float64 pitch_p_term
float64 pitch_i_term
float64 pitch_d_term
float64 pitch_pid_output

# Применяемые корректировки
float64 applied_x_correction
float64 applied_y_correction
float64 applied_roll_correction
float64 applied_pitch_correction

# Состояние регулятора
bool pid_enabled
bool roll_enabled
bool pitch_enabled
bool is_walking           # true = ходьба, false = покой

# Интегральные составляющие
float64 roll_integral
float64 pitch_integral

# Время с последнего обновления
float64 dt
```

#### Пример использования (Python):

```python
#!/usr/bin/env python3
import rospy
from ainex_peripherals.msg import PidDebug

def callback(msg):
    print(f"Roll: {msg.roll_deg:.2f}°, Error: {msg.roll_error:.2f}°")
    print(f"Roll PID: P={msg.roll_p_term:.4f}, I={msg.roll_i_term:.4f}, D={msg.roll_d_term:.4f}")
    print(f"Output: {msg.roll_pid_output:.4f}, Applied Y: {msg.applied_y_correction:.4f}m")

rospy.init_node('pid_debug_listener')
sub = rospy.Subscriber('/stabilization/pid_debug', PidDebug, callback)
rospy.spin()
```

#### Пример использования (rqt_plot):

```bash
# График ошибок
rqt_plot /stabilization/pid_debug/roll_error /stabilization/pid_debug/pitch_error

# График PID компонентов
rqt_plot /stabilization/pid_debug/roll_p_term /stabilization/pid_debug/roll_i_term /stabilization/pid_debug/roll_d_term

# График выходов PID
rqt_plot /stabilization/pid_debug/roll_pid_output /stabilization/pid_debug/pitch_pid_output
```

---

### 2. `/stabilization/pid_config` (подписка)

**Тип:** `ainex_peripherals/PidConfig`

**Описание:** Топик для настройки параметров PID регулятора в реальном времени.

#### Структура сообщения:

```python
Header header

# Включение/выключение
bool pid_enabled
bool roll_enabled
bool pitch_enabled

# PID коэффициенты для roll
float64 roll_kp
float64 roll_ki
float64 roll_kd
float64 roll_max_output
float64 roll_target
float64 roll_integral_limit

# PID коэффициенты для pitch
float64 pitch_kp
float64 pitch_ki
float64 pitch_kd
float64 pitch_max_output
float64 pitch_target
float64 pitch_integral_limit

# Пороги активации
float64 roll_activation_threshold
float64 pitch_activation_threshold

# Базовые смещения позы
float64 base_x_offset
float64 base_y_offset
float64 base_roll_offset
float64 base_pitch_offset

# Пределы смещений от регулятора
float64 limit_x_offset_min
float64 limit_x_offset_max
float64 limit_y_offset_min
float64 limit_y_offset_max
float64 limit_roll_offset_min
float64 limit_roll_offset_max
float64 limit_pitch_offset_min
float64 limit_pitch_offset_max

# Коэффициенты преобразования
float64 roll_to_y_offset_ratio
float64 roll_to_roll_offset_ratio
float64 pitch_to_x_offset_ratio
float64 pitch_to_pitch_offset_ratio

# Настройки фильтра
bool filter_enabled
float64 filter_alpha

# Интервал обновления
float64 update_interval

# Флаги применения
bool apply_to_walking
bool apply_to_rest
bool reset_integral
```

**Важно:** 
- Если значение поля равно `None` или не установлено, оно не обновляется
- Установите `reset_integral=True` для сброса интегральных составляющих при изменении коэффициентов

#### Пример использования (Python):

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from ainex_peripherals.msg import PidConfig

rospy.init_node('pid_config_publisher')

pub = rospy.Publisher('/stabilization/pid_config', PidConfig, queue_size=10)
rospy.sleep(1)  # Ждем подключения подписчика

msg = PidConfig()
msg.header = Header()
msg.header.stamp = rospy.Time.now()

# Обновляем только коэффициенты roll
msg.roll_kp = 0.015
msg.roll_ki = 0.002
msg.roll_kd = 0.008
msg.roll_enabled = True
msg.reset_integral = True  # Сбросить интеграл при изменении коэффициентов

pub.publish(msg)
print("PID config updated")
```

#### Пример использования (rostopic):

```bash
# Публикация конфигурации через командную строку
rostopic pub /stabilization/pid_config ainex_peripherals/PidConfig \
  "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  pid_enabled: true
  roll_enabled: true
  pitch_enabled: true
  roll_kp: 0.015
  roll_ki: 0.002
  roll_kd: 0.008
  roll_max_output: 0.02
  roll_target: 0.0
  roll_integral_limit: 10.0
  pitch_kp: 0.015
  pitch_ki: 0.002
  pitch_kd: 0.008
  pitch_max_output: 0.02
  pitch_target: 90.0
  pitch_integral_limit: 10.0
  roll_activation_threshold: 2.0
  pitch_activation_threshold: 2.0
  base_x_offset: 0.0
  base_y_offset: 0.0
  base_roll_offset: 0.0
  base_pitch_offset: 0.0
  limit_x_offset_min: -0.03
  limit_x_offset_max: 0.03
  limit_y_offset_min: -0.03
  limit_y_offset_max: 0.03
  limit_roll_offset_min: -5.0
  limit_roll_offset_max: 5.0
  limit_pitch_offset_min: -5.0
  limit_pitch_offset_max: 5.0
  roll_to_y_offset_ratio: 1.0
  roll_to_roll_offset_ratio: 0.5
  pitch_to_x_offset_ratio: 1.0
  pitch_to_pitch_offset_ratio: 0.5
  filter_enabled: true
  filter_alpha: 0.7
  update_interval: 0.05
  apply_to_walking: true
  apply_to_rest: true
  reset_integral: false"
```

## Единая конфигурация PID

PID регулятор использует **единую конфигурацию** для покоя и ходьбы. Это означает:
- Одни и те же коэффициенты PID применяются в обоих режимах
- Регулятор всегда пытается удержать корпус вертикально
- Не требуется дублирование или переопределение параметров

## Быстрый старт

1. **Просмотр отладочных данных:**
   ```bash
   rostopic echo /stabilization/pid_debug
   ```

2. **Построение графиков:**
   ```bash
   rqt_plot /stabilization/pid_debug/roll_error /stabilization/pid_debug/roll_pid_output
   ```

3. **Настройка PID коэффициентов:**
   ```python
   # См. примеры выше
   ```

4. **Мониторинг в реальном времени:**
   ```bash
   # В одном терминале
   rostopic echo /stabilization/pid_debug -n 1
   
   # В другом терминале - публикуйте конфигурацию
   ```

## Рекомендации по настройке

1. **Начните с малых коэффициентов** и постепенно увеличивайте
2. **Следите за интегральной составляющей** - она не должна расти бесконечно
3. **Используйте графики** для визуализации реакции системы
4. **Тестируйте в покое** перед применением при ходьбе
5. **Сбрасывайте интеграл** (`reset_integral=True`) при изменении коэффициентов

## Типичные значения

- **kp**: 0.01 - 0.02 (м/градус)
- **ki**: 0.001 - 0.005
- **kd**: 0.005 - 0.01
- **max_output**: 0.015 - 0.03 (м)
- **activation_threshold**: 1.5 - 2.5 (градусы)
- **update_interval**: 0.05 (сек, 20 Hz)

