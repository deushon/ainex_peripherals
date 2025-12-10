# Как проверить логи IMU адаптации

## Быстрая проверка - видите ли вы логи вообще?

### 1. Проверьте, что нода запущена и получает IMU данные

```bash
# Проверьте, что нода работает
rosnode list | grep joystick_control

# Проверьте, что IMU топик публикуется
rostopic hz /imu

# Проверьте, что нода подписана на IMU
rostopic info /imu
```

### 2. Самый простой способ - смотреть логи в терминале, где запущена нода

Если вы запускаете ноду через `rosrun` или `roslaunch`, логи должны появляться прямо в этом терминале.

**При запуске через roslaunch:**
```bash
roslaunch ainex_peripherals joystick_control.launch
```

**При запуске через rosrun:**
```bash
rosrun ainex_peripherals joystick_control.py
```

### 3. Если логи не видны в терминале запуска

Попробуйте один из этих способов:

#### Способ 1: Просмотр через rosout
```bash
# В отдельном терминале
rostopic echo /rosout
```

#### Способ 2: Просмотр через rqt_console
```bash
rqt_console
```

#### Способ 3: Просмотр логов из файла
```bash
# Найдите файл логов
ls -la ~/.ros/log/latest/

# Смотрите последние логи
tail -f ~/.ros/log/latest/*.log | grep -E "IMU|Auto-balance|resonance"
```

### 4. Фильтрация только наших логов

```bash
# Только логи с эмодзи (если терминал поддерживает)
rostopic echo /rosout 2>/dev/null | grep -E "🤖|✅|⚠️|📊|📡|❌"

# Или по ключевым словам
rostopic echo /rosout 2>/dev/null | grep -E "IMU|Auto-balance|resonance|adaptation"
```

### 5. Проверка, что логирование работает

Добавьте тестовый лог в начало `imu_callback`:

```python
rospy.loginfo("TEST: IMU callback called!")
```

Если этот лог не появляется, значит:
- Либо IMU топик не публикуется
- Либо нода не подписана на топик
- Либо топик называется по-другому

### 6. Проверка имени топика IMU

```bash
# Посмотрите, какие топики IMU есть
rostopic list | grep imu

# Проверьте, на какой топик подписана нода
rosnode info /joystick_control | grep -A 5 Subscribers
```

Если топик называется не `/imu`, нужно изменить в коде:
```python
self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
```

## Что должно появиться в логах:

### При запуске ноды:
```
============================================================
🤖 IMU-based Gait Adaptation System Initialized
   Auto-balance: enabled=True
   Auto-balance thresholds: forward=14.3°, backward=-14.3°
   ...
============================================================
```

### При первом получении IMU данных:
```
✅ IMU data received! Starting IMU-based adaptation system...
   First IMU reading - Pitch: 5.2°, Roll: -1.3°, Accel: X=0.15, Y=-0.32, Z=9.81
```

### Каждые 2 секунды:
```
============================================================
📡 IMU Status Report
   Orientation:
      Pitch: 0.120 rad (6.9°)
      Roll:  -0.050 rad (-2.9°)
      ...
============================================================
```

### При автоматической балансировке:
```
🤖 Auto-balance: Robot tilting backward (pitch=-0.28 rad, -16.0°), making backward step (consecutive: 1)
✅ Auto-balance step executed: backward (amplitude: 0.0050)
```

## Если логи все еще не видны:

1. **Проверьте уровень логирования ROS:**
```bash
rosparam get /rosconsole/min_severity
# Должно быть INFO или DEBUG
```

2. **Установите уровень логирования:**
```bash
rosparam set /rosconsole/min_severity INFO
```

3. **Перезапустите ноду**

4. **Проверьте, что нода действительно запущена:**
```bash
rosnode ping /joystick_control
```






