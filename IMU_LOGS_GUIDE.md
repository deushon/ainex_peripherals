# Руководство по просмотру логов IMU и адаптации походки

## Как просматривать логи при запущенном ROS

### 1. Просмотр всех логов ноды в реальном времени

```bash
# Просмотр всех логов ноды joystick_control
rosrun rqt_console rqt_console

# Или через командную строку (фильтрация по уровню)
rosnode info /joystick_control
```

### 2. Просмотр логов через командную строку

```bash
# Просмотр всех логов ROS (все ноды)
roscd && tail -f ~/.ros/log/latest/*.log

# Просмотр логов конкретной ноды в реальном времени
rostopic echo /rosout | grep joystick_control

# Или используя rosnode
rosnode info /joystick_control
```

### 3. Фильтрация логов по уровню важности

В ROS есть несколько уровней логирования:
- `rospy.logdebug()` - отладочная информация (по умолчанию не показывается)
- `rospy.loginfo()` - информационные сообщения
- `rospy.logwarn()` - предупреждения
- `rospy.logerr()` - ошибки

Для просмотра debug логов нужно установить уровень логирования:

```bash
# В другом терминале перед запуском ноды
export ROSCONSOLE_MIN_SEVERITY=DEBUG

# Или в launch файле добавить параметр
<param name="log_level" value="DEBUG"/>
```

### 4. Просмотр логов через rqt

```bash
# Запустить rqt с консолью
rqt

# Затем: Plugins -> Logging -> Console
```

### 5. Просмотр только наших логов (с эмодзи для удобства)

Наши логи содержат специальные маркеры:
- 🤖 - Автоматическая балансировка
- ✅ - Успешные операции
- ⚠️ - Критические предупреждения
- 📊 - Статистика и метрики
- 📡 - Статус IMU
- ❌ - Ошибки

```bash
# Фильтрация по эмодзи (если терминал поддерживает)
rostopic echo /rosout | grep -E "🤖|✅|⚠️|📊|📡|❌"

# Или фильтрация по ключевым словам
rostopic echo /rosout | grep -E "Auto-balance|resonance|IMU Status|adaptation"
```

## Типы логов в системе

### 1. Логи автоматической балансировки

**Когда появляются:**
- При обнаружении наклона робота в покое
- При выполнении автоматического шага
- При стабилизации робота

**Примеры:**
```
🤖 Auto-balance: Robot tilting backward (pitch=-0.28 rad, -16.0°), making backward step (consecutive: 1)
✅ Auto-balance step executed: backward (amplitude: 0.0050)
✅ Auto-balance: Robot stable (pitch=0.05 rad, 2.9°), resetting step counter
```

### 2. Логи обнаружения резонанса

**Когда появляются:**
- При обнаружении высокой амплитуды качания
- При критическом резонансе
- При восстановлении нормальных параметров

**Примеры:**
```
📊 High amplitude detected (amplitude=0.42 m/s²), adaptation factor: 0.75
   Amplitudes - X: 0.15, Y: 0.38, Z: 0.22, Pitch: 0.12 rad
⚠️ CRITICAL resonance detected! (amplitude=0.65 m/s²), reducing step by 50% (factor: 0.50)
✅ Recovering from resonance (amplitude=0.25 m/s²), adaptation factor: 0.95
```

### 3. Логи статуса IMU (каждые 2 секунды)

**Содержат:**
- Текущие углы ориентации (pitch, roll, yaw)
- Ускорения по осям (ax, ay, az)
- Сглаженное значение pitch
- Состояние робота и системы балансировки
- Фактор адаптации

**Пример:**
```
📡 IMU Status:
   Orientation - Pitch: 0.12 rad (6.9°), Roll: -0.05 rad (-2.9°), Yaw: 1.57 rad (90.0°)
   Acceleration - X: 0.15, Y: -0.32, Z: 9.81 m/s²
   Smoothed Pitch: 0.11 rad (6.3°)
   Robot State: stand, Status: stop, Update Param: False
   Auto-balance: enabled=True, consecutive_steps=0
   Resonance: enabled=True, adaptation_factor=1.00
   IMU History Size: 50
```

## Настройка уровня логирования

### В коде (для отладки)

Измените уровень логирования в `imu_callback`:

```python
# Для более детальных логов
rospy.logdebug()  # Вместо loginfo для детальной отладки
```

### Через параметры ROS

```bash
# Установить уровень логирования перед запуском
rosparam set /joystick_control/log_level DEBUG
```

## Полезные команды для отладки

```bash
# Просмотр топика IMU напрямую
rostopic echo /imu

# Просмотр частоты обновления IMU
rostopic hz /imu

# Просмотр информации о ноде
rosnode info /joystick_control

# Просмотр всех параметров ноды
rosparam list | grep joystick_control

# Просмотр конкретного параметра
rosparam get /joystick_control/robot_id
```

## Оптимизация параметров на основе логов

### Что смотреть:

1. **Автоматическая балансировка:**
   - Если шаги выполняются слишком часто → увеличить `auto_step_interval`
   - Если шаги не помогают → увеличить `auto_step_amplitude`
   - Если балансировка не срабатывает → уменьшить пороги `tilt_threshold_*`

2. **Обнаружение резонанса:**
   - Если адаптация срабатывает слишком часто → увеличить `max_safe_amplitude`
   - Если адаптация не помогает → уменьшить `critical_amplitude` или изменить логику
   - Смотреть на амплитуды по осям - какая ось больше всего влияет

3. **Стабильность:**
   - Смотреть на "Smoothed Pitch" - должно быть стабильным
   - Если большая вариация → увеличить `auto_balance_stable_time`

## Сохранение логов в файл

```bash
# Сохранить все логи в файл
rosrun rosout rosout > ~/imu_logs.txt

# Или только логи нашей ноды
rostopic echo /rosout | grep joystick_control > ~/joystick_control_logs.txt
```









