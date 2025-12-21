# Удаление датчика расстояния VL53L0X

## ✅ Что сделано

Датчик расстояния VL53L0X (sensor_node) **отключен** в файле `/home/ubuntu/ros_ws/src/ainex_bringup/launch/base.launch`

## ✅ Что осталось активным

### IMU (Inertial Measurement Unit) ✅
- **Файл:** `/home/ubuntu/ros_ws/src/ainex_peripherals/launch/imu.launch`
- **Компоненты:**
  - `imu_calib` - калибровка IMU данных
  - `imu_filter` - фильтрация IMU данных (complementary filter)
  - Публикует топик: `/imu`

### MAG (Магнитометр) ✅
- **Включен в:** `imu.launch`
- **Параметр:** `use_mag="true"` (строка 4)
- **Топик магнитометра:** `/ros_robot_controller/mag`
- Используется фильтром `imu_filter` для улучшения ориентации

## 📝 Изменения

**Файл:** `/home/ubuntu/ros_ws/src/ainex_bringup/launch/base.launch`

**Было:**
```xml
<!--板载传感器-->
<include file="$(find ainex_sdk)/launch/sensor_node.launch" />
```

**Стало:**
```xml
<!--板载传感器-->
<!-- Датчик расстояния VL53L0X отключен - не используется -->
<!-- <include file="$(find ainex_sdk)/launch/sensor_node.launch" /> -->
```

## 🚀 Перезапуск

После изменений нужно перезапустить ROS:

```bash
# Остановить текущий запуск
killall -9 roslaunch rosmaster rosout
pkill -f "ros.*node"

# Подождать
sleep 2

# Запустить заново
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash
nohup roslaunch ainex_bringup bringup.launch > /tmp/roslaunch.log 2>&1 &
```

## ✅ Проверка после перезапуска

```bash
# Проверить, что sensor_node не запущен
rosnode list | grep sensor
# Должно быть пусто

# Проверить, что IMU работает
rostopic hz /imu
# Должна быть частота публикации

# Проверить, что магнитометр работает
rostopic echo /ros_robot_controller/mag
# Должны приходить данные

# Проверить логи на ошибки
tail -f /tmp/roslaunch.log
```

## 📊 Активные датчики после изменений

✅ **IMU** - активен (топик `/imu`)
✅ **MAG (магнитометр)** - активен (топик `/ros_robot_controller/mag`)
❌ **VL53L0X (датчик расстояния)** - отключен

## ⚠️ Важно

- IMU и MAG критически важны для работы робота и остаются активными
- Датчик расстояния больше не будет вызывать ошибки при запуске
- Если в будущем понадобится датчик расстояния, просто раскомментируйте строку в `base.launch`

