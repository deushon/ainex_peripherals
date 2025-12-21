# Решение проблемы с sensor_node.py

## 🔴 Проблема

Процесс `sensor_node.py` завершается с ошибкой (exit code 1), что приводит к остановке всего ROS launch файла, так как в `sensor_node.launch` указано `required="true"`.

```
REQUIRED process [sensor-5] has died!
process has died [pid 139240, exit code 1, cmd /home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/scripts/sensor_node.py]
Initiating shutdown!
```

## 🔍 Причины

1. **Датчик VL53L0X не подключен или не работает** - код пытается инициализировать датчик на строке 32 без обработки ошибок
2. **Ошибка при инициализации датчика** - `VL53L0X.VL53L0X()` может выбросить исключение
3. **Ошибка при вызове `start_ranging()`** - метод может не работать, если датчик не подключен
4. **Отсутствие обработки исключений** - код не обрабатывает ошибки при работе с датчиком

## ✅ Решения

### Решение 1: Сделать sensor_node необязательным (быстрое решение)

Измените файл `/home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/launch/sensor_node.launch`:

**Было:**
```xml
<node name="sensor" pkg="ainex_sdk" type="sensor_node.py" required="true" output="screen"/>
```

**Стало:**
```xml
<node name="sensor" pkg="ainex_sdk" type="sensor_node.py" required="false" output="screen"/>
```

Или удалите атрибут `required` (по умолчанию `false`):
```xml
<node name="sensor" pkg="ainex_sdk" type="sensor_node.py" output="screen"/>
```

**Плюсы:** Быстрое решение, система продолжит работать без датчика
**Минусы:** Датчик не будет работать, но система не упадет

---

### Решение 2: Добавить обработку ошибок в sensor_node.py (рекомендуется)

Добавьте обработку исключений в файл `/home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/scripts/sensor_node.py`:

```python
def __init__(self, name):
    self.name = name
    rospy.init_node(self.name)
    self.tof_thread = None
    self.enable_button = True
    self.tof = None  # Инициализировать как None
   
    self.frame = rospy.get_param("~frame_id", "range_finder")
    rospy.Service('/sensor/button/enable', SetBool, self.set_button_enable)
    rospy.Subscriber('/sensor/led/set_led_state', Bool, self.set_led_state_callback)
    button_pub = rospy.Publisher('/sensor/button/get_button_state', Bool, queue_size=1)
    self.tof_pub = rospy.Publisher("/sensor/tof/range", Range, queue_size=10)
    rospy.sleep(0.2)
   
    # Попытка инициализации датчика с обработкой ошибок
    try:
        self.tof = VL53L0X.VL53L0X()
        self.start_ranging()
        rospy.loginfo("VL53L0X sensor initialized successfully")
    except Exception as e:
        rospy.logwarn(f"Failed to initialize VL53L0X sensor: {e}")
        rospy.logwarn("Continuing without TOF sensor...")
        self.tof = None

    led.set_value(0)
    freq = rospy.get_param('~freq', 50)
    rate = rospy.Rate(freq)
    try:
        while not rospy.is_shutdown():
            led_value = os.popen('cat ' + self.led_path).read()
            if led_value != '':
                led.set_value(int(led_value))
                with open(self.led_path, 'w') as f:
                    f.write('')
            if self.enable_button:
                button_pub.publish(button.get_button_status())
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
```

И обновите метод `pub_tof_data`:

```python
def pub_tof_data(self, evt):
    if self.tof is None:
        return  # Пропустить публикацию, если датчик не инициализирован
    
    try:
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(25.0)
        msg.min_range = 0.005
        msg.max_range = 2.0

        msg.range = self.tof.get_distance() / 1000.0
        self.tof_pub.publish(msg)
    except Exception as e:
        rospy.logwarn(f"Error reading TOF sensor: {e}")
```

**Плюсы:** Система будет работать даже без датчика, но попытается его использовать если он доступен
**Минусы:** Требует изменения кода

---

### Решение 3: Проверить подключение датчика

Если датчик должен работать, проверьте:

1. **Физическое подключение:**
   ```bash
   # Проверить I2C устройства
   i2cdetect -y 1
   ```

2. **Права доступа:**
   ```bash
   # Добавить пользователя в группу i2c
   sudo usermod -a -G i2c ubuntu
   ```

3. **Проверить, что датчик виден:**
   ```bash
   # Проверить I2C шину
   ls -la /dev/i2c-*
   ```

---

### Решение 4: Временно отключить sensor_node

Если датчик не нужен, можно временно закомментировать его в `base.launch`:

Файл: `/home/ubuntu/ros_ws/src/ainex_bringup/launch/base.launch`

```xml
<?xml version="1.0"?>
<launch>
    <!--板载传感器-->
    <!-- Временно отключено из-за проблем с датчиком -->
    <!-- <include file="$(find ainex_sdk)/launch/sensor_node.launch" /> -->

    <!--步态控制节点-->
    <include file="$(find ainex_kinematics)/launch/ainex_controller.launch"/>

    <!--imu过滤-->
    <include file="$(find ainex_peripherals)/launch/imu.launch"/>

</launch>
```

---

## 🚀 Быстрое исправление (рекомендуется)

**Самый быстрый способ - сделать sensor_node необязательным:**

```bash
# Отредактировать launch файл
nano /home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/launch/sensor_node.launch

# Изменить строку:
# Было: required="true"
# Стало: required="false"
```

Или выполните команду:
```bash
sed -i 's/required="true"/required="false"/' /home/ubuntu/ros_ws/src/ainex_driver/ainex_sdk/launch/sensor_node.launch
```

После этого перезапустите ROS:
```bash
# Остановить
killall -9 roslaunch rosmaster rosout

# Запустить заново
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash
nohup roslaunch ainex_bringup bringup.launch > /tmp/roslaunch.log 2>&1 &
```

---

## 📝 Проверка после исправления

После применения исправления проверьте:

```bash
# Проверить, что ROS запущен
rosnode list

# Проверить логи
tail -f /tmp/roslaunch.log

# Проверить, что sensor_node не падает (если он все еще запускается)
rosnode info /sensor
```

---

## ⚠️ Важно

- Если датчик VL53L0X нужен для работы робота, используйте **Решение 2** (добавление обработки ошибок)
- Если датчик не критичен, используйте **Решение 1** (сделать необязательным)
- Если датчик не используется вообще, используйте **Решение 4** (отключить)

