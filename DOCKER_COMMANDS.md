# Команды для управления ROS пакетом в Docker контейнере

## ⚠️ Важно: systemd недоступен

В вашей системе systemd не работает (вероятно, Docker контейнер). Используйте команды ниже.

---

## 🚀 Запуск пакета

```bash
# Настроить окружение ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash

# Запустить главный launch файл (в фоне)
nohup roslaunch ainex_bringup bringup.launch > /tmp/roslaunch.log 2>&1 &

# Запустить OLED дисплей (если нужен)
nohup python3 /home/ubuntu/ros_ws/src/ainex_bringup/scripts/oled_display.py > /tmp/oled.log 2>&1 &
```

**Или используйте скрипт:**
```bash
cd /home/ubuntu/ros_ws/src/ainex_peripherals/scripts
./manage_ros.sh start
```

---

## 🛑 Остановка пакета

```bash
# Остановить все ROS процессы
killall -9 roslaunch rosmaster rosout
pkill -f "ros.*node"
pkill -f "oled_display"
pkill -f "topici_list"
```

**Или используйте скрипт:**
```bash
cd /home/ubuntu/ros_ws/src/ainex_peripherals/scripts
./manage_ros.sh stop
```

---

## 🔄 Перезапуск пакета

```bash
# 1. Остановить
killall -9 roslaunch rosmaster rosout
pkill -f "ros.*node"

# 2. Подождать
sleep 3

# 3. Запустить заново
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash
nohup roslaunch ainex_bringup bringup.launch > /tmp/roslaunch.log 2>&1 &
```

**Или используйте скрипт:**
```bash
cd /home/ubuntu/ros_ws/src/ainex_peripherals/scripts
./manage_ros.sh restart
```

---

## 📊 Проверка статуса

```bash
# Проверить запущенные ROS процессы
ps aux | grep -E "ros|launch" | grep -v grep

# Проверить ROS узлы (если ROS master работает)
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash
rosnode list

# Проверить топики
rostopic list
```

**Или используйте скрипт:**
```bash
cd /home/ubuntu/ros_ws/src/ainex_peripherals/scripts
./manage_ros.sh status
./manage_ros.sh check
```

---

## 📝 Просмотр логов

```bash
# Логи roslaunch
tail -f /tmp/roslaunch.log

# Логи OLED
tail -f /tmp/oled.log

# ROS логи
tail -f ~/.ros/log/latest/*.log
```

**Или используйте скрипт:**
```bash
cd /home/ubuntu/ros_ws/src/ainex_peripherals/scripts
./manage_ros.sh logs
```

---

## 🔧 Удобный скрипт управления

Используйте скрипт `manage_ros.sh` для всех операций:

```bash
cd /home/ubuntu/ros_ws/src/ainex_peripherals/scripts

# Запустить
./manage_ros.sh start

# Остановить
./manage_ros.sh stop

# Перезапустить
./manage_ros.sh restart

# Проверить статус
./manage_ros.sh status

# Проверить ROS узлы и топики
./manage_ros.sh check

# Показать логи
./manage_ros.sh logs
```

---

## 📚 Дополнительная информация

Подробное руководство: `STARTUP_GUIDE.md`
Краткая шпаргалка: `QUICK_COMMANDS.md`

