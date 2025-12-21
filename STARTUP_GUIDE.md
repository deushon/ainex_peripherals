# Руководство по запуску и остановке ROS пакета ainex_peripherals

## 📋 Обзор

ROS пакет `ainex_peripherals` запускается через главный launch файл `bringup.launch` в пакете `ainex_bringup`. 

**⚠️ ВАЖНО**: В вашей системе systemd недоступен (вероятно, контейнер Docker). Используйте команды из раздела "Управление без systemd" ниже.

---

## 🔄 Порядок запуска при перезагрузке

### Автоматический запуск через systemd

При перезагрузке системы автоматически запускаются следующие сервисы:

1. **`start_app_node.service`** - Главный сервис, запускающий весь ROS стек
   - Запускает: `roslaunch ainex_bringup bringup.launch`
   - Включает в себя:
     - Камера (`usb_cam_with_calib.launch`)
     - Базовые узлы (`base.launch`):
       - Датчики (`sensor_node.launch`)
       - Кинематика (`ainex_controller.launch`)
       - IMU (`imu.launch`)
     - Управление джойстиком (`joystick_control.launch`)
     - Web video server
     - ROSBridge для связи с приложением
     - Дополнительные узлы приложения

2. **`oled_display.service`** - Сервис для OLED дисплея
   - Запускает: `python3 /home/ubuntu/ros_ws/src/ainex_bringup/scripts/oled_display.py`

3. **`ros_node.service`** - Дополнительный ROS узел
   - Запускает: `rosrun proverka_nod topici_list.py`

---

## 🚀 Ручной запуск пакета

### ⚠️ УПРАВЛЕНИЕ БЕЗ SYSTEMD (для контейнеров Docker)

**Если systemd недоступен** (как в вашем случае), используйте прямые команды:

#### Запуск всего пакета:
```bash
# Настроить окружение ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash

# Запустить главный launch файл (в фоне)
nohup roslaunch ainex_bringup bringup.launch > /tmp/roslaunch.log 2>&1 &

# Или запустить OLED дисплей отдельно
nohup python3 /home/ubuntu/ros_ws/src/ainex_bringup/scripts/oled_display.py > /tmp/oled.log 2>&1 &
```

#### Остановка всего пакета:
```bash
# Остановить все ROS процессы
killall -9 roslaunch
killall -9 rosmaster
killall -9 rosout
pkill -f "ros.*node"
pkill -f "oled_display"
pkill -f "topici_list"

# Или более мягкая остановка (сначала попытка корректной остановки)
pkill -TERM roslaunch
sleep 2
pkill -9 roslaunch rosmaster rosout
```

#### Перезапуск:
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

#### Проверка статуса:
```bash
# Проверить запущенные ROS процессы
ps aux | grep -E "ros|launch" | grep -v grep

# Проверить ROS узлы (если ROS master работает)
rosnode list

# Проверить топики
rostopic list
```

---

### Вариант 1: Запуск через systemd (только если systemd доступен)

```bash
# Запустить все сервисы
sudo systemctl start start_app_node.service
sudo systemctl start oled_display.service
sudo systemctl start ros_node.service

# Проверить статус
sudo systemctl status start_app_node.service
sudo systemctl status oled_display.service
sudo systemctl status ros_node.service
```

### Вариант 2: Запуск через roslaunch напрямую

```bash
# 1. Настроить окружение ROS
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash

# 2. Запустить главный launch файл
roslaunch ainex_bringup bringup.launch
```

### Вариант 3: Запуск отдельных компонентов

Если нужно запустить только определенные компоненты:

```bash
# Настроить окружение
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash

# Запустить только камеру
roslaunch ainex_peripherals usb_cam_with_calib.launch

# Запустить только IMU
roslaunch ainex_peripherals imu.launch

# Запустить только управление джойстиком
roslaunch ainex_peripherals joystick_control.launch robot_id:=robot_2

# Запустить базовые узлы (сенсоры, кинематика, IMU)
roslaunch ainex_bringup base.launch
```

---

## 🛑 Остановка пакета

### ⚠️ ОСТАНОВКА БЕЗ SYSTEMD (для контейнеров Docker)

**Если systemd недоступен**, используйте прямые команды:

```bash
# Принудительная остановка всех ROS процессов
killall -9 roslaunch rosmaster rosout
pkill -f "ros.*node"
pkill -f "oled_display"
pkill -f "topici_list"

# Проверить, что все остановлено
ps aux | grep -E "ros|launch" | grep -v grep
```

### Вариант 1: Остановка через systemd (только если systemd доступен)

```bash
# Остановить все сервисы
sudo systemctl stop start_app_node.service
sudo systemctl stop oled_display.service
sudo systemctl stop ros_node.service

# Проверить, что все остановлено
sudo systemctl status start_app_node.service
sudo systemctl status oled_display.service
sudo systemctl status ros_node.service
```

### Вариант 2: Принудительная остановка всех ROS процессов

```bash
# Остановить все ROS процессы
killall -9 roslaunch
killall -9 rosmaster
killall -9 rosout
killall -9 python3  # Будет остановлено много процессов, будьте осторожны!

# Или более безопасный вариант - остановить только ROS процессы
pkill -f roslaunch
pkill -f rosmaster
pkill -f "ros.*node"
```

### Вариант 3: Остановка через rosnode (если ROS master работает)

```bash
# Посмотреть все запущенные узлы
rosnode list

# Остановить конкретный узел
rosnode kill /joystick_control
rosnode kill /camera
rosnode kill /imu_filter

# Остановить ROS master (остановит все узлы)
killall rosmaster
```

---

## 🔄 Перезапуск пакета

### Через systemd

```bash
# Перезапустить все сервисы
sudo systemctl restart start_app_node.service
sudo systemctl restart oled_display.service
sudo systemctl restart ros_node.service
```

### Ручной перезапуск (после принудительной остановки)

```bash
# 1. Убедиться, что все процессы остановлены
killall -9 roslaunch rosmaster rosout
pkill -f "ros.*node"

# 2. Подождать несколько секунд
sleep 3

# 3. Запустить заново через systemd
sudo systemctl start start_app_node.service
sudo systemctl start oled_display.service
sudo systemctl start ros_node.service

# Или запустить напрямую через roslaunch
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash
roslaunch ainex_bringup bringup.launch
```

---

## 📊 Проверка статуса

### Проверка systemd сервисов

```bash
# Статус всех ROS сервисов
sudo systemctl status start_app_node.service
sudo systemctl status oled_display.service
sudo systemctl status ros_node.service

# Просмотр логов
sudo journalctl -u start_app_node.service -f
sudo journalctl -u oled_display.service -f
```

### Проверка ROS узлов

```bash
# Список всех запущенных узлов
rosnode list

# Информация о конкретном узле
rosnode info /joystick_control

# Проверка, что узел работает
rosnode ping /joystick_control
```

### Проверка ROS топиков

```bash
# Список всех топиков
rostopic list

# Проверка частоты публикации
rostopic hz /imu
rostopic hz /joy
rostopic hz /camera/image_raw

# Просмотр данных топика
rostopic echo /imu
rostopic echo /joy
```

### Проверка процессов

```bash
# Все ROS процессы
ps aux | grep -E "ros|launch" | grep -v grep

# Конкретный процесс
ps aux | grep roslaunch
ps aux | grep rosmaster
```

---

## 🔧 Управление автозапуском

### Включить автозапуск при перезагрузке

```bash
# Включить автозапуск (постоянно)
sudo systemctl enable start_app_node.service
sudo systemctl enable oled_display.service
sudo systemctl enable ros_node.service
```

### Отключить автозапуск при перезагрузке

```bash
# Отключить автозапуск (постоянно)
sudo systemctl disable start_app_node.service
sudo systemctl disable oled_display.service
sudo systemctl disable ros_node.service
```

### Проверить, включен ли автозапуск

```bash
# Проверить статус автозапуска
systemctl is-enabled start_app_node.service
systemctl is-enabled oled_display.service
systemctl is-enabled ros_node.service
```

---

## 📝 Структура launch файлов

Главный launch файл `bringup.launch` включает в себя:

```
bringup.launch
├── usb_cam_with_calib.launch (камера)
│   ├── usb_cam.launch
│   └── image_calib.launch
├── base.launch (базовые узлы)
│   ├── sensor_node.launch (датчики)
│   ├── ainex_controller.launch (кинематика)
│   └── imu.launch (IMU фильтрация)
├── joystick_control.launch (управление джойстиком)
├── web_video_server (веб-сервер видео)
├── rosbridge.launch (ROS Bridge)
├── start.launch (приложение)
├── teleop_fetch.launch (VR телеоперация)
└── topici_list.launch (проверка топиков)
```

---

## ⚠️ Важные замечания

1. **Порядок запуска**: ROS master должен запуститься первым. При использовании systemd это происходит автоматически через `roslaunch`.

2. **Зависимости**: 
   - Камера требует наличие устройства `/dev/usb_cam`
   - Джойстик требует наличие устройства `/dev/input/js0`
   - Serial порт для Arduino требует наличие устройства `/dev/ttyUSB0` (или другого, указанного в параметрах)

3. **Переменные окружения**: При ручном запуске обязательно нужно настроить ROS окружение:
   ```bash
   source /opt/ros/noetic/setup.bash
   source /home/ubuntu/ros_ws/devel/setup.bash
   ```

4. **Логи**: Логи systemd сервисов можно просмотреть через:
   ```bash
   sudo journalctl -u start_app_node.service -n 100
   ```

5. **Проверка перед остановкой**: Перед принудительной остановкой проверьте, какие процессы запущены:
   ```bash
   ps aux | grep ros
   ```

---

## 🆘 Устранение проблем

### Проблема: Сервис не запускается

```bash
# Проверить логи
sudo journalctl -u start_app_node.service -n 50

# Проверить права доступа к файлам
ls -la /home/ubuntu/ros_ws/src/ainex_bringup/scripts/

# Проверить, что ROS workspace собран
cd /home/ubuntu/ros_ws
catkin_make  # или catkin build
```

### Проблема: ROS master не запускается

```bash
# Проверить, не запущен ли уже ROS master
ps aux | grep rosmaster

# Если запущен, остановить
killall rosmaster

# Проверить порт 11311
netstat -tuln | grep 11311
```

### Проблема: Узлы не запускаются

```bash
# Проверить, что ROS master работает
rostopic list

# Если команда не работает, запустить ROS master вручную
roscore

# В другом терминале запустить узлы
roslaunch ainex_bringup bringup.launch
```

---

## 📚 Дополнительные команды

### Просмотр всех systemd сервисов ROS

```bash
systemctl list-units --type=service | grep -E "ros|start_app|oled"
```

### Просмотр логов в реальном времени

```bash
# Логи systemd
sudo journalctl -u start_app_node.service -f

# Логи ROS (если запущено через roslaunch)
tail -f ~/.ros/log/latest/*.log
```

### Очистка логов ROS

```bash
rm -rf ~/.ros/log/*
```

---

**Дата создания**: 2025
**Версия**: 1.0

