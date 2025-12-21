# Быстрые команды для управления ROS пакетом

## ⚠️ ВАЖНО: Если systemd недоступен (Docker контейнер)

Используйте команды из раздела "Без systemd" ниже.

---

## 🚀 Запуск

### Без systemd (для контейнеров):
```bash
source /opt/ros/noetic/setup.bash && source /home/ubuntu/ros_ws/devel/setup.bash
nohup roslaunch ainex_bringup bringup.launch > /tmp/roslaunch.log 2>&1 &
```

### С systemd (если доступен):

```bash
# Запустить все через systemd
sudo systemctl start start_app_node.service
sudo systemctl start oled_display.service
sudo systemctl start ros_node.service

# Или запустить напрямую
source /opt/ros/noetic/setup.bash && source /home/ubuntu/ros_ws/devel/setup.bash && roslaunch ainex_bringup bringup.launch
```

## 🛑 Остановка

### Без systemd (для контейнеров):
```bash
killall -9 roslaunch rosmaster rosout
pkill -f "ros.*node"
pkill -f "oled_display"
```

### С systemd (если доступен):
```bash
sudo systemctl stop start_app_node.service
sudo systemctl stop oled_display.service
sudo systemctl stop ros_node.service
```

## 🔄 Перезапуск

```bash
# Перезапустить через systemd
sudo systemctl restart start_app_node.service
sudo systemctl restart oled_display.service
sudo systemctl restart ros_node.service

# Ручной перезапуск после принудительной остановки
killall -9 roslaunch rosmaster rosout && sleep 3 && sudo systemctl start start_app_node.service
```

## 📊 Проверка статуса

```bash
# Статус сервисов
sudo systemctl status start_app_node.service
sudo systemctl status oled_display.service

# ROS узлы
rosnode list

# ROS топики
rostopic list

# Процессы
ps aux | grep ros | grep -v grep
```

## 🔧 Автозапуск

```bash
# Включить автозапуск
sudo systemctl enable start_app_node.service
sudo systemctl enable oled_display.service

# Отключить автозапуск
sudo systemctl disable start_app_node.service
sudo systemctl disable oled_display.service

# Проверить статус
systemctl is-enabled start_app_node.service
```

## 📝 Логи

```bash
# Логи systemd
sudo journalctl -u start_app_node.service -f

# Логи ROS
tail -f ~/.ros/log/latest/*.log
```

