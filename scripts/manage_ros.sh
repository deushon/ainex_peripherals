#!/bin/bash

# Скрипт для управления ROS пакетом ainex_peripherals
# Использование: ./manage_ros.sh [start|stop|restart|status|enable|disable]

# Проверка доступности systemd
SYSTEMD_AVAILABLE=false
if systemctl list-units &>/dev/null 2>&1; then
    SYSTEMD_AVAILABLE=true
    SERVICES=("start_app_node.service" "oled_display.service" "ros_node.service")
fi

function show_usage() {
    echo "Использование: $0 [команда]"
    echo ""
    echo "Команды:"
    echo "  start     - Запустить все ROS сервисы"
    echo "  stop      - Остановить все ROS сервисы"
    echo "  restart   - Перезапустить все ROS сервисы"
    echo "  status    - Показать статус всех сервисов"
    echo "  enable    - Включить автозапуск при перезагрузке"
    echo "  disable   - Отключить автозапуск при перезагрузке"
    echo "  kill      - Принудительно остановить все ROS процессы"
    echo "  check     - Проверить ROS узлы и топики"
    echo "  logs      - Показать логи сервисов"
    echo ""
}

function start_services() {
    if [ "$SYSTEMD_AVAILABLE" = true ]; then
        echo "🚀 Запуск ROS сервисов через systemd..."
        for service in "${SERVICES[@]}"; do
            echo "  Запуск $service..."
            sudo systemctl start "$service" 2>/dev/null && echo "    ✅ $service запущен" || echo "    ❌ Ошибка запуска $service"
        done
    else
        echo "🚀 Запуск ROS пакета без systemd..."
        echo "  Настройка окружения ROS..."
        source /opt/ros/noetic/setup.bash 2>/dev/null
        source /home/ubuntu/ros_ws/devel/setup.bash 2>/dev/null
        
        echo "  Запуск главного launch файла..."
        nohup roslaunch ainex_bringup bringup.launch > /tmp/roslaunch.log 2>&1 &
        ROSLAUNCH_PID=$!
        sleep 2
        if ps -p $ROSLAUNCH_PID > /dev/null; then
            echo "    ✅ ROS launch запущен (PID: $ROSLAUNCH_PID)"
        else
            echo "    ❌ Ошибка запуска ROS launch"
            echo "    Проверьте логи: tail -f /tmp/roslaunch.log"
        fi
        
        echo "  Запуск OLED дисплея..."
        nohup python3 /home/ubuntu/ros_ws/src/ainex_bringup/scripts/oled_display.py > /tmp/oled.log 2>&1 &
        OLED_PID=$!
        sleep 1
        if ps -p $OLED_PID > /dev/null; then
            echo "    ✅ OLED дисплей запущен (PID: $OLED_PID)"
        else
            echo "    ⚠️  OLED дисплей не запустился (возможно, не требуется)"
        fi
    fi
    echo "Готово!"
}

function stop_services() {
    if [ "$SYSTEMD_AVAILABLE" = true ]; then
        echo "🛑 Остановка ROS сервисов через systemd..."
        for service in "${SERVICES[@]}"; do
            echo "  Остановка $service..."
            sudo systemctl stop "$service" 2>/dev/null && echo "    ✅ $service остановлен" || echo "    ❌ Ошибка остановки $service"
        done
    else
        echo "🛑 Остановка ROS процессов без systemd..."
        echo "  Остановка roslaunch..."
        killall -9 roslaunch 2>/dev/null && echo "    ✅ roslaunch остановлен" || echo "    ⚠️  roslaunch не найден"
        
        echo "  Остановка rosmaster..."
        killall -9 rosmaster 2>/dev/null && echo "    ✅ rosmaster остановлен" || echo "    ⚠️  rosmaster не найден"
        
        echo "  Остановка rosout..."
        killall -9 rosout 2>/dev/null && echo "    ✅ rosout остановлен" || echo "    ⚠️  rosout не найден"
        
        echo "  Остановка ROS узлов..."
        pkill -f "ros.*node" 2>/dev/null && echo "    ✅ ROS узлы остановлены" || echo "    ⚠️  ROS узлы не найдены"
        
        echo "  Остановка OLED дисплея..."
        pkill -f "oled_display" 2>/dev/null && echo "    ✅ OLED дисплей остановлен" || echo "    ⚠️  OLED дисплей не найден"
        
        echo "  Остановка дополнительных процессов..."
        pkill -f "topici_list" 2>/dev/null && echo "    ✅ Дополнительные процессы остановлены" || echo "    ⚠️  Дополнительные процессы не найдены"
    fi
    echo "Готово!"
}

function restart_services() {
    echo "🔄 Перезапуск ROS сервисов..."
    stop_services
    sleep 2
    start_services
}

function show_status() {
    echo "📊 Статус ROS системы:"
    echo ""
    
    if [ "$SYSTEMD_AVAILABLE" = true ]; then
        for service in "${SERVICES[@]}"; do
            echo "=== $service ==="
            sudo systemctl status "$service" --no-pager -l | head -10
            echo ""
        done
    else
        echo "=== ROS процессы ==="
        ps aux | grep -E "roslaunch|rosmaster|rosout|oled_display" | grep -v grep || echo "ROS процессы не найдены"
        echo ""
    fi
    
    echo "=== ROS узлы ==="
    if command -v rosnode &> /dev/null 2>&1; then
        source /opt/ros/noetic/setup.bash 2>/dev/null
        source /home/ubuntu/ros_ws/devel/setup.bash 2>/dev/null
        if rostopic list &> /dev/null; then
            rosnode list 2>/dev/null || echo "ROS master не запущен"
        else
            echo "ROS master не запущен"
        fi
    else
        echo "ROS окружение не настроено"
    fi
    echo ""
}

function enable_autostart() {
    if [ "$SYSTEMD_AVAILABLE" = true ]; then
        echo "🔧 Включение автозапуска через systemd..."
        for service in "${SERVICES[@]}"; do
            echo "  Включение автозапуска для $service..."
            sudo systemctl enable "$service" 2>/dev/null && echo "    ✅ Автозапуск включен для $service" || echo "    ❌ Ошибка для $service"
        done
    else
        echo "⚠️  systemd недоступен. Автозапуск должен быть настроен через:"
        echo "   - Docker entrypoint скрипт"
        echo "   - ~/.bashrc или ~/.profile"
        echo "   - Supervisor или другой init system"
    fi
    echo "Готово!"
}

function disable_autostart() {
    if [ "$SYSTEMD_AVAILABLE" = true ]; then
        echo "🔧 Отключение автозапуска через systemd..."
        for service in "${SERVICES[@]}"; do
            echo "  Отключение автозапуска для $service..."
            sudo systemctl disable "$service" 2>/dev/null && echo "    ✅ Автозапуск отключен для $service" || echo "    ❌ Ошибка для $service"
        done
    else
        echo "⚠️  systemd недоступен. Отключите автозапуск в:"
        echo "   - Docker entrypoint скрипте"
        echo "   - ~/.bashrc или ~/.profile"
        echo "   - Supervisor конфигурации"
    fi
    echo "Готово!"
}

function kill_ros_processes() {
    echo "⚠️  Принудительная остановка всех ROS процессов..."
    echo "  Остановка roslaunch..."
    killall -9 roslaunch 2>/dev/null
    echo "  Остановка rosmaster..."
    killall -9 rosmaster 2>/dev/null
    echo "  Остановка rosout..."
    killall -9 rosout 2>/dev/null
    echo "  Остановка ROS узлов..."
    pkill -f "ros.*node" 2>/dev/null
    echo "✅ Все ROS процессы остановлены"
}

function check_ros() {
    echo "🔍 Проверка ROS системы..."
    echo ""
    
    # Проверка ROS master
    if rostopic list &> /dev/null; then
        echo "✅ ROS master запущен"
        echo ""
        
        echo "=== Запущенные узлы ==="
        rosnode list 2>/dev/null | head -20
        echo ""
        
        echo "=== Активные топики ==="
        rostopic list 2>/dev/null | head -20
        echo ""
        
        echo "=== Проверка основных топиков ==="
        echo -n "  /imu: "
        timeout 1 rostopic hz /imu &> /dev/null && echo "✅ активен" || echo "❌ не активен"
        
        echo -n "  /joy: "
        timeout 1 rostopic hz /joy &> /dev/null && echo "✅ активен" || echo "❌ не активен"
        
        echo -n "  /camera/image_raw: "
        timeout 1 rostopic hz /camera/image_raw &> /dev/null && echo "✅ активен" || echo "❌ не активен"
    else
        echo "❌ ROS master не запущен"
        echo "   Запустите: $0 start"
    fi
    echo ""
}

function show_logs() {
    echo "📝 Логи:"
    echo ""
    
    if [ "$SYSTEMD_AVAILABLE" = true ]; then
        for service in "${SERVICES[@]}"; do
            echo "=== Логи $service ==="
            sudo journalctl -u "$service" -n 50 --no-pager
            echo ""
        done
    else
        echo "=== Логи roslaunch ==="
        if [ -f /tmp/roslaunch.log ]; then
            tail -50 /tmp/roslaunch.log
        else
            echo "Лог файл не найден"
        fi
        echo ""
        
        echo "=== Логи OLED ==="
        if [ -f /tmp/oled.log ]; then
            tail -50 /tmp/oled.log
        else
            echo "Лог файл не найден"
        fi
        echo ""
        
        echo "=== ROS логи ==="
        if [ -d ~/.ros/log/latest ]; then
            ls -lt ~/.ros/log/latest/*.log 2>/dev/null | head -5 | while read line; do
                file=$(echo "$line" | awk '{print $NF}')
                echo "--- $file ---"
                tail -20 "$file" 2>/dev/null
                echo ""
            done
        else
            echo "ROS логи не найдены"
        fi
    fi
}

# Основная логика
case "$1" in
    start)
        start_services
        ;;
    stop)
        stop_services
        ;;
    restart)
        restart_services
        ;;
    status)
        show_status
        ;;
    enable)
        enable_autostart
        ;;
    disable)
        disable_autostart
        ;;
    kill)
        kill_ros_processes
        ;;
    check)
        check_ros
        ;;
    logs)
        show_logs
        ;;
    *)
        show_usage
        exit 1
        ;;
esac

exit 0

