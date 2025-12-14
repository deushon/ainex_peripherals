#!/bin/bash
# encoding: utf-8
#
# Отладочный скрипт для проверки физических положений осей робота
# Выводит данные IMU в реальном времени для проверки соответствия физических движений осям
#

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Отладка осей IMU робота${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo -e "${YELLOW}Инструкции:${NC}"
echo "1. Убедитесь, что ROS запущен и топик /IMU активен"
echo "2. Наблюдайте за значениями при физическом перемещении робота:"
echo "   - Наклоните робота ВПЕРЕД - проверьте pitch"
echo "   - Наклоните робота НАЗАД - проверьте pitch"
echo "   - Наклоните робота ВЛЕВО - проверьте roll"
echo "   - Наклоните робота ВПРАВО - проверьте roll"
echo "   - Поверните робота ВЛЕВО - проверьте yaw"
echo "   - Поверните робота ВПРАВО - проверьте yaw"
echo ""
echo -e "${YELLOW}Стандартная система координат для роботов:${NC}"
echo "  X: вперед (forward)"
echo "  Y: влево (left)"
echo "  Z: вверх (up)"
echo ""
echo -e "${YELLOW}Нажмите Ctrl+C для выхода${NC}"
echo ""
echo -e "${CYAN}========================================${NC}"
echo ""

# Проверяем наличие топика
if ! rostopic list | grep -q "imu"; then
    echo -e "${RED}ОШИБКА: Топик /imu не найден!${NC}"
    echo "Убедитесь, что:"
    echo "  1. ROS master запущен (roscore)"
    echo "  2. IMU нода запущена"
    echo "  3. Топик /imu публикуется"
    exit 1
fi

# Функция для форматирования числа
format_number() {
    printf "%8.3f" "$1"
}

# Функция для определения знака
get_sign() {
    if (( $(echo "$1 > 0" | bc -l) )); then
        echo "+"
    elif (( $(echo "$1 < 0" | bc -l) )); then
        echo "-"
    else
        echo "0"
    fi
}

# Основной цикл обработки данных
rostopic echo /imu -n 1 | while IFS= read -r line; do
    # Пропускаем пустые строки и заголовки
    if [[ -z "$line" ]] || [[ "$line" == "---" ]]; then
        continue
    fi
    
    # Парсим данные
    if [[ "$line" =~ orientation: ]]; then
        read -r qx_line
        read -r qy_line
        read -r qz_line
        read -r qw_line
        
        qx=$(echo "$qx_line" | grep -oP 'x: \K[+-]?[0-9]+\.?[0-9]*')
        qy=$(echo "$qy_line" | grep -oP 'y: \K[+-]?[0-9]+\.?[0-9]*')
        qz=$(echo "$qz_line" | grep -oP 'z: \K[+-]?[0-9]+\.?[0-9]*')
        qw=$(echo "$qw_line" | grep -oP 'w: \K[+-]?[0-9]+\.?[0-9]*')
        
        # Вычисляем углы Эйлера из quaternion
        # Roll (X-axis rotation)
        sinr_cosp=$(echo "2 * ($qw * $qx + $qy * $qz)" | bc -l)
        cosr_cosp=$(echo "1 - 2 * ($qx * $qx + $qy * $qy)" | bc -l)
        roll_rad=$(echo "a($sinr_cosp / $cosr_cosp)" | bc -l)
        roll_deg=$(echo "$roll_rad * 180 / 3.14159265359" | bc -l)
        
        # Pitch (Y-axis rotation)
        sinp=$(echo "2 * ($qw * $qy - $qz * $qx)" | bc -l)
        if (( $(echo "sqrt($sinp * $sinp) >= 1" | bc -l) )); then
            pitch_rad=$(echo "3.14159265359 / 2 * ($sinp / sqrt($sinp * $sinp))" | bc -l)
        else
            pitch_rad=$(echo "a($sinp / sqrt(1 - $sinp * $sinp))" | bc -l)
        fi
        pitch_deg=$(echo "$pitch_rad * 180 / 3.14159265359" | bc -l)
        
        # Yaw (Z-axis rotation)
        siny_cosp=$(echo "2 * ($qw * $qz + $qx * $qy)" | bc -l)
        cosy_cosp=$(echo "1 - 2 * ($qy * $qy + $qz * $qz)" | bc -l)
        yaw_rad=$(echo "a($siny_cosp / $cosy_cosp)" | bc -l)
        yaw_deg=$(echo "$yaw_rad * 180 / 3.14159265359" | bc -l)
        
        # Очищаем экран и выводим данные
        clear
        echo -e "${CYAN}========================================${NC}"
        echo -e "${CYAN}  Отладка осей IMU робота${NC}"
        echo -e "${CYAN}========================================${NC}"
        echo ""
        echo -e "${GREEN}ОРИЕНТАЦИЯ (Quaternion):${NC}"
        echo "  qx: $(format_number $qx)  qy: $(format_number $qy)"
        echo "  qz: $(format_number $qz)  qw: $(format_number $qw)"
        echo ""
        echo -e "${GREEN}УГЛЫ ЭЙЛЕРА (градусы):${NC}"
        echo -e "  ${BLUE}Roll (крен, наклон влево/вправо):${NC}  $(format_number $roll_deg)°"
        echo -e "  ${BLUE}Pitch (тангаж, наклон вперед/назад):${NC}  $(format_number $pitch_deg)°"
        echo -e "  ${BLUE}Yaw (рыскание, поворот влево/вправо):${NC}  $(format_number $yaw_deg)°"
        echo ""
    elif [[ "$line" =~ angular_velocity: ]]; then
        read -r gx_line
        read -r gy_line
        read -r gz_line
        
        gx=$(echo "$gx_line" | grep -oP 'x: \K[+-]?[0-9]+\.?[0-9]*')
        gy=$(echo "$gy_line" | grep -oP 'y: \K[+-]?[0-9]+\.?[0-9]*')
        gz=$(echo "$gz_line" | grep -oP 'z: \K[+-]?[0-9]+\.?[0-9]*')
        
        echo -e "${GREEN}УГЛОВАЯ СКОРОСТЬ (рад/с):${NC}"
        echo -e "  ${BLUE}gx (вращение вокруг X):${NC}  $(format_number $gx)"
        echo -e "  ${BLUE}gy (вращение вокруг Y):${NC}  $(format_number $gy)"
        echo -e "  ${BLUE}gz (вращение вокруг Z):${NC}  $(format_number $gz)"
        echo ""
    elif [[ "$line" =~ linear_acceleration: ]]; then
        read -r ax_line
        read -r ay_line
        read -r az_line
        
        ax=$(echo "$ax_line" | grep -oP 'x: \K[+-]?[0-9]+\.?[0-9]*')
        ay=$(echo "$ay_line" | grep -oP 'y: \K[+-]?[0-9]+\.?[0-9]*')
        az=$(echo "$az_line" | grep -oP 'z: \K[+-]?[0-9]+\.?[0-9]*')
        
        echo -e "${GREEN}ЛИНЕЙНОЕ УСКОРЕНИЕ (м/с²):${NC}"
        echo -e "  ${BLUE}ax (ускорение по X):${NC}  $(format_number $ax)"
        echo -e "  ${BLUE}ay (ускорение по Y):${NC}  $(format_number $ay)"
        echo -e "  ${BLUE}az (ускорение по Z):${NC}  $(format_number $az)"
        echo ""
        echo -e "${CYAN}========================================${NC}"
        echo -e "${YELLOW}Нажмите Ctrl+C для выхода${NC}"
    fi
done

