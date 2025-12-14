#!/bin/bash
#
# Мониторинг DNS - периодически проверяет и восстанавливает DNS настройки
# Можно запускать в cron или как отдельный демон

DNS_FILE="/etc/resolv.conf"
FIX_SCRIPT="/home/ubuntu/ros_ws/src/ainex_peripherals/scripts/fix_dns.sh"
LOG_FILE="/var/log/fix_dns.log"

# Создаем лог файл если его нет
touch "$LOG_FILE" 2>/dev/null || LOG_FILE="$HOME/fix_dns.log"

# Проверяем наличие fix_dns.sh
if [ ! -f "$FIX_SCRIPT" ]; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: $FIX_SCRIPT not found!" >> "$LOG_FILE"
    exit 1
fi

# Проверяем DNS
if ! grep -q "^nameserver" "$DNS_FILE" 2>/dev/null; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: No DNS servers found, running fix script..." >> "$LOG_FILE"
    sudo "$FIX_SCRIPT" >> "$LOG_FILE" 2>&1
    exit $?
fi

# Проверяем что DNS работает
if ! host google.com > /dev/null 2>&1; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: DNS resolution failed, running fix script..." >> "$LOG_FILE"
    sudo "$FIX_SCRIPT" >> "$LOG_FILE" 2>&1
    exit $?
fi

exit 0
