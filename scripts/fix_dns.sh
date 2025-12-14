#!/bin/bash
#
# Скрипт для автоматического восстановления DNS настроек
# Запускается при загрузке системы и проверяет наличие DNS серверов

DNS_FILE="/etc/resolv.conf"
DNS_SERVERS=(
    "8.8.8.8"
    "8.8.4.4"
    "1.1.1.1"
    "1.0.0.1"
)

# Проверяем, есть ли хотя бы один nameserver в файле
if ! grep -q "^nameserver" "$DNS_FILE" 2>/dev/null; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] DNS servers not found in $DNS_FILE. Restoring..."
    
    # Создаем новый resolv.conf с DNS серверами
    cat > "$DNS_FILE" << EOF
# DNS servers configured automatically by fix_dns.sh
# Restored at $(date '+%Y-%m-%d %H:%M:%S')

# Google DNS
nameserver 8.8.8.8
nameserver 8.8.4.4
# Cloudflare DNS (backup)
nameserver 1.1.1.1
nameserver 1.0.0.1
EOF
    
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] DNS servers restored successfully"
    exit 0
fi

# Проверяем, что все нужные DNS серверы присутствуют
MISSING_SERVERS=()
for server in "${DNS_SERVERS[@]}"; do
    if ! grep -q "nameserver $server" "$DNS_FILE" 2>/dev/null; then
        MISSING_SERVERS+=("$server")
    fi
done

# Если какие-то серверы отсутствуют, восстанавливаем файл
if [ ${#MISSING_SERVERS[@]} -gt 0 ]; then
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Some DNS servers are missing. Restoring..."
    
    cat > "$DNS_FILE" << EOF
# DNS servers configured automatically by fix_dns.sh
# Restored at $(date '+%Y-%m-%d %H:%M:%S')

# Google DNS
nameserver 8.8.8.8
nameserver 8.8.4.4
# Cloudflare DNS (backup)
nameserver 1.1.1.1
nameserver 1.0.0.1
EOF
    
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] DNS servers restored successfully"
fi

exit 0
