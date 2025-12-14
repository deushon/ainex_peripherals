#!/bin/bash

# Скрипт для копирования всех файлов из внешней ActionGroups в локальную action_groups
# Автоматически заменяет существующие файлы с тем же именем

SOURCE_DIR="/home/ubuntu/software/ainex_controller/ActionGroups"
TARGET_DIR="$(dirname "$(dirname "$(readlink -f "$0")")")/action_groups"

# Проверяем существование исходной директории
if [ ! -d "$SOURCE_DIR" ]; then
    echo "Ошибка: Исходная директория не найдена: $SOURCE_DIR"
    exit 1
fi

# Создаем целевую директорию, если она не существует
mkdir -p "$TARGET_DIR"

if [ $? -ne 0 ]; then
    echo "Ошибка: Не удалось создать целевую директорию: $TARGET_DIR"
    exit 1
fi

# Проверяем, есть ли файлы для копирования
if [ -z "$(ls -A "$SOURCE_DIR" 2>/dev/null)" ]; then
    echo "Предупреждение: Исходная директория пуста: $SOURCE_DIR"
    exit 0
fi

# Копируем все файлы из внешней ActionGroups в локальную action_groups
# -f: принудительно заменяет существующие файлы
# -v: выводит информацию о копируемых файлах
echo "Копирование файлов из $SOURCE_DIR в $TARGET_DIR..."
cp -fv "$SOURCE_DIR"/* "$TARGET_DIR"/

if [ $? -eq 0 ]; then
    echo "Успешно скопированы все файлы!"
    echo "Скопированные файлы:"
    ls -lh "$TARGET_DIR"
else
    echo "Ошибка при копировании файлов"
    exit 1
fi


