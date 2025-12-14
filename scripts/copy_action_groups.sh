#!/bin/bash

# Скрипт для копирования всех файлов из action_groups в целевую директорию
# Автоматически заменяет существующие файлы с тем же именем

SOURCE_DIR="$(dirname "$(dirname "$(readlink -f "$0")")")/action_groups"
TARGET_DIR="/home/ubuntu/software/ainex_controller/ActionGroups"

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

# Копируем все файлы из action_groups в целевую директорию
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


