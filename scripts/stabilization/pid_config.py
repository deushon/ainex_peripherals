#!/usr/bin/env python3
# encoding: utf-8
"""
Конфигурация PID регулятора стабилизации корпуса.
Единая конфигурация используется для покоя и ходьбы.
"""

# ========== ЕДИНАЯ КОНФИГУРАЦИЯ PID РЕГУЛЯТОРА ==========
# Оставляем только pitch -> x_offset через библиотеку simple_pid
UNIFIED_PID_CONFIG = {
    # Включение/выключение PID стабилизации
    'pid_enabled': True,
    'pitch_enabled': True,

    # PID коэффициенты для pitch (метры на градус)
    'pitch_kp': 0.0005,
    'pitch_ki': 0.00005,
    'pitch_kd': 0.0002,
    'pitch_max_output': 0.02,  # Максимальное смещение по X (м)
    'pitch_target': 90.0,      # Целевой угол pitch (градусы)

    # Пороги активации
    'pitch_activation_threshold': 0.0,

    # Базовые смещения позы (начальные значения)
    'base_x_offset': 0.0,
'base_y_offset': 0.0,     # Для ручной настройки без PID (через топик)
'base_roll_offset': 0.0,  # Для ручной настройки без PID (через топик)
    'base_pitch_offset': 0.0,

    # Пределы смещений от регулятора (безопасные пределы)
    'limit_x_offset_min': -0.03,
    'limit_x_offset_max': 0.03,
    'limit_pitch_offset_min': -5.0,
    'limit_pitch_offset_max': 5.0,

    # Коэффициент для преобразования PID выхода в смещение туловища
    'pitch_to_x_offset_ratio': 1.0,
    'pitch_to_pitch_offset_ratio': 0.0,

    # Настройки фильтра IMU
    'filter_enabled': True,
    'filter_alpha': 0.9,

    # Интервал обновления PID
    'update_interval': 0.03,
}

