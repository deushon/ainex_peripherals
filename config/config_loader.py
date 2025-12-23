#!/usr/bin/env python3
# encoding: utf-8
"""
Загрузчик конфигурационных файлов.
"""

import os
import yaml
import rospkg
import rospy


class ConfigLoader:
    """Загрузчик конфигурации из YAML файлов."""
    
    def __init__(self, package_name='ainex_peripherals'):
        """
        Инициализация загрузчика конфигурации.
        
        Args:
            package_name: Имя ROS пакета
        """
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path(package_name)
        self.config_dir = os.path.join(self.package_path, 'config')
        self._cache = {}
    
    def load(self, config_name):
        """
        Загружает конфигурационный файл.
        
        Args:
            config_name: Имя конфига без расширения (например, 'joystick')
        
        Returns:
            dict: Содержимое конфига
        """
        if config_name in self._cache:
            return self._cache[config_name]
        
        config_path = os.path.join(self.config_dir, f'{config_name}.yaml')
        
        if not os.path.exists(config_path):
            try:
                rospy.logwarn(f"Config file not found: {config_path}, using empty config")
            except Exception:
                print(f"Config file not found: {config_path}, using empty config")
            return {}
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                if config is None:
                    config = {}
                self._cache[config_name] = config
                return config
        except Exception as e:
            try:
                rospy.logerr(f"Error loading config {config_name}: {e}")
            except Exception:
                print(f"Error loading config {config_name}: {e}")
            return {}
    
    def get(self, config_name, key_path, default=None):
        """
        Получает значение по пути ключей.
        
        Args:
            config_name: Имя конфига
            key_path: Путь через точку (например, 'joystick.axes_map')
            default: Значение по умолчанию
        
        Returns:
            Значение конфига или default
        """
        config = self.load(config_name)
        keys = key_path.split('.')
        value = config
        
        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
    def save(self, config_name, config_data):
        """
        Сохраняет конфигурацию в YAML файл.
        
        Args:
            config_name: Имя конфига без расширения (например, 'walking')
            config_data: Словарь с данными для сохранения
        
        Returns:
            bool: True если успешно, False при ошибке
        """
        config_path = os.path.join(self.config_dir, f'{config_name}.yaml')
        
        try:
            # Загружаем существующий конфиг, если есть
            existing_config = self.load(config_name)
            
            # Объединяем с новыми данными (новые данные имеют приоритет)
            if existing_config:
                # Глубокое слияние словарей
                merged_config = self._deep_merge(existing_config.copy(), config_data)
            else:
                merged_config = config_data
            
            # Сохраняем в файл
            with open(config_path, 'w') as f:
                yaml.safe_dump(merged_config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            
            # Обновляем кэш
            self._cache[config_name] = merged_config
            
            try:
                rospy.loginfo(f"Config saved: {config_path}")
            except Exception:
                print(f"Config saved: {config_path}")
            
            return True
        except Exception as e:
            try:
                rospy.logerr(f"Error saving config {config_name}: {e}")
            except Exception:
                print(f"Error saving config {config_name}: {e}")
            return False
    
    def _deep_merge(self, base, update):
        """
        Рекурсивно объединяет два словаря.
        
        Args:
            base: Базовый словарь
            update: Словарь с обновлениями
        
        Returns:
            dict: Объединенный словарь
        """
        result = base.copy()
        
        for key, value in update.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._deep_merge(result[key], value)
            else:
                result[key] = value
        
        return result
    
    def set(self, config_name, key_path, value):
        """
        Устанавливает значение по пути ключей и сохраняет конфиг.
        
        Args:
            config_name: Имя конфига
            key_path: Путь через точку (например, 'walking.stabilization_pid.kp')
            value: Значение для установки
        
        Returns:
            bool: True если успешно, False при ошибке
        """
        config = self.load(config_name)
        keys = key_path.split('.')
        
        # Создаем вложенную структуру
        current = config
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            elif not isinstance(current[key], dict):
                current[key] = {}
            current = current[key]
        
        # Устанавливаем значение
        current[keys[-1]] = value
        
        # Сохраняем
        return self.save(config_name, config)

