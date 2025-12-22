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

