#!/usr/bin/env python3
# encoding: utf-8
"""
Обработчик Serial порта для связи с Arduino.
Оптимизирован для производительности без locks.
"""

import time
import rospy
import serial
import threading
import json
from std_msgs.msg import String
from config import ConfigLoader


class SerialHandler:
    """
    Класс для обработки Serial порта.
    Оптимизирован для производительности без locks.
    """
    
    def __init__(self, port_name, baudrate, robot_id, hit_detection_pub, config_loader=None):
        """
        Инициализация обработчика Serial порта.
        
        Args:
            port_name: Имя serial порта
            baudrate: Скорость передачи данных
            robot_id: ID робота
            hit_detection_pub: Публикатор для сообщений о попаданиях
            config_loader: Экземпляр ConfigLoader (опционально)
        """
        self.port_name = port_name
        self.baudrate = baudrate
        self.robot_id = robot_id
        self.hit_detection_pub = hit_detection_pub
        
        self.config = config_loader if config_loader else ConfigLoader()
        serial_config = self.config.load('serial')
        serial_params = serial_config.get('serial', {})
        
        self.reconnect_interval = serial_params.get('reconnect_interval', 5.0)
        self.read_sleep = serial_params.get('read_sleep', 0.01)
        self.error_sleep = serial_params.get('error_sleep', 1.0)
        self.max_consecutive_errors = serial_params.get('max_consecutive_errors', 10)
        
        self.serial_port = None
        self.running = True
        self.consecutive_errors = 0
        
        self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reader_thread.start()
        
        self.reconnect()
        
        rospy.loginfo("SerialHandler initialized")
    
    def reconnect(self):
        """Попытка переподключения к Arduino serial порту."""
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                self.serial_port.in_waiting
                return True
            except Exception:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
        
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        
        try:
            import os
            if os.path.exists(self.port_name):
                self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=1)
                rospy.loginfo(f"Successfully connected to Arduino on {self.port_name} at {self.baudrate} baud")
                self.consecutive_errors = 0
                return True
            else:
                rospy.logdebug(f"Serial port {self.port_name} does not exist yet")
                return False
        except serial.SerialException as e:
            rospy.logdebug(f"Failed to connect to serial port: {e}")
            return False
        except Exception as e:
            rospy.logwarn(f"Unexpected error during serial reconnection: {e}")
            return False
    
    def get_port(self):
        """
        Возвращает объект serial порта.
        
        Returns:
            Serial объект или None
        """
        return self.serial_port
    
    def is_connected(self):
        """
        Проверяет подключение к serial порту.
        
        Returns:
            bool: True если подключен
        """
        return self.serial_port is not None and self.serial_port.is_open
    
    def write(self, data):
        """
        Отправляет данные в serial порт.
        
        Args:
            data: Данные для отправки (bytes или str)
        """
        if not self.is_connected():
            rospy.logwarn("Cannot write to serial port: not connected")
            return False
        
        try:
            if isinstance(data, str):
                data = data.encode('utf-8')
            self.serial_port.write(data)
            return True
        except Exception as e:
            rospy.logwarn(f"Failed to write to serial port: {e}")
            return False
    
    def _read_loop(self):
        """Цикл чтения из serial порта."""
        rospy.loginfo("Serial port reading thread started")
        
        while self.running and not rospy.is_shutdown():
            try:
                if not self.is_connected():
                    time.sleep(self.error_sleep)
                    continue
                
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        rospy.loginfo(f"Received from Arduino: '{line}'")
                        if line == "HIT":
                            hit_data = {
                                'robot_id': self.robot_id,
                                'timestamp': rospy.get_time(),
                                'hit_detected': True
                            }
                            hit_msg = String()
                            hit_msg.data = json.dumps(hit_data)
                            self.hit_detection_pub.publish(hit_msg)
                            rospy.loginfo(f"Published hit detection to /game/hit_detection: {hit_msg.data}")
                        else:
                            rospy.logwarn(f"Received unknown response from Arduino: '{line}'")
                    self.consecutive_errors = 0
                else:
                    time.sleep(self.read_sleep)
            except serial.SerialException as e:
                self.consecutive_errors += 1
                if self.consecutive_errors >= self.max_consecutive_errors:
                    rospy.logwarn(f"Serial port error in reading thread (attempt {self.consecutive_errors}): {e}")
                    rospy.logwarn("Serial port appears disconnected. Will retry when connection is restored")
                    self.consecutive_errors = 0
                time.sleep(self.error_sleep)
            except Exception as e:
                rospy.logwarn(f"Unexpected error in serial reading thread: {e}")
                time.sleep(self.error_sleep / 2.0)
        
        rospy.loginfo("Serial port reading thread stopped")
    
    def stop(self):
        """Останавливает обработчик."""
        self.running = False
        if self.serial_port is not None and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

