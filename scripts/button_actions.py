#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль обработки действий на кнопки джойстика.
"""

# ========== КОНФИГУРАЦИЯ ==========
# Звук
SOUND_FILE_PATH = '/usr/share/sounds/alsa/Front_Left.wav'
SOUND_PLAY_TIMEOUT = 5  # Таймаут воспроизведения звука (сек)

# Управление руками (Square button)
ARM_DURATION = 0.5  # Длительность движения рук (сек)
ARM_POSITIONS_PRESSED = [[13, 253], [15, 880], [17, 480], [19, 520]]
ARM_POSITIONS_RELEASED = [[13, 707], [15, 840], [17, 480], [19, 133]]

# Звуковые сигналы (частота, длительность, пауза, повторения)
BUZZER_R1_FREQ = 2000  # Частота для R1 (увеличение скорости)
BUZZER_L1_FREQ = 1500  # Частота для L1 (уменьшение скорости)
BUZZER_CIRCLE_FREQ = 1500  # Частота для Circle (подъем)
BUZZER_START_FREQ = 1900  # Частота для Start (сброс высоты)
BUZZER_DURATION = 0.1  # Длительность сигнала
BUZZER_PAUSE = 0.05  # Пауза между сигналами
BUZZER_REPEATS = 1  # Количество повторений

# Высота робота
HEIGHT_RESET_TARGET = 0.025  # Целевая высота при сбросе
HEIGHT_RESET_STEP = 0.005  # Шаг изменения высоты при сбросе
HEIGHT_RESET_DELAY = 0.05  # Задержка между шагами (сек)
# ===================================

import rospy
import serial
import os
import subprocess
import math
import time
from ainex_sdk import Board
from std_msgs.msg import Bool

# Константы состояний кнопок (совместимость с ButtonState из joystick_control.py)
BUTTON_PRESSED = 1
BUTTON_RELEASED = 3


class ButtonActions:
    """
    Класс для обработки действий на кнопки джойстика.
    """
    
    def __init__(self, board, gait_manager, speed_control, motion_manager, 
                 game_services, serial_getter, robot_state_getter, firing_state_pub=None):
        """
        Инициализация модуля действий кнопок.
        
        Args:
            board: Экземпляр Board
            gait_manager: Экземпляр GaitManager
            speed_control: Экземпляр SpeedControl
            motion_manager: Экземпляр MotionManager
            game_services: Экземпляр GameServices
            serial_getter: Функция для получения serial порта
            robot_state_getter: Функция для получения состояния робота
            firing_state_pub: Публикатор состояния стрельбы (опционально)
        """
        self.board = board
        self.gait_manager = gait_manager
        self.speed_control = speed_control
        self.motion_manager = motion_manager
        self.game_services = game_services
        self.get_serial_port = serial_getter
        self.get_robot_state = robot_state_getter
        self.firing_state_pub = firing_state_pub
        
        # Звук для кнопки X
        self.sound = None
        self.sound_file_path = None
        self._setup_sound()
        
        rospy.loginfo("ButtonActions module initialized")
    
    def _setup_sound(self):
        """Настройка звука для кнопки X."""
        try:
            import pygame
            if os.path.exists(SOUND_FILE_PATH):
                pygame.mixer.init()
                self.sound = pygame.mixer.Sound(SOUND_FILE_PATH)
            else:
                self.sound_file_path = SOUND_FILE_PATH
        except Exception as e:
            rospy.logdebug(f"Could not initialize pygame sound: {e}")
            self.sound_file_path = SOUND_FILE_PATH
    
    def r1_callback(self, new_state):
        """Обработчик кнопки R1 - увеличение скорости."""
        if new_state == BUTTON_PRESSED and self.speed_control.get_speed_mode() < 4:
            new_mode = self.speed_control.get_speed_mode() + 1
            self.speed_control.set_speed_mode(new_mode)
            self.board.set_buzzer(BUZZER_R1_FREQ, BUZZER_DURATION, BUZZER_PAUSE, BUZZER_REPEATS)
    
    def l1_callback(self, new_state):
        """Обработчик кнопки L1 - уменьшение скорости."""
        if new_state == BUTTON_PRESSED and self.speed_control.get_speed_mode() > 1:
            new_mode = self.speed_control.get_speed_mode() - 1
            self.speed_control.set_speed_mode(new_mode)
            self.board.set_buzzer(BUZZER_L1_FREQ, BUZZER_DURATION, BUZZER_PAUSE, BUZZER_REPEATS)
    
    def cross_callback(self, new_state):
        """Обработчик кнопки Cross - стрельба."""
        if not self.game_services.can_fire():
            if new_state == BUTTON_PRESSED:
                rospy.logwarn("Fire command blocked - no permission")
            return
        
        ser = self.get_serial_port()
        if ser is None or not ser.is_open:
            rospy.logwarn("Cannot send firing command: Arduino not connected.")
            return
        
        try:
            if new_state == BUTTON_PRESSED:
                # Публикуем состояние стрельбы СРАЗУ (до отправки команды в Arduino)
                # Это важно для backend, который валидирует попадания
                if self.firing_state_pub is not None:
                    firing_msg = Bool()
                    firing_msg.data = True
                    self.firing_state_pub.publish(firing_msg)
                ser.write(b"FIRE\n")
                rospy.loginfo("Firing started")
            elif new_state == BUTTON_RELEASED:
                # Публикуем состояние стрельбы СРАЗУ (до отправки команды в Arduino)
                if self.firing_state_pub is not None:
                    firing_msg = Bool()
                    firing_msg.data = False
                    self.firing_state_pub.publish(firing_msg)
                ser.write(b"STOP\n")
                rospy.loginfo("Firing stopped")
        except serial.SerialException as e:
            rospy.logwarn(f"Failed to send firing command to Arduino: {e}")
    
    def x_button_callback(self, new_state):
        """Обработчик кнопки X - воспроизведение звука."""
        if new_state == BUTTON_PRESSED:
            if self.sound:
                try:
                    self.sound.play()
                except Exception as e:
                    rospy.logwarn(f"Failed to play sound via pygame: {e}")
            elif self.sound_file_path and os.path.exists(self.sound_file_path):
                try:
                    for cmd in ['aplay', 'paplay']:
                        try:
                            subprocess.run(
                                [cmd, self.sound_file_path],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                                timeout=SOUND_PLAY_TIMEOUT
                            )
                            break
                        except (subprocess.TimeoutExpired, FileNotFoundError):
                            continue
                except Exception as e:
                    rospy.logwarn(f"Failed to play sound via system command: {e}")
    
    def square_callback(self, new_state):
        """Обработчик кнопки Square - управление руками."""
        if new_state == BUTTON_PRESSED:
            self.board.bus_servo_set_position(ARM_DURATION, ARM_POSITIONS_PRESSED)
        elif new_state == BUTTON_RELEASED:
            self.board.bus_servo_set_position(ARM_DURATION, ARM_POSITIONS_RELEASED)
    
    def circle_callback(self, new_state):
        """Обработчик кнопки Circle - подъем робота."""
        if new_state == BUTTON_PRESSED:
            robot_state = self.get_robot_state()
            rospy.loginfo(f"Circle (B) button pressed. Current robot state: '{robot_state}'.")
            
            if robot_state == 'stand':
                rospy.loginfo("Robot is already in a 'stand' state. No get-up action will be performed.")
                self.board.set_buzzer(BUZZER_CIRCLE_FREQ, BUZZER_DURATION, BUZZER_PAUSE, BUZZER_REPEATS)
                return
            
            action_to_run = None
            if robot_state == 'lie_to_stand':
                action_to_run = 'lie_to_stand'
            elif robot_state == 'recline_to_stand':
                action_to_run = 'recline_to_stand'
            
            rospy.loginfo(f"Initiating {action_to_run}.")
            self.board.set_buzzer(BUZZER_CIRCLE_FREQ, BUZZER_DURATION, BUZZER_PAUSE, BUZZER_REPEATS)
            
            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated successfully.")
                else:
                    rospy.logwarn("MotionManager not initialized or no action specified.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action('{action_to_run}'): {e}")
    
    def start_callback(self, new_state, height_getter, height_setter):
        """Обработчик кнопки Start - сброс высоты робота."""
        if new_state == BUTTON_PRESSED:
            rospy.loginfo("Start button pressed. Resetting body height.")
            self.board.set_buzzer(BUZZER_START_FREQ, BUZZER_DURATION, BUZZER_PAUSE, BUZZER_REPEATS)
            # reset_height теперь использует единый источник истины через speed_control
            new_height = self.reset_height(HEIGHT_RESET_TARGET)
            height_setter(new_height)
    
    def reset_height(self, target_height=HEIGHT_RESET_TARGET):
        """
        Сбрасывает высоту робота до целевого значения.
        Использует единый метод set_body_height из speed_control.
        
        Args:
            target_height: Целевая высота
        
        Returns:
            float: Новое значение высоты
        """
        # Получаем текущую высоту из gait_manager через speed_control
        current_height = self.speed_control.get_body_height()
        t = int(abs(target_height - current_height) / HEIGHT_RESET_STEP)
        if t != 0:
            direction = math.copysign(1, target_height - current_height)
            for _ in range(t):
                current_height += HEIGHT_RESET_STEP * direction
                # Используем единый метод для установки высоты
                self.speed_control.set_body_height(current_height)
                time.sleep(HEIGHT_RESET_DELAY)
        return self.speed_control.get_body_height()

