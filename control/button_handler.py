#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль обработки действий на кнопки джойстика.
"""

import rospy
import serial
import os
import subprocess
import math
import time
from ainex_sdk import Board
from std_msgs.msg import Bool
from config import ConfigLoader


BUTTON_PRESSED = 1
BUTTON_RELEASED = 3


class ButtonHandler:
    """
    Класс для обработки действий на кнопки джойстика.
    """
    
    def __init__(self, board, gait_manager, speed_control, motion_manager, 
                 game_services, serial_getter, robot_state_getter, 
                 firing_state_pub=None, config_loader=None):
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
            config_loader: Экземпляр ConfigLoader (опционально)
        """
        self.board = board
        self.gait_manager = gait_manager
        self.speed_control = speed_control
        self.motion_manager = motion_manager
        self.game_services = game_services
        self.get_serial_port = serial_getter
        self.get_robot_state = robot_state_getter
        self.firing_state_pub = firing_state_pub
        
        self.config = config_loader if config_loader else ConfigLoader()
        buttons_config = self.config.load('buttons')
        buttons_params = buttons_config.get('buttons', {})
        walking_config = self.config.load('walking')
        walking_params = walking_config.get('walking', {})
        
        actions_config = buttons_params.get('actions', {})
        self.lie_to_stand_action = actions_config.get('lie_to_stand', 'lie_to_stand')
        self.recline_to_stand_action = actions_config.get('recline_to_stand', 'recline_to_stand')
        
        sound_config = buttons_params.get('sound', {})
        self.sound_file_path = sound_config.get('file_path', '/usr/share/sounds/alsa/Front_Left.wav')
        self.sound_play_timeout = sound_config.get('play_timeout', 5)
        
        arm_config = buttons_params.get('arm', {})
        self.arm_duration = arm_config.get('duration', 0.5)
        self.arm_positions_pressed = arm_config.get('positions_pressed', [[13, 253], [15, 880], [17, 480], [19, 520]])
        self.arm_positions_released = arm_config.get('positions_released', [[13, 707], [15, 840], [17, 480], [19, 133]])
        
        buzzer_config = buttons_params.get('buzzer', {})
        self.buzzer_r1_freq = buzzer_config.get('r1_freq', 2000)
        self.buzzer_l1_freq = buzzer_config.get('l1_freq', 1500)
        self.buzzer_circle_freq = buzzer_config.get('circle_freq', 1500)
        self.buzzer_start_freq = buzzer_config.get('start_freq', 1900)
        self.buzzer_duration = buzzer_config.get('duration', 0.1)
        self.buzzer_pause = buzzer_config.get('pause', 0.05)
        self.buzzer_repeats = buzzer_config.get('repeats', 1)
        
        height_reset_config = walking_params.get('height', {}).get('reset', {})
        self.height_reset_target = height_reset_config.get('target', 0.025)
        self.height_reset_step = height_reset_config.get('step', 0.005)
        self.height_reset_delay = height_reset_config.get('delay', 0.05)
        
        self.sound = None
        self._setup_sound()
        
        rospy.loginfo("ButtonHandler module initialized")
    
    def _setup_sound(self):
        """Настройка звука для кнопки X."""
        try:
            import pygame
            if os.path.exists(self.sound_file_path):
                pygame.mixer.init()
                self.sound = pygame.mixer.Sound(self.sound_file_path)
        except Exception:
            pass
    
    def r1_callback(self, new_state):
        """Обработчик кнопки R1 - увеличение скорости."""
        if new_state == BUTTON_PRESSED and self.speed_control.get_speed_mode() < 4:
            new_mode = self.speed_control.get_speed_mode() + 1
            self.speed_control.set_speed_mode(new_mode)
            self.board.set_buzzer(self.buzzer_r1_freq, self.buzzer_duration, self.buzzer_pause, self.buzzer_repeats)
    
    def l1_callback(self, new_state):
        """Обработчик кнопки L1 - уменьшение скорости."""
        if new_state == BUTTON_PRESSED and self.speed_control.get_speed_mode() > 1:
            new_mode = self.speed_control.get_speed_mode() - 1
            self.speed_control.set_speed_mode(new_mode)
            self.board.set_buzzer(self.buzzer_l1_freq, self.buzzer_duration, self.buzzer_pause, self.buzzer_repeats)
    
    def cross_callback(self, new_state):
        """Обработчик кнопки Cross - стрельба."""
        if not self.game_services.can_fire():
            if new_state == BUTTON_PRESSED:
                rospy.logwarn("Fire command blocked - no permission")
            return
        
        ser = self.get_serial_port()
        if ser is None or not ser.is_open:
            rospy.logwarn("Cannot send firing command: Arduino not connected")
            return
        
        try:
            if new_state == BUTTON_PRESSED:
                if self.firing_state_pub is not None:
                    firing_msg = Bool()
                    firing_msg.data = True
                    self.firing_state_pub.publish(firing_msg)
                ser.write(b"FIRE\n")
                rospy.loginfo("Firing started")
            elif new_state == BUTTON_RELEASED:
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
                                timeout=self.sound_play_timeout
                            )
                            break
                        except (subprocess.TimeoutExpired, FileNotFoundError):
                            continue
                except Exception as e:
                    rospy.logwarn(f"Failed to play sound via system command: {e}")
    
    def square_callback(self, new_state):
        """Обработчик кнопки Square - управление руками."""
        if new_state == BUTTON_PRESSED:
            self.board.bus_servo_set_position(self.arm_duration, self.arm_positions_pressed)
        elif new_state == BUTTON_RELEASED:
            self.board.bus_servo_set_position(self.arm_duration, self.arm_positions_released)
    
    def circle_callback(self, new_state):
        """Обработчик кнопки Circle - подъем робота."""
        if new_state == BUTTON_PRESSED:
            robot_state = self.get_robot_state()
            rospy.loginfo(f"Circle button pressed. Current robot state: '{robot_state}'")
            
            if robot_state == 'stand':
                rospy.loginfo("Robot is already in a 'stand' state. No get-up action will be performed")
                self.board.set_buzzer(self.buzzer_circle_freq, self.buzzer_duration, self.buzzer_pause, self.buzzer_repeats)
                return
            
            action_to_run = None
            if robot_state == 'lie_to_stand':
                action_to_run = self.lie_to_stand_action
            elif robot_state == 'recline_to_stand':
                action_to_run = self.recline_to_stand_action
            
            rospy.loginfo(f"Initiating {action_to_run}")
            self.board.set_buzzer(self.buzzer_circle_freq, self.buzzer_duration, self.buzzer_pause, self.buzzer_repeats)
            
            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated successfully")
                else:
                    rospy.logwarn("MotionManager not initialized or no action specified")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action('{action_to_run}'): {e}")
    
    def start_callback(self, new_state, height_getter, height_setter):
        """Обработчик кнопки Start - сброс высоты робота."""
        if new_state == BUTTON_PRESSED:
            rospy.loginfo("Start button pressed. Resetting body height")
            self.board.set_buzzer(self.buzzer_start_freq, self.buzzer_duration, self.buzzer_pause, self.buzzer_repeats)
            new_height = self._reset_height(self.height_reset_target)
            height_setter(new_height)
    
    def _reset_height(self, target_height):
        """
        Сбрасывает высоту робота до целевого значения.
        
        Args:
            target_height: Целевая высота
        
        Returns:
            float: Новое значение высоты
        """
        current_height = self.speed_control.get_body_height()
        steps = int(abs(target_height - current_height) / self.height_reset_step)
        
        if steps != 0:
            direction = math.copysign(1.0, target_height - current_height)
            for _ in range(steps):
                current_height += self.height_reset_step * direction
                self.speed_control.set_body_height(current_height)
                time.sleep(self.height_reset_delay)
        
        return self.speed_control.get_body_height()

