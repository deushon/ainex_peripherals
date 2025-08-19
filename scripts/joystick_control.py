#!/usr/bin/env python3
# encoding: utf-8
import time
import rospy
import serial
import threading
import pygame.mixer
import os
import math
from ainex_sdk import Board
from sensor_msgs.msg import Joy, Imu
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager
from std_msgs.msg import String

# --- Constants and Mappings ---
AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', 'x_button', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState:
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

# --- Serial Reader Thread ---
class SerialReader(threading.Thread):
    def __init__(self, ser_instance, hit_info_pub_instance):
        super().__init__()
        self.ser = ser_instance
        self.hit_info_pub = hit_info_pub_instance
        self.running = True

    def run(self):
        rospy.loginfo("Serial port reading thread started.")
        while self.running and not rospy.is_shutdown():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        rospy.loginfo(f"Received from Arduino: '{line}'")
                        if line == "HIT":
                            hit_msg = String()
                            hit_msg.data = "1"
                            self.hit_info_pub.publish(hit_msg)
                            rospy.loginfo("Published to /hit_info: 1")
                        else:
                            rospy.logwarn(f"Received unknown response from Arduino: '{line}'")
            except serial.SerialException as e:
                rospy.logerr(f"Serial port error in reading thread: {e}")
                self.running = False
            except Exception as e:
                rospy.logerr(f"Error in serial reading thread: {e}")
            time.sleep(0.01)
        rospy.loginfo("Serial port reading thread stopped.")

    def stop(self):
        self.running = False

# --- Main Controller Class ---
class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)

        # --- Hardware and ROS Initialization ---
        self.board = Board()
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager()
        
        # --- State Variables ---
        self.speed_mode = 1
        self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude = 0, 0, 0
        self.init_z_offset = 0.025
        self.time_stamp_ry = 0
        self.status = 'stop'
        self.update_param, self.update_height = False, False
        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP)))

        # --- NEW: State variables for Get-Up functionality (from joystick_control 1.py) ---
        self.robot_state = 'stand'
        self.lie_to_stand_action_name = 'lie_to_stand'
        self.recline_to_stand_action_name = 'recline_to_stand'
        self.count_lie = 0
        self.count_recline = 0
        self.FALL_COUNT_THRESHOLD = 50

        # --- NEW: Refactored speed parameters into a dictionary ---
        self.setup_speed_parameters()

        # --- Initialization from primer.py ---
        self.ser = None
        self.sound = None
        self.setup_primer_features()

        # --- Start ROS Subscribers ---
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        # NEW: Subscriber for robot state from IMU
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        rospy.loginfo("Joystick Controller Initialized. Speed Mode: 1")
        time.sleep(0.2)

    def setup_speed_parameters(self):
        """Initializes a dictionary with all walking parameters for each speed mode."""
        self.speed_params = {
            1: {'period_time': [400, 0.2, 0.022], 'x_amp': 0.01, 'y_amp': 0.015, 'angle_amp': 8, 'z_move_amplitude': 0.025},
            2: {'period_time': [500, 0.2, 0.028], 'x_amp': 0.015, 'y_amp': 0.015, 'angle_amp': 10, 'z_move_amplitude': 0.02, 'gait_base': {'dsp_ratio': 0.2, 'step_fb_ratio': 0.028, 'y_swap_amplitude': 0.02, 'z_swap_amplitude': 0.006, 'init_y_offset': -0.008}},
            3: {'period_time': [400, 0.2, 0.028], 'x_amp': 0.01, 'y_amp': 0.015, 'angle_amp': 10, 'z_move_amplitude': 0.02, 'gait_base': {'dsp_ratio': 0.2, 'init_y_offset': -0.005, 'step_fb_ratio': 0.028, 'y_swap_amplitude': 0.02, 'z_swap_amplitude': 0.006}},
            4: {'period_time': [300, 0.2, 0.028], 'x_amp': 0.01, 'y_amp': 0.015, 'angle_amp': 8, 'z_move_amplitude': 0.015, 'gait_base': {'dsp_ratio': 0.2, 'init_y_offset': -0.008, 'step_fb_ratio': 0.028, 'y_swap_amplitude': 0.021, 'z_swap_amplitude': 0.006, 'pelvis_offset': 5, 'arm_swing_gain': 0.5}}
        }

    def setup_primer_features(self):
        """Initializes Serial, Sound, Publisher, and Thread."""
        try:
            port = rospy.get_param('~port', '/dev/ttyUSB0')
            baud = rospy.get_param('~baudrate', 9600)
            self.ser = serial.Serial(port, baud, timeout=1)
            rospy.loginfo(f"Opened port {port} at {baud} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            return
        try:
            pygame.mixer.init()
            sound_file_path = os.path.abspath("/home/ubuntu/ros_ws/src/proverka_nod/scripts/FIRED.wav")
            if os.path.exists(sound_file_path):
                self.sound = pygame.mixer.Sound(sound_file_path)
                rospy.loginfo(f"Sound file loaded: {sound_file_path}")
            else:
                rospy.logerr(f"Sound file not found: {sound_file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize pygame mixer or load sound: {e}")
        self.hit_info_pub = rospy.Publisher('/hit_info', String, queue_size=10)
        self.serial_reader_thread = SerialReader(self.ser, self.hit_info_pub)
        self.serial_reader_thread.daemon = True
        self.serial_reader_thread.start()
        rospy.on_shutdown(self.serial_reader_thread.stop)

    # NEW: IMU Callback from joystick_control 1.py
    def imu_callback(self, msg: Imu):
        """
        Обрабатывает данные с IMU для определения состояния падения.
        Логика адаптирована из примера мобильного приложения, используя ay и az.
        """
        try:
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            ACCEL_THRESH = 7.0
            ANGLE_THRESH = 30.0
            COUNT_INCREMENT = 1
            COUNT_DECREMENT = 1

            if abs(az) > 1e-6:
                angle_rad = math.atan2(abs(ay), abs(az))
                angle_deg = math.degrees(angle_rad)
            else:
                angle_deg = 90.0

            if angle_deg < ANGLE_THRESH:
                if az > ACCEL_THRESH:
                    self.count_lie += COUNT_INCREMENT
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
                elif az < -ACCEL_THRESH:
                    self.count_recline += COUNT_INCREMENT
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                else:
                    self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                    self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)
            else:
                self.count_lie = max(0, self.count_lie - COUNT_DECREMENT)
                self.count_recline = max(0, self.count_recline - COUNT_DECREMENT)

            old_state = self.robot_state

            if self.count_lie > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'lie_to_stand'
            elif self.count_recline > self.FALL_COUNT_THRESHOLD:
                self.robot_state = 'recline_to_stand'
            else:
                self.robot_state = 'stand'

            if old_state != self.robot_state:
                rospy.loginfo(f"IMU detected robot state change: '{old_state}' -> '{self.robot_state}' (lie_count:{self.count_lie}, recline_count:{self.count_recline})")

        except Exception as e:
            rospy.logwarn(f"Error processing IMU data in imu_callback: {e}")

    def axes_callback(self, axes):
        """Calculates and sets walking parameters based on speed mode and joystick input."""
        self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude = 0, 0, 0
        self.update_param = False
        
        gait_param = self.gait_manager.get_gait_param()
        params = self.speed_params[self.speed_mode]
        period_time = list(params['period_time'])
        
        if self.speed_mode > 1:
            gait_param.update(params['gait_base'])

        # Conditional logic from kinematics, now simplified
        if self.speed_mode == 1 and abs(axes['lx']) > 0.3: period_time[2] = 0.025
        elif self.speed_mode == 2:
            if abs(axes['rx']) > 0.3 and abs(axes['lx']) < 0.3 and abs(axes['ly']) < 0.3: gait_param.update({'init_roll_offset': 0, 'init_y_offset': -0.005, 'y_swap_amplitude': 0.022})
            if abs(axes['lx']) > 0.3 and abs(axes['ly']) < 0.3 and abs(axes['rx']) < 0.3: gait_param.update({'init_y_offset': -0.005, 'y_swap_amplitude': 0.028, 'init_roll_offset': 3})
        elif self.speed_mode == 3:
            if abs(axes['rx']) > 0.3 and abs(axes['lx']) < 0.3 and abs(axes['ly']) < 0.3: gait_param['y_swap_amplitude'] = 0.022
            if abs(axes['lx']) > 0.3 and abs(axes['ly']) < 0.3 and abs(axes['rx']) < 0.3: gait_param.update({'init_y_offset': 0, 'init_roll_offset': 3, 'y_swap_amplitude': 0.025})
        elif self.speed_mode == 4:
            if abs(axes['ly']) > 0.3: gait_param['init_roll_offset'] = -3
            if abs(axes['lx']) > 0.3 and abs(axes['ly']) < 0.3 and abs(axes['rx']) < 0.3: gait_param['init_roll_offset'] = -1.0
            if abs(axes['rx']) > 0.3 and abs(axes['lx']) < 0.3 and abs(axes['ly']) < 0.3: gait_param['init_roll_offset'] = -3

        if abs(axes['ly']) > 0.3: self.x_move_amplitude = math.copysign(params['x_amp'], axes['ly'])
        if abs(axes['lx']) > 0.3: self.y_move_amplitude = math.copysign(params['y_amp'], axes['lx'])
        if abs(axes['rx']) > 0.3: self.angle_move_amplitude = math.copysign(params['angle_amp'], axes['rx'])
        
        self.update_param = any(abs(amp) > 0 for amp in [self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude])

        if self.update_param:
            gait_param['init_z_offset'] = self.init_z_offset
            self.gait_manager.set_step(period_time, self.x_move_amplitude, self.y_move_amplitude, self.angle_move_amplitude, gait_param, step_num=0)
        
        if self.status == 'stop' and self.update_param: self.status = 'move'
        elif self.status == 'move' and not self.update_param:
            self.status = 'stop'
            self.gait_manager.stop()

    def height_callback(self, axes):
        """Handles height adjustment."""
        if rospy.get_time() > self.time_stamp_ry:
            if abs(axes['ry']) > 0.5:
                self.init_z_offset -= 0.005 * math.copysign(1, axes['ry'])
                self.init_z_offset = max(0.025, min(0.06, self.init_z_offset))
                self.update_height = True
            
            if self.update_height and not self.update_param:
                gait_param = self.gait_manager.get_gait_param()
                gait_param['body_height'] = self.init_z_offset
                self.gait_manager.update_param(self.speed_params[1]['period_time'], 0, 0, 0, gait_param, step_num=0)
                self.time_stamp_ry = rospy.get_time() + 0.05
                self.update_height = False

    # --- BUTTON CALLBACKS ---
    def r1_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.speed_mode < 4:
            self.speed_mode += 1
            rospy.loginfo(f"Speed mode: {self.speed_mode}")
            self.board.set_buzzer(2000, 0.1, 0.05, 1)

    def l1_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.speed_mode > 1:
            self.speed_mode -= 1
            rospy.loginfo(f"Speed mode: {self.speed_mode}")
            self.board.set_buzzer(1500, 0.1, 0.05, 1)

    def cross_callback(self, new_state):
        if self.ser:
            if new_state == ButtonState.Pressed: self.ser.write(b"FIRE\n")
            elif new_state == ButtonState.Released: self.ser.write(b"STOP\n")

    def x_button_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.sound: self.sound.play()

    def square_callback(self, new_state):
        duration = 0.5
        if new_state == ButtonState.Pressed:
            self.board.bus_servo_set_position(duration, [[13, 253], [15, 880], [17, 480], [19, 520]])
        elif new_state == ButtonState.Released:
            self.board.bus_servo_set_position(duration, [[13, 707], [15, 840], [17, 480], [19, 133]])
    
    # NEW: Circle Callback from joystick_control 1.py
    def circle_callback(self, new_state):
        """
        Выполняет действие "подняться" (Get Up).
        Автоматически выбирает между lie_to_stand и recline_to_stand
        на основе состояния, определенного через IMU.
        """
        if new_state == ButtonState.Pressed:
            rospy.loginfo(f"Circle (B) button pressed. Current IMU robot state: '{self.robot_state}'.")
            
            # Добавлена новая проверка
            if self.robot_state == 'stand':
                rospy.loginfo("Robot is already in a 'stand' state. No get-up action will be performed.")
                self.board.set_buzzer(1500, 0.1, 0.05, 1)
                return

            action_to_run = None
            log_msg = ""
            
            if self.robot_state == 'lie_to_stand':
                action_to_run = self.lie_to_stand_action_name
                log_msg = "Robot state indicates lying on FRONT. Initiating lie_to_stand."
            elif self.robot_state == 'recline_to_stand':
                action_to_run = self.recline_to_stand_action_name
                log_msg = "Robot state indicates lying on BACK. Initiating recline_to_stand."

            rospy.loginfo(log_msg)
            self.board.set_buzzer(1500, 0.1, 0.05, 1)

            try:
                if self.motion_manager is not None and action_to_run:
                    self.motion_manager.run_action(action_to_run)
                    rospy.loginfo(f"Action '{action_to_run}' initiated successfully.")
                    
                    self.robot_state = 'stand'
                    self.count_lie = 0
                    self.count_recline = 0
                    rospy.logdebug("Robot state set to 'stand' and IMU counters reset after Get Up command.")
                    
                else:
                    rospy.logwarn("MotionManager not initialized or no action specified.")
            except Exception as e:
                rospy.logerr(f"Error calling MotionManager.run_action('{action_to_run}'): {e}")

    def start_callback(self, new_state):
        """Resets the robot's height to default. (Code from your request)."""
        if new_state == ButtonState.Pressed:
            rospy.loginfo("Start button pressed. Resetting body height.")
            self.board.set_buzzer(1900, 0.1, 0.05, 1)
            gait_param = self.gait_manager.get_gait_param()
            t = int(abs(0.025 - self.init_z_offset) / 0.005)
            if t != 0:
                direction = math.copysign(1, 0.025 - self.init_z_offset)
                for _ in range(t):
                    self.init_z_offset += 0.005 * direction
                    gait_param['body_height'] = self.init_z_offset
                    # Use the refactored speed_params dictionary
                    params = self.speed_params[self.speed_mode]
                    gait_param['z_move_amplitude'] = params['z_move_amplitude']
                    self.gait_manager.update_param(params['period_time'], 0.0, 0.0, 0.0, gait_param, step_num=1)
                    time.sleep(0.05)

    # --- Unused Button Callbacks ---
    def select_callback(self, new_state): pass
    def triangle_callback(self, new_state): pass
    def l2_callback(self, new_state): pass
    def r2_callback(self, new_state): pass
    def hat_xl_callback(self, new_state): pass
    def hat_xr_callback(self, new_state): pass
    def hat_yd_callback(self, new_state): pass
    def hat_yu_callback(self, new_state): pass

    # --- Main Joy Message Processor ---
    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        buttons = dict(zip(BUTTON_MAP, joy_msg.buttons))

        axes_changed = any(self.last_axes[key] != value for key, value in axes.items())
        
        if axes_changed:
            try:
                self.axes_callback(axes)
                self.height_callback(axes)
            except Exception as e:
                rospy.logerr(f"Error in axes processing: {str(e)}")

        for key, value in buttons.items():
            if not key: continue
            
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
                callback_name = f"{key}_callback"
                if hasattr(self, callback_name):
                    try:
                        getattr(self, callback_name)(new_state)
                    except Exception as e:
                        rospy.logerr(f"Error in button callback '{callback_name}': {str(e)}")

        self.last_buttons = buttons
        self.last_axes = axes

if __name__ == "__main__":
    try:
        node = JoystickController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"An error occurred in the main execution block: {str(e)}")