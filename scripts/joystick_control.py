#!/usr/bin/env python3
# encoding: utf-8
"""
Точка входа для joystick_control.
"""

import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
package_dir = os.path.dirname(script_dir)

if package_dir not in sys.path:
    sys.path.insert(0, package_dir)

from control.joystick_controller import JoystickController
import rospy

if __name__ == "__main__":
    import traceback
    node = None
    try:
        node = JoystickController()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
    except SystemExit as e:
        rospy.logerr(f"SystemExit with code: {e.code}")
        rospy.logerr(f"Traceback: {traceback.format_exc()}")
        raise
    except Exception as e:
        rospy.logerr(f"An error occurred: {type(e).__name__}: {str(e)}")
        rospy.logerr(f"Traceback: {traceback.format_exc()}")
        raise
    finally:
        if node is not None:
            try:
                rospy.loginfo("Stopping robot in finally block...")
                # Принудительно обнуляем амплитуды
                node.x_move_amplitude = 0.0
                node.y_move_amplitude = 0.0
                node.angle_move_amplitude = 0.0
                node.status = 'stop'
                # Останавливаем робота несколько раз для надежности
                try:
                    node.gait_manager.stop()
                    rospy.sleep(0.1)  # Даем время на остановку
                    node.gait_manager.stop()  # Повторная попытка
                except Exception as e:
                    rospy.logerr(f"Error stopping gait_manager: {e}")
                node.serial_handler.stop()
                rospy.loginfo("Robot stopped in finally block")
            except Exception as e:
                rospy.logerr(f"Error in finally block: {e}")
