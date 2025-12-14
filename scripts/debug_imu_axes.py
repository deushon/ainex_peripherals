#!/usr/bin/env python3
# encoding: utf-8
"""
Отладочный скрипт для проверки физических положений осей робота.
Выводит данные IMU в реальном времени для проверки соответствия физических движений осям.
"""

import rospy
import math
import sys
from sensor_msgs.msg import Imu


class IMUAxesDebugger:
    """Класс для отладки осей IMU."""
    
    def __init__(self):
        rospy.init_node('imu_axes_debugger', anonymous=True)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.data_received = False
        
        print("\n" + "="*60)
        print("  Отладка осей IMU робота")
        print("="*60)
        print("\nИнструкции:")
        print("1. Убедитесь, что ROS запущен и топик /imu активен")
        print("2. Наблюдайте за значениями при физическом перемещении робота:")
        print("   - Наклоните робота ВПЕРЕД - проверьте pitch")
        print("   - Наклоните робота НАЗАД - проверьте pitch")
        print("   - Наклоните робота ВЛЕВО - проверьте roll")
        print("   - Наклоните робота ВПРАВО - проверьте roll")
        print("   - Поверните робота ВЛЕВО - проверьте yaw")
        print("   - Поверните робота ВПРАВО - проверьте yaw")
        print("\nСтандартная система координат для роботов:")
        print("  X: вперед (forward)")
        print("  Y: влево (left)")
        print("  Z: вверх (up)")
        print("\nНажмите Ctrl+C для выхода")
        print("="*60 + "\n")
    
    def imu_callback(self, msg):
        """Обработчик данных IMU."""
        self.data_received = True
        
        # Получаем quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Преобразуем quaternion в углы Эйлера
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Конвертируем в градусы
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Получаем угловую скорость
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        
        # Получаем линейное ускорение
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Очищаем экран и выводим данные
        print("\033[2J\033[H")  # Очистка экрана и перемещение курсора в начало
        print("="*60)
        print("  Отладка осей IMU робота")
        print("="*60)
        print("\nОРИЕНТАЦИЯ (Quaternion):")
        print(f"  qx: {qx:8.4f}  qy: {qy:8.4f}")
        print(f"  qz: {qz:8.4f}  qw: {qw:8.4f}")
        print("\nУГЛЫ ЭЙЛЕРА (градусы):")
        print(f"  Roll (крен, наклон влево/вправо):     {roll_deg:8.2f}°")
        print(f"  Pitch (тангаж, наклон вперед/назад):  {pitch_deg:8.2f}°")
        print(f"  Yaw (рыскание, поворот влево/вправо):  {yaw_deg:8.2f}°")
        print("\nУГЛОВАЯ СКОРОСТЬ (рад/с):")
        print(f"  gx (вращение вокруг X):  {gx:8.3f}")
        print(f"  gy (вращение вокруг Y):  {gy:8.3f}")
        print(f"  gz (вращение вокруг Z):  {gz:8.3f}")
        print("\nЛИНЕЙНОЕ УСКОРЕНИЕ (м/с²):")
        print(f"  ax (ускорение по X):  {ax:8.3f}")
        print(f"  ay (ускорение по Y):  {ay:8.3f}")
        print(f"  az (ускорение по Z):  {az:8.3f}")
        print("\n" + "="*60)
        print("Нажмите Ctrl+C для выхода")
        sys.stdout.flush()
    
    def run(self):
        """Запускает отладчик."""
        try:
            # Ждем получения данных
            timeout = rospy.Duration(5.0)
            start_time = rospy.Time.now()
            while not self.data_received and (rospy.Time.now() - start_time) < timeout:
                rospy.sleep(0.1)
            
            if not self.data_received:
                print("ОШИБКА: Данные IMU не получены за 5 секунд!")
                print("Убедитесь, что:")
                print("  1. ROS master запущен (roscore)")
                print("  2. IMU нода запущена")
                print("  3. Топик /imu публикуется")
                return
            
            rospy.spin()
        except KeyboardInterrupt:
            print("\n\nОтладка завершена.")


if __name__ == '__main__':
    try:
        debugger = IMUAxesDebugger()
        debugger.run()
    except rospy.ROSInterruptException:
        pass

