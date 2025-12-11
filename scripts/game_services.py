#!/usr/bin/env python3
# encoding: utf-8
"""
Модуль сервисов для игры: HP, урон, разрешения, статус матча.
"""

# ========== КОНФИГУРАЦИЯ ==========
MAX_HP = 100  # Максимальное HP робота
HP_PUBLISH_INTERVAL = 0.5  # Интервал публикации HP (секунды)
# ===================================

import rospy
import json
from std_msgs.msg import String, Int32, Bool
from std_srvs.srv import Trigger, TriggerResponse


class GameServices:
    """
    Класс для управления игровыми сервисами: HP, урон, разрешения.
    """
    
    def __init__(self, robot_id):
        """
        Инициализация модуля игровых сервисов.
        
        Args:
            robot_id: ID робота
        """
        self.robot_id = robot_id
        
        # Состояние HP
        self.max_hp = MAX_HP
        self.current_hp = MAX_HP
        
        # Система разрешений
        self.permissions = {
            'movement': False,
            'head': True,
            'firing': False,
            'camera': True
        }
        self.use_detailed_permissions = False
        self.is_locked = False  # Legacy флаг
        
        # Публикаторы
        self.hp_pub = rospy.Publisher('/game/hp', Int32, queue_size=10)
        self.robot_status_pub = rospy.Publisher('/game/robot_status', String, queue_size=10)
        
        # Подписчики
        self.damage_sub = rospy.Subscriber('/game/damage', Int32, self.damage_callback)
        self.control_permissions_sub = rospy.Subscriber('/game/control_permissions', String, self.control_permissions_callback)
        self.control_lock_sub = rospy.Subscriber('/game/control_lock', Bool, self.control_lock_callback)
        self.match_start_sub = rospy.Subscriber('/game/match_start', Bool, self.match_start_callback)
        
        # Сервисы
        self.reset_hp_service = rospy.Service('/game/reset_hp', Trigger, self.reset_hp_service_handler)
        
        # Таймер для периодической публикации HP
        rospy.Timer(rospy.Duration(HP_PUBLISH_INTERVAL), self.publish_hp_status)
        
        rospy.loginfo(f"GameServices initialized for robot: {self.robot_id}")
    
    def damage_callback(self, msg):
        """Обработчик урона от сервера."""
        damage_amount = msg.data
        if damage_amount <= 0:
            return
        
        old_hp = self.current_hp
        self.current_hp = max(0, self.current_hp - damage_amount)
        
        rospy.loginfo(f"💥 Damage received: {damage_amount}, HP: {old_hp} → {self.current_hp}")
        
        # Публикуем обновленное HP
        hp_msg = Int32()
        hp_msg.data = self.current_hp
        self.hp_pub.publish(hp_msg)
        
        # Если HP достигло 0, блокируем робота
        if self.current_hp <= 0:
            self.is_locked = True
            self.permissions['movement'] = False
            self.permissions['head'] = False
            self.permissions['firing'] = False
            self.permissions['camera'] = True
            rospy.logwarn(f"💀 Robot HP reached 0, locking robot")
            self.publish_robot_status("hp_zero_locked")
        
        return self.current_hp <= 0  # Возвращаем True если HP = 0
    
    def control_permissions_callback(self, msg):
        """Обработчик детальных разрешений от сервера."""
        try:
            data = json.loads(msg.data)
            if 'movement' in data:
                self.permissions['movement'] = bool(data['movement'])
            if 'head' in data:
                self.permissions['head'] = bool(data['head'])
            if 'firing' in data:
                self.permissions['firing'] = bool(data['firing'])
            if 'camera' in data:
                self.permissions['camera'] = bool(data['camera'])
            
            self.use_detailed_permissions = True
            rospy.loginfo(f"Updated permissions: {self.permissions}")
            self.publish_robot_status("permissions_changed")
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing control permissions JSON: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing control permissions: {e}")
    
    def control_lock_callback(self, msg):
        """Обработчик legacy команд блокировки/разблокировки."""
        if self.use_detailed_permissions:
            rospy.logdebug("Ignoring legacy control_lock message - using detailed permissions")
            return
        
        self.is_locked = msg.data
        if self.is_locked:
            self.permissions = {
                'movement': False,
                'head': False,
                'firing': False,
                'camera': True
            }
            rospy.loginfo("Robot control locked by server (legacy mode).")
        else:
            self.permissions = {
                'movement': True,
                'head': True,
                'firing': True,
                'camera': True
            }
            rospy.loginfo("Robot control unlocked by server (legacy mode).")
        
        self.publish_robot_status("lock_changed" if self.is_locked else "unlock_changed")
    
    def match_start_callback(self, msg):
        """Обработчик начала нового матча."""
        if msg.data:
            rospy.loginfo("🎮 Match started, resetting HP")
            self.reset_hp()
    
    def reset_hp(self):
        """Сбросить HP на максимум."""
        self.current_hp = self.max_hp
        hp_msg = Int32()
        hp_msg.data = self.current_hp
        self.hp_pub.publish(hp_msg)
        rospy.loginfo(f"💚 HP reset to {self.max_hp}")
        
        # Разблокировать робота при сбросе HP
        self.is_locked = False
        self.permissions = {
            'movement': True,
            'head': True,
            'firing': True,
            'camera': True
        }
        self.publish_robot_status("hp_reset")
    
    def reset_hp_service_handler(self, req):
        """Обработчик сервиса сброса HP."""
        response = TriggerResponse()
        try:
            self.reset_hp()
            response.success = True
            response.message = f"HP reset to {self.max_hp}"
        except Exception as e:
            response.success = False
            response.message = f"Error resetting HP: {str(e)}"
            rospy.logerr(response.message)
        return response
    
    def publish_hp_status(self, event):
        """Публикует HP для синхронизации."""
        hp_msg = Int32()
        hp_msg.data = self.current_hp
        self.hp_pub.publish(hp_msg)
    
    def publish_robot_status(self, status_type):
        """Публикует общий статус робота на сервер."""
        status_data = {
            'robot_id': self.robot_id,
            'status_type': status_type,
            'hp': self.current_hp,
            'locked': self.is_locked,
            'timestamp': rospy.get_time()
        }
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.robot_status_pub.publish(status_msg)
    
    def can_move(self):
        """Проверка разрешения на движение."""
        if self.use_detailed_permissions:
            return self.permissions['movement']
        else:
            return not self.is_locked
    
    def can_control_head(self):
        """Проверка разрешения на управление головой."""
        if self.use_detailed_permissions:
            return self.permissions['head']
        else:
            return not self.is_locked
    
    def can_fire(self):
        """Проверка разрешения на стрельбу."""
        if self.use_detailed_permissions:
            return self.permissions['firing']
        else:
            return not self.is_locked
    
    def can_access_camera(self):
        """Проверка разрешения на доступ к камере."""
        if self.use_detailed_permissions:
            return self.permissions['camera']
        else:
            return True  # Камера всегда доступна в legacy режиме

