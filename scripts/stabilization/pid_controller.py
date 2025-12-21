#!/usr/bin/env python3
# encoding: utf-8
"""
PID контроллер для стабилизации корпуса робота.
Вычисляет управляющие воздействия на основе ориентации IMU.
"""

import math


class PidController:
    """PID контроллер для стабилизации корпуса по roll и pitch."""
    
    def __init__(self, config):
        """
        Инициализирует PID контроллер.
        
        Args:
            config: Словарь конфигурации PID
        """
        self.config = config.copy()
        self.state = {
            'integral': {'roll': 0.0, 'pitch': 0.0},
            'filtered_orientation': {'roll': 0.0, 'pitch': 90.0},
            'last_time': None,
        }
    
    def calculate(self, roll_deg, pitch_deg, roll_vel, pitch_vel, current_time):
        """
        Вычисляет PID выходы для roll и pitch.
        
        Args:
            roll_deg: Угол roll (градусы)
            pitch_deg: Угол pitch (градусы)
            roll_vel: Угловая скорость roll (град/с)
            pitch_vel: Угловая скорость pitch (град/с)
            current_time: Текущее время (сек)
        
        Returns:
            dict: Словарь с результатами PID для roll и pitch
        """
        # Применяем фильтрацию, если включена
        if self.config.get('filter_enabled', True):
            alpha = self.config.get('filter_alpha', 0.7)
            # Инициализируем при первом вызове
            if self.state['filtered_orientation']['roll'] == 0.0 and \
               self.state['filtered_orientation']['pitch'] == 90.0:
                self.state['filtered_orientation']['roll'] = roll_deg
                self.state['filtered_orientation']['pitch'] = pitch_deg
            else:
                self.state['filtered_orientation']['roll'] = \
                    alpha * self.state['filtered_orientation']['roll'] + (1 - alpha) * roll_deg
                self.state['filtered_orientation']['pitch'] = \
                    alpha * self.state['filtered_orientation']['pitch'] + (1 - alpha) * pitch_deg
            roll_deg = self.state['filtered_orientation']['roll']
            pitch_deg = self.state['filtered_orientation']['pitch']
        
        # Вычисляем ошибки относительно целевых углов
        roll_error = roll_deg - self.config.get('roll_target', 0.0)
        pitch_error = pitch_deg - self.config.get('pitch_target', 90.0)
        
        # Вычисляем dt
        last_time = self.state.get('last_time')
        update_interval = self.config.get('update_interval', 0.05)
        dt = current_time - last_time if last_time else update_interval
        dt = max(0.001, min(0.1, dt))  # Ограничиваем dt разумными пределами
        self.state['last_time'] = current_time
        
        # Вычисляем PID выходы
        roll_result = self._calculate_pid(
            error=roll_error,
            integral=self.state['integral']['roll'],
            derivative=roll_vel,
            dt=dt,
            axis='roll'
        )
        roll_result['error'] = roll_error
        
        pitch_result = self._calculate_pid(
            error=pitch_error,
            integral=self.state['integral']['pitch'],
            derivative=pitch_vel,
            dt=dt,
            axis='pitch'
        )
        pitch_result['error'] = pitch_error
        
        # Обновляем интегральные составляющие
        self.state['integral']['roll'] += roll_error * dt
        self.state['integral']['pitch'] += pitch_error * dt
        
        # Ограничиваем интегральные составляющие (anti-windup)
        roll_integral_limit = self.config.get('roll_integral_limit', 10.0)
        pitch_integral_limit = self.config.get('pitch_integral_limit', 10.0)
        self.state['integral']['roll'] = max(-roll_integral_limit, 
                                             min(roll_integral_limit, self.state['integral']['roll']))
        self.state['integral']['pitch'] = max(-pitch_integral_limit, 
                                              min(pitch_integral_limit, self.state['integral']['pitch']))
        
        return {
            'roll': roll_result,
            'pitch': pitch_result
        }
    
    def _calculate_pid(self, error, integral, derivative, dt, axis):
        """
        Вычисляет выход PID контроллера для одной оси.
        
        Args:
            error: Текущая ошибка
            integral: Интегральная составляющая
            derivative: Производная (угловая скорость)
            dt: Интервал времени
            axis: Ось ('roll' или 'pitch')
        
        Returns:
            dict: Словарь с компонентами PID и выходом
        """
        # Берем значения напрямую из конфигурации (без дефолтных значений)
        # Если значения нет в конфигурации, это ошибка конфигурации
        kp = self.config[f'{axis}_kp']
        ki = self.config[f'{axis}_ki']
        kd = self.config[f'{axis}_kd']
        max_output = self.config[f'{axis}_max_output']
        
        # P компонента
        p_term = kp * error
        
        # I компонента
        i_term = ki * integral
        
        # D компонента
        d_term = kd * derivative
        
        # Суммируем и ограничиваем
        output = p_term + i_term + d_term
        output = max(-max_output, min(max_output, output))
        
        return {
            'p_term': p_term,
            'i_term': i_term,
            'd_term': d_term,
            'output': output
        }
    
    def reset_integral(self):
        """Сбрасывает интегральные составляющие."""
        self.state['integral'] = {'roll': 0.0, 'pitch': 0.0}
    
    def update_config(self, new_config):
        """
        Обновляет конфигурацию PID.
        
        Args:
            new_config: Словарь с новыми значениями конфигурации
        """
        self.config.update(new_config)
    
    def get_state(self):
        """Возвращает текущее состояние контроллера."""
        return self.state.copy()

