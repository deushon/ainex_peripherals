#!/usr/bin/env python3
# encoding: utf-8
"""
PID контроллер для стабилизации корпуса робота.
Теперь используем библиотеку simple_pid и оставляем только регулирование по pitch.
"""

try:
    # MIT licensed, компактная и проверенная библиотека PID
    from simple_pid import PID
except ImportError as e:
    raise ImportError(
        "simple_pid не установлен. Установите пакет: pip install simple-pid"
    ) from e


class PidController:
    """PID контроллер для стабилизации корпуса по pitch (x_offset)."""

    def __init__(self, config):
        """
        Инициализирует PID контроллер.

        Args:
            config: Словарь конфигурации PID
        """
        self.config = config.copy()
        self.state = {
            "integral": {"roll": 0.0, "pitch": 0.0},
            "filtered_pitch": None,
            "last_time": None,
        }
        self._build_pid()

    def _build_pid(self):
        """Создает/пересоздает экземпляр simple_pid с актуальной конфигурацией."""
        pitch_target = self.config.get("pitch_target", 90.0)
        sample_time = self.config.get("update_interval", 0.05)
        max_output = self.config.get("pitch_max_output", 0.02)

        self.pid = PID(
            self.config.get("pitch_kp", 0.0),
            self.config.get("pitch_ki", 0.0),
            self.config.get("pitch_kd", 0.0),
            setpoint=pitch_target,
            sample_time=sample_time,
            output_limits=(-max_output, max_output),
            proportional_on_measurement=False,
        )
        self.pid.reset() 

    def calculate(self, roll_deg, pitch_deg, roll_vel, pitch_vel, current_time):
        """
        Вычисляет PID выход для pitch. Остальные параметры оставлены для совместимости.

        Args:
            roll_deg: Угол roll (игнорируется)
            pitch_deg: Угол pitch (градусы)
            roll_vel: Угловая скорость roll (игнорируется)
            pitch_vel: Угловая скорость pitch (игнорируется)
            current_time: Текущее время (сек)

        Returns:
            dict: Словарь с результатами PID {'roll': {...}, 'pitch': {...}}
        """
        # Простая фильтрация угла pitch, если включена
        if self.config.get("filter_enabled", True):
            alpha = self.config.get("filter_alpha", 0.7)
            if self.state["filtered_pitch"] is None:
                self.state["filtered_pitch"] = pitch_deg
            else:
                self.state["filtered_pitch"] = (
                    alpha * self.state["filtered_pitch"] + (1 - alpha) * pitch_deg
                )
            pitch_deg = self.state["filtered_pitch"]

        # Поддерживаем last_time для отладочных данных (PID сам держит sample_time)
        last_time = self.state.get("last_time")
        update_interval = self.config.get("update_interval", 0.05)
        dt = current_time - last_time if last_time else update_interval
        dt = max(0.001, min(0.1, dt))
        self.state["last_time"] = current_time

        # Рассчитываем PID (simple_pid считает ошибку как setpoint - input)
        output = self.pid(pitch_deg)
        p_term, i_term, d_term = self.pid.components
        error = pitch_deg - self.pid.setpoint

        # Обновляем видимую интегральную составляющую для отладки
        self.state["integral"] = {"roll": 0.0, "pitch": getattr(self.pid, "_integral", 0.0)}

        roll_result = {
            "error": 0.0,
            "p_term": 0.0,
            "i_term": 0.0,
            "d_term": 0.0,
            "output": 0.0,
        }
        pitch_result = {
            "error": error,
            "p_term": p_term,
            "i_term": i_term,
            "d_term": d_term,
            "output": output,
        }

        return {"roll": roll_result, "pitch": pitch_result}

    def reset_integral(self):
        """Сбрасывает состояние PID."""
        self.pid.reset()
        self.state["integral"] = {"roll": 0.0, "pitch": 0.0}
        self.state["last_time"] = None

    def update_config(self, new_config):
        """
        Обновляет конфигурацию PID.

        Args:
            new_config: Словарь с новыми значениями конфигурации
        """
        self.config.update(new_config)
        if any(
            key in new_config
            for key in (
                "pitch_kp",
                "pitch_ki",
                "pitch_kd",
                "pitch_target",
                "pitch_max_output",
                "update_interval",
            )
        ):
            self._build_pid()

    def get_state(self):
        """Возвращает текущее состояние контроллера."""
        return self.state.copy()

