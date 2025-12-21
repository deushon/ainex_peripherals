# Отладочные данные для PID регулятора стабилизации корпуса
# Используется для построения графиков и настройки PID

Header header

# === Ориентация робота (из IMU) ===
float64 roll_deg          # Угол roll (градусы)
float64 pitch_deg         # Угол pitch (градусы)
float64 yaw_deg           # Угол yaw (градусы)

# Угловые скорости (рад/с)
float64 roll_velocity     # Угловая скорость по roll
float64 pitch_velocity    # Угловая скорость по pitch
float64 yaw_velocity      # Угловая скорость по yaw

# Линейные ускорения (м/с²)
float64 linear_accel_x
float64 linear_accel_y
float64 linear_accel_z

# === Текущие значения позы для стабилизации ===
# Смещения туловища (текущие значения из gait_param)
float64 init_x_offset     # Смещение по X (м)
float64 init_y_offset     # Смещение по Y (м)
float64 init_roll_offset  # Смещение по roll (градусы)
float64 init_pitch_offset # Смещение по pitch (градусы)

# === Управляющие воздействия от PID ===
# Ошибки (отклонения от целевых значений)
float64 roll_error        # Ошибка по roll (градусы)
float64 pitch_error       # Ошибка по pitch (градусы)

# PID компоненты для roll
float64 roll_p_term       # Пропорциональная составляющая
float64 roll_i_term       # Интегральная составляющая
float64 roll_d_term       # Дифференциальная составляющая
float64 roll_pid_output   # Общий выход PID для roll

# PID компоненты для pitch
float64 pitch_p_term      # Пропорциональная составляющая
float64 pitch_i_term      # Интегральная составляющая
float64 pitch_d_term      # Дифференциальная составляющая
float64 pitch_pid_output  # Общий выход PID для pitch

# Применяемые корректировки (после ограничений и коэффициентов)
float64 applied_x_correction      # Применяемая корректировка по X (м)
float64 applied_y_correction      # Применяемая корректировка по Y (м)
float64 applied_roll_correction   # Применяемая корректировка по roll (градусы)
float64 applied_pitch_correction  # Применяемая корректировка по pitch (градусы)

# === Состояние регулятора ===
bool pid_enabled          # Включен ли PID
bool roll_enabled         # Включена ли стабилизация по roll
bool pitch_enabled        # Включена ли стабилизация по pitch
bool is_walking           # Работает ли робот в режиме ходьбы (true) или покоя (false)

# Интегральные составляющие (для отладки)
float64 roll_integral     # Текущее значение интеграла для roll
float64 pitch_integral    # Текущее значение интеграла для pitch

# Время с последнего обновления (сек)
float64 dt                # Интервал времени с последнего обновления PID

