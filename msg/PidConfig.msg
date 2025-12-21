# Конфигурация PID регулятора стабилизации корпуса
# Используется для настройки PID в реальном времени

Header header

# === Включение/выключение ===
bool pid_enabled          # Глобальное включение/выключение PID
bool roll_enabled         # Включить/выключить стабилизацию по roll
bool pitch_enabled        # Включить/выключить стабилизацию по pitch

# === PID коэффициенты для roll ===
float64 roll_kp           # Пропорциональный коэффициент (м/градус)
float64 roll_ki            # Интегральный коэффициент
float64 roll_kd            # Дифференциальный коэффициент
float64 roll_max_output    # Максимальное смещение по Y (м)
float64 roll_target        # Целевой угол roll (градусы) - вертикально
float64 roll_integral_limit # Лимит интегральной составляющей (anti-windup)

# === PID коэффициенты для pitch ===
float64 pitch_kp          # Пропорциональный коэффициент (м/градус)
float64 pitch_ki           # Интегральный коэффициент
float64 pitch_kd          # Дифференциальный коэффициент
float64 pitch_max_output  # Максимальное смещение по X (м)
float64 pitch_target      # Целевой угол pitch (градусы) - вертикально
float64 pitch_integral_limit # Лимит интегральной составляющей (anti-windup)

# === Пороги активации ===
float64 roll_activation_threshold   # Минимальное отклонение для активации PID (градусы)
float64 pitch_activation_threshold  # Минимальное отклонение для активации PID (градусы)

# === Базовые смещения позы (начальные значения) ===
float64 base_x_offset     # Базовое смещение по X (м)
float64 base_y_offset     # Базовое смещение по Y (м)
float64 base_roll_offset  # Базовое смещение по roll (градусы)
float64 base_pitch_offset # Базовое смещение по pitch (градусы)

# === Пределы смещений от регулятора ===
# Минимальные и максимальные значения смещений, которые может задать PID
float64 limit_x_offset_min
float64 limit_x_offset_max
float64 limit_y_offset_min
float64 limit_y_offset_max
float64 limit_roll_offset_min
float64 limit_roll_offset_max
float64 limit_pitch_offset_min
float64 limit_pitch_offset_max

# === Коэффициенты преобразования PID выхода в смещения туловища ===
float64 roll_to_y_offset_ratio      # Коэффициент для init_y_offset от roll PID
float64 roll_to_roll_offset_ratio   # Коэффициент для init_roll_offset от roll PID
float64 pitch_to_x_offset_ratio     # Коэффициент для init_x_offset от pitch PID
float64 pitch_to_pitch_offset_ratio # Коэффициент для init_pitch_offset от pitch PID

# === Настройки фильтра IMU ===
bool filter_enabled       # Включить/выключить фильтрацию данных IMU
float64 filter_alpha      # Коэффициент фильтрации (0-1), больше = меньше фильтрации

# === Интервал обновления PID ===
float64 update_interval   # Интервал обновления PID корректировок (сек)

# === Флаги применения ===
bool apply_to_walking    # Применить настройки к стабилизации при ходьбе
bool apply_to_rest       # Применить настройки к стабилизации в покое
bool reset_integral      # Сбросить интегральные составляющие при применении

