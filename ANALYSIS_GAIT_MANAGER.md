# Анализ зависимостей и использования GaitManager

## Архитектура зависимостей

```
joystick_control.py (главный контроллер)
    │
    ├──> GaitManager (создается здесь, передается в модули)
    │
    ├──> SpeedControl
    │       └──> получает: gait_manager
    │       └──> использует: gait_manager.get_gait_param()
    │       └──> использует: gait_manager.set_step()
    │       └──> использует: gait_manager.update_param()
    │       └──> использует: gait_manager.stop()
    │
    ├──> AutoStabilization
    │       └──> получает: gait_manager, speed_params, speed_mode, init_z_offset
    │       └──> использует: gait_manager.get_gait_param()
    │       └──> использует: gait_manager.set_step()
    │       └──> синхронизируется с: speed_mode из SpeedControl
    │
    └──> ButtonActions
            └──> получает: gait_manager, speed_control
            └──> использует: gait_manager.get_gait_param()
            └──> использует: gait_manager.update_param()
            └──> использует: speed_control.get_speed_params()
```

## Детальный анализ использования GaitManager

### 1. joystick_control.py

**Создание:**
- Строка 120: `self.gait_manager = GaitManager()` - единственный экземпляр

**Передача в модули:**
- Строка 141: `SpeedControl(self.gait_manager)` - передается по ссылке
- Строка 150: `AutoStabilization(..., self.gait_manager, ...)` - передается по ссылке
- Строка 157: `ButtonActions(..., self.gait_manager, ...)` - передается по ссылке

**Прямое использование:**
- Строка 311: `self.gait_manager.stop()` - остановка при блокировке движения

**Синхронизация данных:**
- Строки 315-319: Синхронизация `speed_mode` и `speed_params` между `SpeedControl` и `AutoStabilization`
- Строка 351: Синхронизация `init_z_offset` с `AutoStabilization`

### 2. speed_control.py

**Хранение:**
- Строка 39: `self.gait_manager = gait_manager` - хранит ссылку

**Использование в process_axes():**
- Строка 118: `gait_param = self.gait_manager.get_gait_param()` - получение параметров
- Строка 123: `gait_param.update(params.get('gait_base', {}))` - обновление параметров из конфига
- Строки 130-144: Динамическое изменение параметров в зависимости от режима скорости и осей джойстика
  - `init_roll_offset` - изменяется в режимах 2, 3, 4
  - `init_y_offset` - изменяется в режимах 2, 3
  - `y_swap_amplitude` - изменяется в режимах 2, 3
- Строка 157: `gait_param['init_z_offset'] = init_z_offset` - установка высоты
- Строки 171, 181: `self.gait_manager.set_step(...)` - установка движения
  - Параметры: `period_time`, `x_amplitude`, `y_amplitude`, `angle_amplitude`, `gait_param`, `step_num=0`
  - НЕ используется: `arm_swap` (по умолчанию 30°)
- Строка 192: `self.gait_manager.stop()` - остановка движения

**Использование в process_height():**
- Строка 219: `gait_param = self.gait_manager.get_gait_param()` - получение параметров
- Строка 220: `gait_param['body_height'] = new_init_z_offset` - установка высоты
- Строка 222: `self.gait_manager.update_param(...)` - обновление без движения
  - Параметры: `period_time`, `0, 0, 0` (без движения), `gait_param`, `step_num=0`

**Параметры gait_param, используемые в speed_control:**
- `init_z_offset` / `body_height` - высота корпуса
- `init_roll_offset` - крен (динамически меняется)
- `init_y_offset` - смещение по Y (динамически меняется)
- `y_swap_amplitude` - амплитуда обмена по Y (динамически меняется)
- `dsp_ratio` - из `gait_base` (режимы 2-4)
- `step_fb_ratio` - из `gait_base` (режимы 2-4)
- `z_swap_amplitude` - из `gait_base` (режимы 2-4)
- `pelvis_offset` - из `gait_base` (режим 4)
- `arm_swing_gain` - из `gait_base` (режим 4)

**Параметры gait_param, НЕ используемые:**
- `init_x_offset` - не устанавливается явно
- `init_pitch_offset` - не используется
- `init_yaw_offset` - не используется
- `hip_pitch_offset` - не используется
- `step_height` - не используется напрямую (используется `z_move_amplitude` в конфиге)
- `angle_move_amplitude` - устанавливается через `set_step()`, но не в `gait_param`
- `move_aim_on` - не используется

### 3. auto_stabilization.py

**Хранение:**
- Строка 78: `self.gait_manager = gait_manager` - хранит ссылку
- Строка 79: `self.speed_params = speed_params` - копия параметров из SpeedControl
- Строка 80: `self.speed_mode = speed_mode` - текущий режим скорости

**Синхронизация:**
- Синхронизируется с `SpeedControl` через `joystick_control.py` (строки 315-319)
- При изменении `speed_mode` обновляются `speed_params`

**Использование в _make_step():**
- Строка 362: `gait_param = self.gait_manager.get_gait_param()` - получение параметров
- Строка 363: `params = self.speed_params[self.speed_mode]` - получение параметров режима
- Строка 364: `period_time = list(params['period_time'])` - копирование периода
- Строка 368: `period_time[0] = int(period_time[0] * speed_factor)` - динамическое изменение скорости
- Строка 371: `gait_param.update(params.get('gait_base', {}))` - обновление базовых параметров (режимы > 1)
- Строка 373: `gait_param['init_z_offset'] = self.init_z_offset` - установка высоты
- Строка 376: `self.gait_manager.set_step(...)` - выполнение шага стабилизации
  - Параметры: `period_time` (динамически измененный), `step_x`, `step_y`, `0` (без поворота), `gait_param`, `step_num=1`
  - НЕ используется: `arm_swap` (по умолчанию 30°)

**Параметры gait_param, используемые в auto_stabilization:**
- `init_z_offset` - высота корпуса
- Параметры из `gait_base` (режимы > 1):
  - `dsp_ratio`
  - `step_fb_ratio`
  - `y_swap_amplitude`
  - `z_swap_amplitude`
  - `init_y_offset`
  - `pelvis_offset` (режим 4)
  - `arm_swing_gain` (режим 4)

**Параметры gait_param, НЕ используемые:**
- Все те же, что и в `speed_control.py`
- Дополнительно: не используется динамическая настройка `arm_swap` в зависимости от `change_magnitude`

### 4. button_actions.py

**Хранение:**
- Строка 65: `self.gait_manager = gait_manager` - хранит ссылку
- Строка 66: `self.speed_control = speed_control` - для доступа к параметрам

**Использование в reset_height():**
- Строка 208: `gait_param = self.gait_manager.get_gait_param()` - получение параметров
- Строка 214: `gait_param['body_height'] = init_z_offset` - установка высоты
- Строка 215: `params = self.speed_control.get_speed_params()[1]` - получение параметров режима 1
- Строка 216: `gait_param['z_move_amplitude'] = params['z_move_amplitude']` - установка высоты шага
- Строка 217: `self.gait_manager.update_param(...)` - обновление с одним шагом
  - Параметры: `period_time`, `0.0, 0.0, 0.0` (без движения), `gait_param`, `step_num=1`
  - НЕ используется: `arm_swap` (по умолчанию 30°)

**Параметры gait_param, используемые в button_actions:**
- `body_height` - высота корпуса
- `z_move_amplitude` - высота подъема ноги

## Поток данных

### Инициализация:
1. `joystick_control` создает `GaitManager`
2. Передает его в `SpeedControl`, `AutoStabilization`, `ButtonActions`
3. `SpeedControl` создает `speed_params` (конфигурация режимов)
4. `AutoStabilization` получает копию `speed_params` и `speed_mode`

### Во время работы:

**Обработка джойстика (joystick_control -> speed_control):**
1. `axes_callback()` получает данные джойстика
2. Проверяет синхронизацию `speed_mode` между модулями
3. Вызывает `speed_control.process_axes()`
4. `speed_control` получает `gait_param`, модифицирует его, вызывает `gait_manager.set_step()`
5. Результат возвращается в `joystick_control` для автостабилизации

**Автостабилизация (joystick_control -> auto_stabilization):**
1. `imu_callback()` получает данные IMU
2. Вызывает `auto_stabilization.process()`
3. Если нужно, вызывает `auto_stabilization._make_step()`
4. `_make_step()` получает `gait_param`, модифицирует его, вызывает `gait_manager.set_step()`

**Изменение высоты (joystick_control -> speed_control):**
1. `height_callback()` получает данные джойстика
2. Вызывает `speed_control.process_height()`
3. `speed_control` получает `gait_param`, изменяет `body_height`, вызывает `gait_manager.update_param()`

**Сброс высоты (joystick_control -> button_actions):**
1. `start_callback()` вызывается при нажатии кнопки
2. Вызывает `button_actions.reset_height()`
3. `button_actions` получает `gait_param`, изменяет `body_height`, вызывает `gait_manager.update_param()` в цикле

## Проблемы и возможности для улучшения

### 1. Дублирование параметров
- `init_z_offset` и `body_height` - это одно и то же, но используются разные ключи
- В `speed_control` используется `init_z_offset`, в `button_actions` - `body_height`

### 2. Неиспользуемые параметры
- `arm_swap` - всегда используется значение по умолчанию (30°)
- `init_pitch_offset` - не используется, но может помочь при стабилизации
- `init_yaw_offset` - не используется
- `hip_pitch_offset` - не используется
- `update_pose()` - не используется, но может быть полезен для плавной коррекции

### 3. Синхронизация
- `speed_mode` синхронизируется вручную в `joystick_control`
- `init_z_offset` синхронизируется вручную между модулями
- Можно улучшить через общий state manager

### 4. Возможности для динамической настройки
- `arm_swap` можно менять в зависимости от `change_magnitude` в автостабилизации
- `init_pitch_offset` можно использовать для компенсации наклона вперед/назад
- `update_pose()` можно использовать для плавной коррекции позы без движения
- `set_body_height()` можно использовать вместо ручного цикла в `button_actions`

