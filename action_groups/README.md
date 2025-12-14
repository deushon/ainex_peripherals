# Action Groups

Эта папка содержит action groups для робота. Все action groups хранятся здесь и версионируются в git.

## Используемые action groups

- `lie_to_stand` - подъем из положения лежа (используется в button_actions.py)
- `recline_to_stand` - подъем из положения откинувшись назад (используется в button_actions.py)
- `BACK_UP` - подъем при падении назад (автоматический подъем, используется в joystick_control.py)
- `LEFT_UP` - подъем при падении влево (автоматический подъем, используется в joystick_control.py)
- `RIGHT_UP` - подъем при падении вправо (автоматический подъем, используется в joystick_control.py)

## Текущие action groups

Все необходимые action groups уже скопированы в эту папку:
- ✅ `lie_to_stand.d6a` - подъем из положения лежа
- ✅ `recline_to_stand.d6a` - подъем из положения откинувшись назад
- ✅ `BACK_UP.d6a` - подъем при падении назад
- ✅ `LEFT_UP.d6a` - подъем при падении влево
- ✅ `RIGHT_UP.d6a` - подъем при падении вправо

## Как добавить новые action groups

1. Скопируйте файлы action groups в эту папку (`/home/ubuntu/ros_ws/src/ainex_peripherals/action_groups/`)
2. Убедитесь, что имена файлов точно соответствуют именам, используемым в коде
3. Файлы должны быть в формате `.d6a` (или другом формате, который понимает MotionManager)
4. Добавьте файлы в git: `git add action_groups/`

## Автоматический поиск

Код автоматически ищет action groups в этой папке. Если файл не найден здесь, MotionManager будет искать его в стандартных местах.

## Использование в коде

Action groups используются в:
- `scripts/button_actions.py` - для ручного подъема (кнопка Circle)
- `scripts/joystick_control.py` - для автоматического подъема при падении (через imu_handler)
- `scripts/imu_handler.py` - для автоматического подъема при падении

Путь к action groups настраивается автоматически в `joystick_control.py` при инициализации.

