import odrive
from odrive.enums import *
import time
import math

# 1. Подключение к ODrive
print("Поиск ODrive...")
try:
    odrv0 = odrive.find_any()
    print(f"Подключено к ODrive: {odrv0.serial_number}")
except Exception as e:
    print(f"Ошибка подключения: {e}")
    exit()

# Присваиваем ось для удобства
axis = odrv0.axis0

print("Очистка ошибок")
odrv0.axis0.clear_errors()


# 2. Проверка, что мотор откалиброван
if not axis.motor.is_calibrated:
    print("Ошибка: Мотор не откалиброван. Пожалуйста, сначала выполните калибровку.")
    exit()

if not axis.encoder.is_ready:
    print("Ошибка: Энкодер не готов. Пожалуйста, сначала выполните калибровку.")
    exit()

# 3. Перевод в режим управления положением
print("Активация режима CLOSED_LOOP_CONTROL...")
# Используем числовое значение 8 для AXIS_STATE_CLOSED_LOOP_CONTROL
axis.requested_state = 8

# Ждем, пока ODrive перейдет в нужный режим
while axis.current_state != 8:
    time.sleep(0.1)

print("Режим активирован.")

try:
    # --- Установка режима траектории (старый, совместимый метод) ---
    print("Установка режима управления по траектории...")
    # Используем числовое значение 4 для CONTROL_MODE_TRAJECTORY_CONTROL
    axis.controller.config.control_mode = 4
    
    # --- ВАЖНО: Увеличиваем допуск по скорости для старых прошивок ---
    # Это предотвратит "тихий" сбой, если мотор на старте немного превысит vel_limit
    print("Увеличение допуска по лимиту скорости...")
    axis.controller.config.vel_limit_tolerance = 1.5 
    # ---

    # 4. Управление положением
    # Значение input_pos задается в [оборотах]
    target_pos_turns = 3.0

    print(f"Движение в позицию: {target_pos_turns} оборотов")
    # Используем старый метод задания цели
    axis.controller.input_pos = target_pos_turns

    # Используем простое ожидание
    time.sleep(4)

    print(f"Текущая позиция: {axis.encoder.pos_estimate:.2f} оборотов")

    print("Возвращение в исходное положение (0 оборотов)")
    axis.controller.input_pos = 0

    time.sleep(4)

    print(f"Текущая позиция: {axis.encoder.pos_estimate:.2f} оборотов")

except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # Возвращаем допуск к значению по умолчанию
    axis.controller.config.vel_limit_tolerance = 1.2

    # 5. Возврат в режим ожидания
    print("Выход из режима управления (AXIS_STATE_IDLE).")
    # Используем числовое значение 1 для AXIS_STATE_IDLE
    axis.requested_state = 1
    print("Ошибки:", odrv0.axis0.error)
    odrv0.axis0.clear_errors()
    print("Ошибки:", odrv0.axis0.error)
