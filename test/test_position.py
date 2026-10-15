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
    # --- НОВЫЙ БЛОК: Установка режима траектории и режима ВВОДА ---
    print("Установка режима управления по траектории...")
    # Используем числовое значение 4 для CONTROL_MODE_TRAJECTORY_CONTROL
    axis.controller.config.control_mode = 4
    
    # ИЗМЕНЕНИЕ: Устанавливаем режим ввода, предназначенный для траекторий
    # Используем числовое значение 5 для INPUT_MODE_TRAP_TRAJ
    print("Установка режима ввода для траекторий...")
    axis.controller.config.input_mode = 5
    # ---

    # 4. Управление положением
    # Значение задается в [оборотах]
    target_pos_turns = 3.0

    print(f"Движение в позицию: {target_pos_turns} оборотов")
    # ИЗМЕНЕНИЕ: Используем специальную команду для запуска траектории
    axis.controller.move_to_pos(target_pos_turns)

    # Даем мотору время доехать до цели.
    # Вместо слепого ожидания, будем ждать завершения траектории.
    print("  ...ожидание завершения движения...")
    while not axis.controller.trajectory_done:
        time.sleep(0.1)

    print(f"Текущая позиция: {axis.encoder.pos_estimate:.2f} оборотов")

    print("Возвращение в исходное положение (0 оборотов)")
    # ИЗМЕНЕНИЕ: Используем специальную команду
    axis.controller.move_to_pos(0)

    print("  ...ожидание завершения движения...")
    while not axis.controller.trajectory_done:
        time.sleep(0.1)

    print(f"Текущая позиция: {axis.encoder.pos_estimate:.2f} оборотов")

except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # 5. Возврат в режим ожидания
    print("Выход из режима управления (AXIS_STATE_IDLE).")
    # Используем числовое значение 1 для AXIS_STATE_IDLE
    axis.requested_state = 1
    print("Ошибки:", odrv0.axis0.error)
    odrv0.axis0.clear_errors()
    print("Ошибки:", odrv0.axis0.error)
