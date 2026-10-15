import odrive
from enums import *
import time

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

# 3. Настройка и активация режима управления
print("Настройка режима управления по траектории...")
# Этот режим использует vel_limit и accel_limit из конфигурации для плавного движения
# Используем числовое значение для CONTROL_MODE_TRAJECTORY_CONTROL, так как локальный enums.py устарел
axis.controller.config.control_mode = 4

print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)
print("Режим активирован.")

try:
    # --- Тест: Один оборот ВПЕРЕД и назад ---
    print("\n--- ЭТАП 1: Движение ВПЕРЕД ---")
    target_pos = 0.5
    
    # Движение к цели
    print(f"Движение на {target_pos} оборот(ов)...")
    axis.controller.input_pos = target_pos
    # Ожидаем завершения движения (с запасом)
    time.sleep(3)
    print(f"Позиция достигнута: {axis.encoder.pos_estimate:.3f}")

    # Возвращение в ноль
    print("Возвращение в ноль...")
    axis.controller.input_pos = 0
    # Ожидаем завершения движения (с запасом)
    time.sleep(3)
    print(f"Позиция достигнута: {axis.encoder.pos_estimate:.3f}")


    # --- НОВЫЙ БЛОК: Тест: Один оборот НАЗАД и обратно ---
    print("\n--- ЭТАП 2: Движение НАЗАД ---")
    target_pos = -0.5 # Цель в обратную сторону

    # Движение к цели
    print(f"Движение на {target_pos} оборот(ов)...")
    axis.controller.input_pos = target_pos
    time.sleep(3)
    print(f"Позиция достигнута: {axis.encoder.pos_estimate:.3f}")

    # Возвращение в ноль
    print("Возвращение в ноль...")
    axis.controller.input_pos = 0
    time.sleep(3)
    print(f"Позиция достигнута: {axis.encoder.pos_estimate:.3f}")


except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # Возврат в режим ожидания
    print("\nВыход из режима управления (AXIS_STATE_IDLE).")
    axis.requested_state = AXIS_STATE_IDLE
    
    # ДОБАВЛЕНО: Пауза для гарантированного отключения мотора
    print("Ожидание 1 секунду для полного отключения...")
    time.sleep(1)
    
    print("Мотор полностью обесточен.")
    print("Ошибки:", odrv0.axis0.error)
    odrv0.axis0.clear_errors()
    print("Ошибки:", odrv0.axis0.error)
