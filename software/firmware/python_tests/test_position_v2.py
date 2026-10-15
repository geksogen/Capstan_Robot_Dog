import odrive
from odrive.enums import * # ИСПРАВЛЕНО: Используем enums из библиотеки
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

# 3. Перевод в режим управления с обратной связью
print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)
print("Режим активирован.")

try:
    # --- ЧАСТЬ 1: Движение по синусоиде с округлением до 3 знаков ---
    print("\n--- НАЧАЛО ТЕСТА 1: Движение по синусоиде с округлением ---")
    # Убедимся, что мы в режиме прямого управления позицией
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    
    t0 = time.monotonic()
    for i in range(5): # Уменьшено для быстрого теста
        setpoint_raw = 0.5 * math.sin((time.monotonic() - t0) * 2)
        setpoint = round(setpoint_raw, 3)
        
        print(f"Движение № {i+1} в позицию: {setpoint} оборотов")
        axis.controller.input_pos = setpoint
        time.sleep(1)
        print(f"Текущая позиция: {axis.encoder.pos_estimate:.3f} оборотов")
    
    # Возврат в ноль перед следующим тестом
    axis.controller.input_pos = 0
    time.sleep(2)


    # --- ЧАСТЬ 2: Вращение с разными скоростями в режиме VELOCITY_CONTROL ---
    print("\n--- НАЧАЛО ТЕСТА 2: Вращение с разными скоростями (Velocity Control) ---")
    
    # Активируем режим управления по скорости
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    
    speeds_to_test = [1.0, 3.0, -2.0] # Список скоростей для теста (в об/с), включая отрицательную

    for speed in speeds_to_test:
        print(f"\nТестируем скорость: {speed} об/с")
        
        # Запускаем вращение
        print(f"Вращение со скоростью {speed} об/с в течение 3 секунд...")
        axis.controller.input_vel = speed
        time.sleep(3)
        
        # Остановка
        print("Остановка мотора...")
        axis.controller.input_vel = 0
        # Даем время на полную остановку (особенно важно после высоких скоростей)
        time.sleep(2) 
        print(f"Текущая позиция после остановки: {axis.encoder.pos_estimate:.3f}")


except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # Возврат в режим ожидания
    print("\nВыход из режима управления (AXIS_STATE_IDLE).")
    axis.requested_state = AXIS_STATE_IDLE
    print("Ошибки:", odrv0.axis0.error)
    odrv0.axis0.clear_errors()
    print("Ошибки:", odrv0.axis0.error)
