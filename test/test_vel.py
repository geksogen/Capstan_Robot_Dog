import odrive
from enums import * # Убрано во избежание конфликтов
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

# 3. Перевод в режим управления
print("Активация режима CLOSED_LOOP_CONTROL...")
# Используем числовое значение 8 для AXIS_STATE_CLOSED_LOOP_CONTROL
axis.requested_state = 8

# Ждем, пока ODrive перейдет в нужный режим
while axis.current_state != 8:
    time.sleep(0.1)

print("Режим активирован.")

try:
    # --- Установка правильного режима ---
    print("Переключение в режим управления скоростью (VELOCITY_CONTROL)...")
    # Используем числовое значение 2 для CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.control_mode = 2
    axis.encoder.set_linear_count(0)
    time.sleep(0.5)
    speeds_to_test = [2.0, -4.0, 6.0] 

    for speed in speeds_to_test:
        print(f"\nТестируем скорость: {speed} об/с")
        
        # Запускаем вращение
        print(f"Вращение со скоростью {speed} об/с в течение 3 секунд...")
        axis.controller.input_vel = speed
        time.sleep(3)
        
        # Остановка
        print("Остановка мотора...")
        axis.controller.input_vel = 0
        # Даем время на полную остановку
        time.sleep(1) 
        
        current_pos = axis.encoder.pos_estimate
        print(f"Текущая позиция после остановки: {current_pos:.3f}")


except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # 5. Возврат в режим ожидания
    print("\nВыход из режима управления (AXIS_STATE_IDLE).")
    # Используем числовое значение 1 для AXIS_STATE_IDLE
    axis.requested_state = 1
    time.sleep(1)
    print("Мотор обесточен.")
    print("Ошибки:", odrv0.axis0.error)
    odrv0.axis0.clear_errors()
    print("Ошибки:", odrv0.axis0.error)
