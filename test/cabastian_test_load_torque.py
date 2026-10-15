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

# 3. Активация режима управления
print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)
print("Режим активирован.")

try:
    # --- Демонстрация режима контроля момента ---
    print("\n--- НАЧАЛО ТЕСТА: Режим контроля момента (Torque Control) ---")

    # --- НОВЫЙ БЛОК: Настройка безопасности ---
    # Устанавливаем лимит скорости как "аварийный тормоз".
    # Если скорость по какой-либо причине (например, при самовозбуждении)
    # превысит это значение, ODrive выдаст ошибку и отключит мотор.
    safety_vel_limit = 2.0 # оборотов/секунду
    print(f"Установка лимита скорости для безопасности: {safety_vel_limit} об/с")
    axis.controller.config.vel_limit = safety_vel_limit
    # --- КОНЕЦ НОВОГО БЛОКА ---

    # Переключаемся в режим контроля момента
    print("Переключение в CONTROL_MODE_TORQUE_CONTROL...")
    # Используем числовое значение 1 для CONTROL_MODE_TORQUE_CONTROL
    axis.controller.config.control_mode = 1
    
    # Устанавливаем нулевой момент
    axis.controller.input_torque = 0.0
    
    print("\n" + "="*50)
    print("ВНИМАНИЕ! МОТОР В РЕЖИМЕ НУЛЕВОГО МОМЕНТА!")
    print("Теперь вы можете свободно двигать ногу рукой.")
    print("Она не будет сопротивляться, но и не будет держать вес.")
    print("Тест продлится 15 секунд.")
    print("="*50 + "\n")

    # Держим этот режим 15 секунд
    time.sleep(15)

    print("\n--- ЗАВЕРШЕНИЕ ТЕСТА МОМЕНТА ---")
    print("Возвращение в режим управления позицией...")
    
    # Возвращаемся в режим управления позицией, чтобы нога снова стала "жесткой"
    # Используем числовое значение 4 для CONTROL_MODE_TRAJECTORY_CONTROL
    axis.controller.config.control_mode = 4
    
    # Даем команду вернуться в ноль для безопасности
    axis.controller.input_pos = 0
    print("Нога снова удерживает позицию 0.")
    time.sleep(2)


except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # Возврат в режим ожидания
    print("\nВыход из режима управления (AXIS_STATE_IDLE).")
    axis.requested_state = AXIS_STATE_IDLE
    
    # Пауза для гарантированного отключения мотора
    print("Ожидание 1 секунду для полного отключения...")
    time.sleep(1)
    
    print("Мотор полностью обесточен.")
    print("Ошибки:", odrv0.axis0.error)
    odrv0.axis0.clear_errors()
    print("Ошибки:", odrv0.axis0.error)
