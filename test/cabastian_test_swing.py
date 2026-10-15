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
# Используем числовое значение для CONTROL_MODE_TRAJECTORY_CONTROL, так как локальный enums.py устарел
axis.controller.config.control_mode = 4

print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)
print("Режим активирован.")

try:
    # --- Параметры теста качания ---
    swing_amplitude = 0.35  # Амплитуда качания в оборотах (0.25 = 90 градусов)
    num_cycles = 10          # Количество полных циклов качания
    # Время на движение в одну сторону. Должно быть достаточным с учетом vel_limit.
    move_time = 0.2

    # --- НОВЫЙ БЛОК: Настройка и проверка безопасности ---
    MAX_ROTATION_LIMIT = 1.0  # Максимальная позиция: 1 оборот
    MIN_ROTATION_LIMIT = -1.0 # Минимальная позиция: -1 оборот

    print(f"\nУстановлены лимиты вращения: от {MIN_ROTATION_LIMIT} до {MAX_ROTATION_LIMIT} оборотов.")

    # Проверяем, не выходят ли параметры теста за пределы безопасности
    if not (MIN_ROTATION_LIMIT <= swing_amplitude <= MAX_ROTATION_LIMIT and \
            MIN_ROTATION_LIMIT <= -swing_amplitude <= MAX_ROTATION_LIMIT):
        print("!!! ОШИБКА БЕЗОПАСНОСТИ: Заданная амплитуда качания выходит за установленные лимиты!")
        print("!!! Тест прерван.")
        exit()
    # --- КОНЕЦ НОВОГО БЛОКА ---

    print(f"\n--- НАЧАЛО ТЕСТА КАЧАНИЯ: {num_cycles} циклов по +/-{swing_amplitude * 360}° ---")

    # Сначала вернемся в ноль для чистоты эксперимента
    axis.controller.input_pos = 0
    time.sleep(move_time)

    for i in range(num_cycles):
        print(f"\n--- Цикл №{i + 1} из {num_cycles} ---")

        # --- Движение в положительную сторону ---
        target_pos = swing_amplitude
        # Проверка перед отправкой команды (хорошая практика)
        if MIN_ROTATION_LIMIT <= target_pos <= MAX_ROTATION_LIMIT:
            print(f"Движение в +{target_pos * 360}°...")
            axis.controller.input_pos = target_pos
            time.sleep(move_time)
            print(f"Позиция: {axis.encoder.pos_estimate:.3f}")
        else:
            print(f"ОТМЕНА: Цель {target_pos} выходит за лимиты безопасности.")

        # --- Движение в отрицательную сторону ---
        target_pos = -swing_amplitude
        # Проверка перед отправкой команды (хорошая практика)
        if MIN_ROTATION_LIMIT <= target_pos <= MAX_ROTATION_LIMIT:
            print(f"Движение в {target_pos * 360}°...")
            axis.controller.input_pos = target_pos
            time.sleep(move_time)
            print(f"Позиция: {axis.encoder.pos_estimate:.3f}")
        else:
            print(f"ОТМЕНА: Цель {target_pos} выходит за лимиты безопасности.")

    # Возвращение в ноль в конце теста
    print("\nВозвращение в ноль...")
    axis.controller.input_pos = 0
    time.sleep(move_time)
    print(f"Финальная позиция: {axis.encoder.pos_estimate:.3f}")


except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # Возврат в режим ожидания
    print("\n--- ТЕСТ ЗАВЕРШЕН ---")
    print("Выход из режима управления (AXIS_STATE_IDLE).")
    axis.requested_state = AXIS_STATE_IDLE
    
    # Пауза для гарантированного отключения мотора
    print("Ожидание 1 секунду для полного отключения...")
    time.sleep(1)
    
    print("Мотор полностью обесточен.")
    print("Ошибки:", odrv0.axis0.error)
    odrv0.axis0.clear_errors()
    print("Ошибки:", odrv0.axis0.error)
