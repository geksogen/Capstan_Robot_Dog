import odrive
from odrive.enums import *
import time
import math

# --- НОВАЯ ФУНКЦИЯ: "Умное" ожидание завершения движения ---
def wait_for_trajectory_done(axis, timeout=5):
    """
    Ожидает, пока ODrive не завершит движение по траектории.
    Выходит по таймауту, чтобы избежать бесконечного цикла.
    """
    print("  ...ожидание завершения движения...")
    start_time = time.monotonic()
    while not axis.controller.trajectory_done:
        time.sleep(0.01)
        if time.monotonic() - start_time > timeout:
            print("  !!! ОШИБКА: Движение не завершилось за отведенное время!")
            return False
    return True
# --- КОНЕЦ НОВОЙ ФУНКЦИИ ---

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

# --- Переменные для анализа погрешности ---
total_error = 0.0
max_error = 0.0
total_measurements = 0

try:
    # --- Нагрузочный тест: 10 циклов движения ---
    num_cycles = 10
    print(f"\n--- НАЧАЛО НАГРУЗОЧНОГО ТЕСТА: {num_cycles} циклов ---")

    for i in range(num_cycles):
        print(f"\n--- Цикл №{i + 1} из {num_cycles} ---")

        # --- Движение ВПЕРЕД ---
        target_pos = 1.0
        print(f"Движение на {target_pos} оборот(ов)...")
        axis.controller.input_pos = target_pos
        wait_for_trajectory_done(axis) # ИЗМЕНЕНО: "Умное" ожидание
        
        # Анализ погрешности
        current_pos = axis.encoder.pos_estimate
        error = abs(target_pos - current_pos)
        total_error += error
        max_error = max(max_error, error)
        total_measurements += 1
        print(f"Позиция: {current_pos:.3f}, Погрешность: {error:.4f}")

        # --- Возвращение в ноль ---
        target_pos = 0.0
        print("Возвращение в ноль...")
        axis.controller.input_pos = target_pos
        wait_for_trajectory_done(axis) # ИЗМЕНЕНО: "Умное" ожидание

        # Анализ погрешности
        current_pos = axis.encoder.pos_estimate
        error = abs(target_pos - current_pos)
        total_error += error
        max_error = max(max_error, error)
        total_measurements += 1
        print(f"Позиция: {current_pos:.3f}, Погрешность: {error:.4f}")

        # --- Движение НАЗАД ---
        target_pos = -1.0
        print(f"Движение на {target_pos} оборот(ов)...")
        axis.controller.input_pos = target_pos
        wait_for_trajectory_done(axis) # ИЗМЕНЕНО: "Умное" ожидание

        # Анализ погрешности
        current_pos = axis.encoder.pos_estimate
        error = abs(target_pos - current_pos)
        total_error += error
        max_error = max(max_error, error)
        total_measurements += 1
        print(f"Позиция: {current_pos:.3f}, Погрешность: {error:.4f}")

        # --- Возвращение в ноль ---
        target_pos = 0.0
        print("Возвращение в ноль...")
        axis.controller.input_pos = target_pos
        wait_for_trajectory_done(axis) # ИЗМЕНЕНО: "Умное" ожидание

        # Анализ погрешности
        current_pos = axis.encoder.pos_estimate
        error = abs(target_pos - current_pos)
        total_error += error
        max_error = max(max_error, error)
        total_measurements += 1
        print(f"Позиция: {current_pos:.3f}, Погрешность: {error:.4f}")

except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # --- Вывод итоговой статистики ---
    print("\n--- ТЕСТ ЗАВЕРШЕН ---")
    if total_measurements > 0:
        print("\n--- Анализ погрешности позиционирования ---")
        print(f"Максимальная погрешность: {max_error:.4f} оборотов")
        print(f"Средняя погрешность:    {total_error / total_measurements:.4f} оборотов на {total_measurements} измерениях")

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
