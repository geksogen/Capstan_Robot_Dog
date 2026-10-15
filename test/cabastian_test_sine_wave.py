import odrive
from enums import *
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
if not axis.motor.is_calibrated or not axis.encoder.is_ready:
    print("Ошибка: Мотор или энкодер не откалиброван/не готов.")
    exit()

# 3. Настройка и активация режима управления
print("Настройка режима прямого управления позицией...")
# Для плавного отслеживания траектории лучше подходит прямое управление позицией,
# так как мы будем отправлять команды с высокой частотой.
# Используем числовое значение 3 для CONTROL_MODE_POSITION_CONTROL
axis.controller.config.control_mode = 3

print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)
print("Режим активирован.")

try:
    # --- Сохраняем исходное значение интегратора ---
    original_integrator_gain = axis.controller.config.vel_integrator_gain

    # --- Параметры движения по синусоиде ---
    swing_amplitude = 0.25  # Амплитуда качания в оборотах (0.25 = 90 градусов)
    frequency = 0.5         # Частота в Гц (0.5 Гц = 1 полный цикл за 2 секунды)
    duration = 120           # Общая длительность теста в секундах

    # --- Настройка и проверка безопасности ---
    MAX_ROTATION_LIMIT = 1.0
    MIN_ROTATION_LIMIT = -1.0

    print(f"\nУстановлены лимиты вращения: от {MIN_ROTATION_LIMIT} до {MAX_ROTATION_LIMIT} оборотов.")
    if not (MIN_ROTATION_LIMIT <= swing_amplitude <= MAX_ROTATION_LIMIT):
        print("!!! ОШИБКА БЕЗОПАСНОСТИ: Заданная амплитуда выходит за лимиты!")
        exit()

    print(f"\n--- НАЧАЛО ТЕСТА: Движение по синусоиде ---")
    print(f"Амплитуда: +/-{swing_amplitude * 360}°, Частота: {frequency} Гц, Длительность: {duration} с")

    # --- ВРЕМЕННО ОТКЛЮЧАЕМ ИНТЕГРАТОР НА ВРЕМЯ ВСЕХ ДИНАМИЧЕСКИХ ДВИЖЕНИЙ ---
    if original_integrator_gain > 0:
        print(f"Временно отключаем интегратор (было: {original_integrator_gain})...")
        axis.controller.config.vel_integrator_gain = 0.0
    # ---

    start_time = time.monotonic()
    while time.monotonic() - start_time < duration:
        # 1. Вычисляем прошедшее время
        elapsed_time = time.monotonic() - start_time
        
        # 2. Вычисляем целевую позицию по формуле синуса
        # 2 * math.pi - для преобразования частоты в радианы/секунду
        target_pos = swing_amplitude * math.sin(elapsed_time * frequency * 2 * math.pi)
        
        # 3. Отправляем команду (проверка на лимиты здесь не нужна, т.к. амплитуда уже проверена)
        axis.controller.input_pos = target_pos
        
        # 4. Небольшая пауза, чтобы цикл выполнялся примерно 100 раз в секунду
        time.sleep(0.01)

    # --- Возвращение в ноль в конце теста ---
    print("\nПлавное возвращение в ноль (без интегратора)...")
    axis.controller.config.control_mode = 4 
    axis.controller.input_pos = 0
    
    # Ждем завершения основного движения
    start_time = time.monotonic()
    while not axis.controller.trajectory_done:
        time.sleep(0.01)
        if time.monotonic() - start_time > 5:
            print("  !!! ОШИБКА: Движение не завершилось за отведенное время!")
            break
    print(f"Позиция после основного движения: {axis.encoder.pos_estimate:.3f}")

    # --- НОВЫЙ БЛОК: Финальная коррекция с помощью интегратора ---
    # Теперь, когда движение почти завершено, включаем интегратор,
    # чтобы убрать статическую ошибку от гравитации.
    if original_integrator_gain > 0:
        print(f"Включаем интегратор ({original_integrator_gain}) для финальной коррекции...")
        axis.controller.config.vel_integrator_gain = original_integrator_gain
        # Даем интегратору время "дотянуть" позицию до идеала
        time.sleep(0.5)
    # ---

    print(f"Финальная позиция после коррекции: {axis.encoder.pos_estimate:.3f}")


except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # --- ВОССТАНАВЛИВАЕМ ИНТЕГРАТОР ОБРАТНО (на случай прерывания теста) ---
    if 'original_integrator_gain' in locals() and axis.controller.config.vel_integrator_gain != original_integrator_gain:
        print(f"\nВосстанавливаем исходные настройки интегратора ({original_integrator_gain})...")
        axis.controller.config.vel_integrator_gain = original_integrator_gain
    # ---

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
