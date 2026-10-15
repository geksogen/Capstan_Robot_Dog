import odrive
from enums import *
import time
import math
import matplotlib.pyplot as plt
import csv

# --- НОВАЯ ФУНКЦИЯ: Сохранение данных в CSV ---
def save_data_to_csv(filename, time_pts, target_pts, actual_pts):
    """
    Сохраняет собранные данные в CSV-файл.
    """
    print(f"\nСохранение данных в файл: {filename}")
    header = ['time_s', 'target_pos_turns', 'actual_pos_turns']
    
    rows = zip(time_pts, target_pts, actual_pts)

    try:
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
        print("Данные успешно сохранены.")
    except Exception as e:
        print(f"!!! Ошибка при сохранении файла: {e}")
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
if not axis.motor.is_calibrated or not axis.encoder.is_ready:
    print("Ошибка: Мотор или энкодер не откалиброван/не готов.")
    exit()

# 3. Настройка и активация режима управления
print("Настройка режима прямого управления позицией...")
axis.controller.config.control_mode = 3 # CONTROL_MODE_POSITION_CONTROL

# --- ИЗМЕНЕНИЕ: Плавный старт без рывка ---
print("Определение текущей позиции для плавного старта...")
# 1. Считываем текущую позицию мотора
start_pos = axis.encoder.pos_estimate
# 2. Устанавливаем ее как первую цель, чтобы избежать рывка
axis.controller.input_pos = start_pos
print(f"  Стартовая позиция определена: {start_pos:.3f} оборотов")

print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)
print("Режим активирован.")
# --- КОНЕЦ ИЗМЕНЕНИЯ ---

# --- Списки для сбора данных для графика ---
time_points = []
target_positions = []
actual_positions = []
# ---

try:
    # --- Параметры движения по синусоиде ---
    swing_amplitude = 0.5
    frequency = 0.8 # 0.5 Гц = 1 полный цикл за 2 секунды (0.8)
    duration = 150

    # ИЗМЕНЕНИЕ: Используем стартовую позицию как центр колебаний
    center_point = start_pos

    print(f"\n--- НАЧАЛО ТЕСТА: Движение по синусоиде вокруг точки {center_point:.3f} ---")
    print(f"Амплитуда: +/-{swing_amplitude * 360}°, Частота: {frequency} Гц, Длительность: {duration} с")

    start_time = time.monotonic()
    while time.monotonic() - start_time < duration:
        elapsed_time = time.monotonic() - start_time
        
        # ИЗМЕНЕНИЕ: Вычисляем целевую позицию относительно центральной точки
        sine_offset = swing_amplitude * math.sin(elapsed_time * frequency * 2 * math.pi)
        target_pos = center_point + sine_offset
        
        # Отправляем команду
        axis.controller.input_pos = target_pos
        
        # --- Сбор данных ---
        current_pos = axis.encoder.pos_estimate
        
        time_points.append(elapsed_time)
        target_positions.append(target_pos)
        actual_positions.append(current_pos)
        # ---
        
        time.sleep(0.01)

    # Плавное возвращение в ЦЕНТРАЛЬНУЮ точку
    print(f"\nПлавное возвращение в центральную позицию ({center_point:.3f})...")
    axis.controller.config.control_mode = 4 # TRAJECTORY_CONTROL
    axis.controller.input_pos = center_point # Возвращаемся в исходную точку
    
    # "Умное" ожидание
    start_time = time.monotonic()
    while not axis.controller.trajectory_done:
        time.sleep(0.01)
        if time.monotonic() - start_time > 5: break
            
    print(f"Финальная позиция: {axis.encoder.pos_estimate:.3f}")

except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # Возврат в режим ожидания
    print("\n--- ТЕСТ ЗАВЕРШЕН ---")
    axis.requested_state = AXIS_STATE_IDLE
    print("Мотор обесточен.")

# --- Построение графика ---
if time_points:
    save_data_to_csv('sine_wave_data.csv', time_points, target_positions, actual_positions)
    
    print("\nПостроение графика...")
    fig, ax = plt.subplots()
    ax.plot(time_points, target_positions, '--', label='Целевая позиция (Setpoint)')
    ax.plot(time_points, actual_positions, label='Фактическая позиция (Encoder)')
    
    ax.set_xlabel('Время (с)')
    ax.set_ylabel('Позиция (обороты)')
    ax.set_title('Отслеживание траектории синусоиды')
    ax.legend()
    ax.grid(True)
    
    plt.show()
# ---
