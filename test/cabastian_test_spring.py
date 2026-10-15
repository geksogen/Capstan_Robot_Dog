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

# 2. Проверка калибровки (как обычно)
if not axis.motor.is_calibrated or not axis.encoder.is_ready:
    print("Ошибка: Мотор или энкодер не откалиброван/не готов.")
    exit()

# 3. Активация режима управления
print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)
print("Режим активирован.")

try:
    # --- Демонстрация режима "Виртуальная пружина" с демпфированием ---
    print("\n--- НАЧАЛО ТЕСТА: Режим 'Виртуальная пружина' (ПД-контроллер) ---")

    # Переключаемся в режим контроля момента
    print("Переключение в CONTROL_MODE_TORQUE_CONTROL...")
    axis.controller.config.control_mode = 1 # Число для TORQUE_CONTROL
    
    # --- Параметры виртуальной пружины ---
    # УМЕНЬШЕНО: Начнем с очень мягкой пружины, чтобы убрать колебания
    stiffness = 0.1
    # ДОБАВЛЕНО: Коэффициент демпфирования (амортизатор)
    damping = 0.02
    
    center_point = 0.0
    duration = 20
    
    print("\n" + "="*50)
    print(f"АКТИВИРОВАНА ВИТУАЛЬНАЯ ПРУЖИНА (Жесткость: {stiffness}, Демпфирование: {damping})")
    print("Нога будет плавно возвращаться к нулевой позиции.")
    print("Попробуйте отклонить ее рукой и отпустить.")
    print(f"Тест продлится {duration} секунд.")
    print("="*50 + "\n")

    start_time = time.monotonic()
    while time.monotonic() - start_time < duration:
        # 1. Считываем позицию и СКОРОСТЬ
        current_pos = axis.encoder.pos_estimate
        current_vel = axis.encoder.vel_estimate # <--- НОВОЕ
        
        # 2. Вычисляем ошибку
        pos_error = center_point - current_pos
        
        # 3. Вычисляем целевой момент по новой формуле (ПД-контроллер)
        torque_p = stiffness * pos_error      # Пропорциональная часть (пружина)
        torque_d = damping * current_vel      # Дифференциальная часть (амортизатор)
        target_torque = torque_p - torque_d   # <--- ИЗМЕНЕНО
        
        # 4. Задаем момент
        axis.controller.input_torque = target_torque
        
        time.sleep(0.01)

    print("\n--- ЗАВЕРШЕНИЕ ТЕСТА ПРУЖИНЫ ---")
    # Устанавливаем нулевой момент перед сменой режима
    axis.controller.input_torque = 0.0
    print("Возвращение в 'жесткий' режим управления позицией...")
    
    # Возвращаемся в обычный режим
    axis.controller.config.control_mode = 4 # Число для TRAJECTORY_CONTROL
    axis.controller.input_pos = 0
    print("Нога снова жестко удерживает позицию 0.")
    time.sleep(2)


except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # Возврат в режим ожидания
    print("\nВыход из режима управления (AXIS_STATE_IDLE).")
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(1)
    print("Мотор полностью обесточен.")
    odrv0.axis0.clear_errors()
