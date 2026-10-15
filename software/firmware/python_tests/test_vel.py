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

# 2. Проверка, что мотор откалиброван
if not axis.motor.is_calibrated:
    print("Ошибка: Мотор не откалиброван. Пожалуйста, сначала выполните калибровку.")
    exit()

if not axis.encoder.is_ready:
    print("Ошибка: Энкодер не готов. Пожалуйста, сначала выполните калибровку.")
    exit()

# 3. Перевод в режим управления скоростью
print("Активация режима CLOSED_LOOP_CONTROL...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Ждем, пока ODrive перейдет в нужный режим
while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    time.sleep(0.1)

print("Режим активирован.")
print("Установка режима управления скоростью...")
axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

try:
    # 4. Управление скоростью
    target_vel = 2.0  # оборотов в секунду

    print(f"Вращение со скоростью {target_vel} об/с в течение 5 секунд...")
    axis.controller.input_vel = target_vel
    time.sleep(5)

    print(f"Текущая скорость: {axis.encoder.vel_estimate:.2f} об/с")

    print("Вращение в обратную сторону...")
    axis.controller.input_vel = -target_vel
    time.sleep(5)

    print(f"Текущая скорость: {axis.encoder.vel_estimate:.2f} об/с")

    print("Остановка мотора.")
    axis.controller.input_vel = 0

except Exception as e:
    print(f"Произошла ошибка во время управления: {e}")

finally:
    # 5. Возврат в режим ожидания
    print("Выход из режима управления (AXIS_STATE_IDLE).")
    axis.requested_state = AXIS_STATE_IDLE
