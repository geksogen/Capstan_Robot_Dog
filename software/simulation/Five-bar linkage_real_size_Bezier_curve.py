import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class AaedmusaLeg2D:
    def __init__(self, e=0.065, c=0.100, d=0.150, h=0.054):
        self.e = e  # 65 мм
        self.c = c  # 100 мм
        self.d = d  # 150 мм
        self.h = h  # 54 мм

        # Координаты моторов с учетом смещения h
        self.xA, self.zA = -e / 2.0, -h
        self.xB, self.zB = e / 2.0, -h

    def inverse_kinematics(self, x, z):
        """Расчет углов моторов для конфигурации РОМБ (в радианах)"""
        c2 = self.c ** 2
        d2 = self.d ** 2

        # --- Левый мотор (A) ---
        L_A = np.hypot(x - self.xA, z - self.zA)
        if L_A > (self.c + self.d) or L_A < abs(self.c - self.d): return None
        gamma_A = np.arctan2(z - self.zA, x - self.xA)
        alpha_A = np.arccos((c2 + L_A ** 2 - d2) / (2 * self.c * L_A))

        # --- Правый мотор (B) ---
        L_B = np.hypot(x - self.xB, z - self.zB)
        if L_B > (self.c + self.d) or L_B < abs(self.c - self.d): return None
        gamma_B = np.arctan2(z - self.zB, x - self.xB)
        alpha_B = np.arccos((c2 + L_B ** 2 - d2) / (2 * self.c * L_B))

        # Ромб без перекрещивания
        theta_a = gamma_A - alpha_A
        theta_b = gamma_B + alpha_B

        return theta_a, theta_b

    def get_joint_positions(self, x, z):
        angles = self.inverse_kinematics(x, z)
        if angles is None: return None
        ta, tb = angles

        xKA = self.xA + self.c * np.cos(ta)
        zKA = self.zA + self.c * np.sin(ta)
        xKB = self.xB + self.c * np.cos(tb)
        zKB = self.zB + self.c * np.sin(tb)

        return (self.xA, self.zA), (xKA, zKA), (xKB, zKB), (self.xB, self.zB), (x, z)


# --- Генерация траектории по Кривой Безье ---
def bezier_curve(p0, p1, p2, p3, num_pts):
    """Вычисляет точки кубической кривой Безье без использования оператора :="""
    t = np.linspace(0, 1, num_pts)
    t_col = t  # Обычное понятное присваивание на отдельной строке

    # Формула кубической кривой Безье
    curve = (1 - t_col)[:, None] ** 3 * p0 + \
            3 * (1 - t_col)[:, None] ** 2 * t[:, None] * p1 + \
            3 * (1 - t_col)[:, None] * t[:, None] ** 2 * p2 + \
            t[:, None] ** 3 * p3
    return curve


def generate_gait_trajectory(z_clearance, step_length, num_steps):
    """
    Генерирует полный цикл шага:
    50% времени — ровная опора (Stance)
    50% времени — плавный перенос ноги вверх по Безье (Swing)
    """
    half_steps = num_steps // 2

    # 1. Фаза опоры (Stance Phase) — строго горизонтальная линия
    x_stance = np.linspace(step_length / 2, -step_length / 2, half_steps)
    z_stance = np.full(half_steps, z_clearance)
    stance_pts = np.column_stack((x_stance, z_stance))

    # 2. Фаза переноса (Swing Phase) — кривая Безье
    # Опорные точки для Безье (задают форму траектории переноса)
    p0 = np.array([-step_length / 2, z_clearance])  # Старт отрыва от земли
    p1 = np.array([-step_length / 2, z_clearance + 0.045])  # Подъем вертикально вверх
    p2 = np.array([step_length / 2, z_clearance + 0.045])  # Перенос вперед на высоте
    p3 = np.array([step_length / 2, z_clearance])  # Мягкое опускание на землю

    swing_pts = bezier_curve(p0, p1, p2, p3, half_steps)

    # Соединяем обе фазы в единый цикл
    full_trajectory = np.vstack((stance_pts, swing_pts))
    return full_trajectory[:, 0], full_trajectory[:, 1]


# --- Инициализация и настройки ---
leg = AaedmusaLeg2D()
num_steps = 80
z_height = -0.19  # Высота робота (19 см)
stride = 0.09  # Длина шага (9 см)

trajectory_x, trajectory_z = generate_gait_trajectory(z_height, stride, num_steps)

# --- Расчет массивов углов моторов для графика ---
angles_deg_A = []
angles_deg_B = []
for i in range(num_steps):
    res = leg.inverse_kinematics(trajectory_x[i], trajectory_z[i])
    if res:
        angles_deg_A.append(np.degrees(res[0]))
        angles_deg_B.append(np.degrees(res[1]))
    else:
        angles_deg_A.append(0)
        angles_deg_B.append(0)

# --- Создание окна с двумя графиками (Subplots) ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))
plt.subplots_adjust(wspace=0.3)

# Настройка Левого графика (Визуализация ноги)
ax1.set_xlim(-0.16, 0.16)
ax1.set_ylim(-0.27, 0.02)
ax1.set_aspect('equal')
ax1.grid(True, linestyle='--', alpha=0.5)
ax1.set_title("Кинематика ноги (Кривая Безье)")
ax1.set_xlabel("X (Вперед / Назад), м")
ax1.set_ylabel("Z (Высота от оси Abad), м")

path_line, = ax1.plot([], [], 'g-', lw=1.5, alpha=0.7, label="Траектория Безье")
leg_lines, = ax1.plot([], [], 'o-', lw=3, color='#1f77b4', ms=8, label="Звенья ноги")
base_line, = ax1.plot([leg.xA, leg.xB], [leg.zA, leg.zB], 'ks-', lw=4, ms=10, label="Корпус (Моторы)")
ax1.plot(0, 0, 'rx', ms=10, lw=2, label="Ось Abad (0,0)")
ax1.legend(loc="upper right")

# Настройка Правого графика (Углы моторов)
ax2.grid(True, linestyle='--', alpha=0.5)
ax2.set_title("Текущие углы BLDC моторов")
ax2.set_xlabel("Шаг временного цикла (Frame)")
ax2.set_ylabel("Угол мотора, Градусы")
ax2.set_xlim(0, num_steps)
# Динамически выставляем лимиты по углам с запасом
ax2.set_ylim(min(min(angles_deg_A), min(angles_deg_B)) - 10, max(max(angles_deg_A), max(angles_deg_B)) + 10)

angle_A_line, = ax2.plot([], [], color='red', lw=2, label="Мотор А (Левый)")
angle_B_line, = ax2.plot([], [], color='blue', lw=2, label="Мотор B (Правый)")
current_frame_marker = ax2.axvline(x=0, color='gray', linestyle=':', alpha=0.8)
ax2.legend(loc="upper right")


# --- Функция обновления анимации ---
def update(frame):
    x_t = trajectory_x[frame]
    z_t = trajectory_z[frame]

    # Обновление геометрии ноги
    points = leg.get_joint_positions(x_t, z_t)
    if points is not None:
        (A, KA, KB, B, P) = points
        leg_lines.set_data([A[0], KA[0], P[0], KB[0], B[0]], [A[1], KA[1], P[1], KB[1], B[1]])

    # Обновление линии траектории
    path_line.set_data(trajectory_x[:frame + 1], trajectory_z[:frame + 1])

    # Обновление графиков углов
    angle_A_line.set_data(range(frame + 1), angles_deg_A[:frame + 1])
    angle_B_line.set_data(range(frame + 1), angles_deg_B[:frame + 1])
    current_frame_marker.set_xdata([frame])

    return leg_lines, path_line, angle_A_line, angle_B_line


# Запуск анимации
ani = FuncAnimation(fig, update, frames=num_steps, interval=25, blit=False, repeat=True)
plt.show()
