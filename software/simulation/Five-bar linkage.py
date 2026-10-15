import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class FiveBarVisualizer:
    def __init__(self, e, c, d):
        self.e = e  # Расстояние между моторами
        self.c = c  # Длина верхних звеньев (плечо)
        self.d = d  # Длина нижних звеньев (предплечье)

        # Координаты моторов (базовые шарниры)
        self.xA, self.zA = -e / 2.0, 0.0
        self.xB, self.zB = e / 2.0, 0.0

    def inverse_kinematics(self, x, z):
        """Вычисление углов моторов (обратная кинематика)"""

        def solve_angle(xM, zM, side="left"):
            L = np.hypot(x - xM, z - zM)
            if L > (self.c + self.d) or L < abs(self.c - self.d):
                return None
            gamma = np.arctan2(z - zM, x - xM)
            alpha = np.arccos((self.c ** 2 + L ** 2 - self.d ** 2) / (2 * self.c * L))
            return gamma + alpha if side == "left" else gamma - alpha

        theta_a = solve_angle(self.xA, self.zA, "left")
        theta_b = solve_angle(self.xB, self.zB, "right")
        return theta_a, theta_b

    def get_joint_positions(self, x, z):
        """Находит точные координаты коленей для отрисовки звеньев"""
        angles = self.inverse_kinematics(x, z)
        if angles[0] is None or angles[1] is None:
            return None
        ta, tb = angles

        # Координаты коленей
        xKA = self.xA + self.c * np.cos(ta)
        zKA = self.zA + self.c * np.sin(ta)
        xKB = self.xB + self.c * np.cos(tb)
        zKB = self.zB + self.c * np.sin(tb)

        return (self.xA, self.zA), (xKA, zKA), (xKB, zKB), (self.xB, self.zB), (x, z)


# --- Настройки геометрии робособаки (в метрах) ---
leg = FiveBarVisualizer(e=0.05, c=0.09, d=0.14)

# --- Генерация траектории шага (эллипс) ---
num_steps = 60
t = np.linspace(0, 2 * np.pi, num_steps)
# Центр эллипса (высота ноги) и его радиусы (длина и высота шага)
z_offset = -0.16
trajectory_x = 0.04 * np.cos(t)
trajectory_z = z_offset + 0.02 * np.sin(t)
# Прижимаем нижнюю часть эллипса, имитируя опору о землю (stance phase)
trajectory_z = np.where(trajectory_z > z_offset, z_offset, trajectory_z)

# --- Настройка графика matplotlib ---
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-0.15, 0.15)
ax.set_ylim(-0.22, 0.05)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.5)
ax.set_title("Симуляция пятизвенника (Five-Bar Linkage)")
ax.set_xlabel("X (Вылет вперед/назад), м")
ax.set_ylabel("Z (Высота ноги), м")

# Отрисовка элементов механизма
path_line, = ax.plot([], [], 'g--', alpha=0.6, label="Траектория")  # Траектория стопы
leg_lines, = ax.plot([], [], 'o-', lw=3, color='#1f77b4', ms=8, label="Звенья ноги")  # Сама нога
base_line, = ax.plot([leg.xA, leg.xB], [leg.zA, leg.zB], 'ks-', lw=4, ms=10, label="Корпус (Моторы)")  # База

ax.legend(loc="upper right")


def update(frame):
    x_target = trajectory_x[frame]
    z_target = trajectory_z[frame]

    points = leg.get_joint_positions(x_target, z_target)
    if points is None:
        return leg_lines, path_line

    (A, KA, KB, B, P) = points

    # Собираем координаты звеньев последовательно: A -> KA -> P -> KB -> B
    nodes_x = [A[0], KA[0], P[0], KB[0], B[0]]
    nodes_z = [A[1], KA[1], P[1], KB[1], B[1]]

    leg_lines.set_data(nodes_x, nodes_z)
    path_line.set_data(trajectory_x[:frame + 1], trajectory_z[:frame + 1])

    return leg_lines, path_line


# Запуск анимации
ani = FuncAnimation(fig, update, frames=num_steps, interval=30, blit=True, repeat=True)
plt.show()
