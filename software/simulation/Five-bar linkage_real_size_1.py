import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class AaedmusaLeg2D:
    def __init__(self, e=0.065, c=0.100, d=0.150, h=0.054):
        self.e = e  # 65 мм -> 0.065 м (расстояние между моторами)
        self.c = c  # 100 мм -> 0.100 м (длина ведущего плеча)
        self.d = d  # 150 мм -> 0.150 м (длина ведомого плеча)
        self.h = h  # 54 мм -> 0.054 м (смещение вниз относительно оси Abad)

        # Координаты моторов А (левый) и В (правый) с учетом смещения h
        self.xA, self.zA = -e / 2.0, -h
        self.xB, self.zB = e / 2.0, -h

    def inverse_kinematics(self, x, z):
        """Расчет углов моторов для конфигурации РОМБ (без пересечения плеч)"""
        c2 = self.c ** 2
        d2 = self.d ** 2

        # --- Левый мотор (A) ---
        L_A = np.hypot(x - self.xA, z - self.zA)
        if L_A > (self.c + self.d) or L_A < abs(self.c - self.d):
            return None

        gamma_A = np.arctan2(z - self.zA, x - self.xA)
        alpha_A = np.arccos((c2 + L_A ** 2 - d2) / (2 * self.c * L_A))

        # --- Правый мотор (B) ---
        L_B = np.hypot(x - self.xB, z - self.zB)
        if L_B > (self.c + self.d) or L_B < abs(self.c - self.d):
            return None

        gamma_B = np.arctan2(z - self.zB, x - self.xB)
        alpha_B = np.arccos((c2 + L_B ** 2 - d2) / (2 * self.c * L_B))

        # Разводим локти строго НАРУЖУ без перекрещивания в центре
        theta_a = gamma_A - alpha_A
        theta_b = gamma_B + alpha_B

        return theta_a, theta_b

    def get_joint_positions(self, x, z):
        """Расчет 2D-координат всех шарниров для визуализации звеньев"""
        angles = self.inverse_kinematics(x, z)
        if angles is None:
            return None
        ta, tb = angles

        # Координаты коленных шарниров (K_A и K_B)
        xKA = self.xA + self.c * np.cos(ta)
        zKA = self.zA + self.c * np.sin(ta)
        xKB = self.xB + self.c * np.cos(tb)
        zKB = self.zB + self.c * np.sin(tb)

        return (self.xA, self.zA), (xKA, zKA), (xKB, zKB), (self.xB, self.zB), (x, z)


# --- Инициализация геометрии ноги проекта aaedmusa ---
leg = AaedmusaLeg2D()

# --- Параметры траектории шага (эллипс) ---
num_steps = 60
t = np.linspace(0, 2 * np.pi, num_steps)

z_offset = -0.19  # Высота нейтральной стойки робота (19 см от оси Abad)
trajectory_x = 0.05 * np.cos(t)  # Длина шага 10 см (+- 5 см вперед/назад)
trajectory_z = z_offset + 0.025 * np.sin(t)  # Высота подъема стопы 2.5 см

# Фаза опоры (stance phase) — выпрямляем низ эллипса в линию земли
trajectory_z = np.where(trajectory_z > z_offset, z_offset, trajectory_z)

# --- Настройка окна визуализации matplotlib ---
fig, ax = plt.subplots(figsize=(7, 7))
ax.set_xlim(-0.18, 0.18)
ax.set_ylim(-0.28, 0.02)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.5)
ax.set_title("Правильная симуляция ноги (Широкий РОМБ без пересечений)", fontsize=11)
ax.set_xlabel("X (Вперед / Назад), м")
ax.set_ylabel("Z (Высота от оси Abad), м")

# Отрисовка графических слоев
path_line, = ax.plot([], [], 'g--', alpha=0.6, label="Траектория стопы")
leg_lines, = ax.plot([], [], 'o-', lw=3, color='#1f77b4', ms=8, label="Звенья ноги")
base_line, = ax.plot([leg.xA, leg.xB], [leg.zA, leg.zB], 'ks-', lw=4, ms=10,
                     label=f"Блок моторов (h={int(leg.h * 1000)}мм)")

# Красный маркер — условный центр вращения абдукции (0,0) на корпусе
ax.plot(0, 0, 'rx', ms=10, lw=2, label="Ось Abad (0,0)")
ax.legend(loc="upper right")


def update(frame):
    x_target = trajectory_x[frame]
    z_target = trajectory_z[fra  me]

    points = leg.get_joint_positions(x_target, z_target)
    if points is None:
        return leg_lines, path_line

    (A, KA, KB, B, P) = points

    # ИСПРАВЛЕНО: Теперь берем отдельно X [индекс 0] и отдельно Z [индекс 1] из каждой точки!
    nodes_x = [A[0], KA[0], P[0], KB[0], B[0]]
    nodes_z = [A[1], KA[1], P[1], KB[1], B[1]]

    leg_lines.set_data(nodes_x, nodes_z)
    path_line.set_data(trajectory_x[:frame + 1], trajectory_z[:frame + 1])

    return leg_lines, path_line


# Запуск циклической анимации
ani = FuncAnimation(fig, update, frames=num_steps, interval=30, blit=True, repeat=True)
plt.show()
