import numpy as np
import casadi as ca

from src.config import FINE_X_COEF, FINE_Y_COEF, FINE_V_COEF, FINE_STEER_COEF, FINE_ACC_COEF, \
    FINE_STEER_DOT_COEF, FINE_ACC_DOT_COEF, N, dt, V_REF, FINE_THETA_COEF, FINE_LATERAL_COEF


def evaluate_first_step_cost(x, y, v, theta, acc, steer, acc0, steer0, x_ref, y_ref, v_ref, theta_ref, PATH_TOLERANCE_M):
    """Оценивает вклад в cost для первого шага управления (k=0)"""

    # Оценка отклонения по x для первого шага
    if abs(x - x_ref[0]) > PATH_TOLERANCE_M:
        fine_x = FINE_X_COEF * (x - x_ref[0]) ** 2
    else:
        fine_x = 0

    if abs(y - y_ref[0]) > PATH_TOLERANCE_M:
        fine_y = FINE_Y_COEF * (y - y_ref[0]) ** 2
    else:
        fine_y = 0

    # Рассчет поперечного отклонения (между ближайшими двумя точками траектории)
    lateral_deviation = calculate_lateral_deviation(x, y, x_ref[0], y_ref[0], x_ref[1], y_ref[1])

    fine_lateral = FINE_LATERAL_COEF * lateral_deviation ** 2

    fine_v = FINE_V_COEF * (v - v_ref[0]) ** 2

    fine_theta = FINE_THETA_COEF * (theta - theta_ref[0]) ** 2

    fine_steer = FINE_STEER_COEF * steer ** 2
    fine_acc = FINE_ACC_COEF * acc ** 2

    fine_steer_dot = FINE_STEER_DOT_COEF * (steer - steer0) ** 2
    fine_acc_dot = FINE_ACC_DOT_COEF * (acc - acc0) ** 2

    # Суммарный вклад для первого шага
    total_cost_first_step = fine_x + fine_y + fine_v + fine_steer + fine_acc + fine_steer_dot + fine_acc_dot + fine_theta + fine_lateral

    # Возвращаем результат по частям для анализа
    return {
        "fine_x": fine_x,
        "fine_y": fine_y,
        "fine_v": fine_v,
        "fine_steer": fine_steer,
        "fine_acc": fine_acc,
        "fine_steer_dot": fine_steer_dot,
        "fine_acc_dot": fine_acc_dot,
        "fine_theta": fine_theta,
        "fine_lateral": fine_lateral,
        "total_cost_first_step": total_cost_first_step
    }

def update_reference_point(x0, y0, current_idx, x_traj, y_traj, min_distance=5.0):
    """
    Обновляет индекс референсной точки на траектории, когда автомобиль проходит через текущую точку.

    :param x0: Текущая координата x автомобиля.
    :param y0: Текущая координата y автомобиля.
    :param current_idx: Текущий индекс на траектории.
    :param x_traj: Массив с координатами x траектории.
    :param y_traj: Массив с координатами y траектории.
    :param min_distance: Минимальное расстояние до точки, при котором она обновляется.
    :return: Обновленный индекс на траектории.
    """
    # Рассчитываем расстояние до текущей точки траектории
    distance_to_current_point = np.sqrt((x_traj[current_idx] - x0) ** 2 + (y_traj[current_idx] - y0) ** 2)

    # Если расстояние меньше минимального, сдвигаем точку вперед
    if distance_to_current_point < min_distance:
        # Обновляем индекс на следующий, если не достигнут конец траектории
        current_idx = (current_idx + 1) % len(x_traj)  # Циклическое перемещение по траектории

    return current_idx

def draw_full_trajectory(carla, world, x_traj, y_traj):
    for i in range(len(x_traj) - 1):
        start_point = carla.Location(x=x_traj[i], y=y_traj[i], z=0.2)
        end_point = carla.Location(x=x_traj[i + 1], y=y_traj[i + 1], z=0.2)
        world.debug.draw_line(start_point, end_point, thickness=0.1, color=carla.Color(0, 255, 0), life_time=dt * 2)


def get_eight_trajectory(x_init, y_init, total_points=100):
    # Параметры для восьмерки
    a = 25  # Масштаб восьмерки
    T = 2 * np.pi  # Период для траектории

    # Генерация полной траектории "восьмерки"
    t_values = np.linspace(0, T, total_points)
    x_traj = x_init + a * np.sin(t_values)
    y_traj = y_init + a * np.sin(t_values) * np.cos(t_values)

    # Примерная скорость для каждого шага
    v_ref = [V_REF for _ in range(total_points)]

    # Поворот траектории на 90 градусов (π/2 радиан) по часовой стрелке
    cos_angle = np.cos(-np.pi / 4)
    sin_angle = np.sin(-np.pi / 4)

    # Применение матрицы поворота
    x_traj_rotated = x_init + cos_angle * (x_traj - x_init) - sin_angle * (y_traj - y_init)
    y_traj_rotated = y_init + sin_angle * (x_traj - x_init) + cos_angle * (y_traj - y_init)

    # Вычисляем угол направления движения для каждой точки траектории
    theta_ref = []
    for i in range(total_points - 1):
        dx = x_traj_rotated[i + 1] - x_traj_rotated[i]
        dy = y_traj_rotated[i + 1] - y_traj_rotated[i]
        theta = np.arctan2(dy, dx)  # Направление движения (угол ориентации)
        theta_ref.append(theta)

    # Для последней точки можем использовать угол направления последней секции
    theta_ref.append(theta_ref[-1])

    return x_traj_rotated, y_traj_rotated, v_ref, theta_ref

def get_ref_trajectory(x_traj, y_traj, theta_traj, current_idx):
    if current_idx + N < len(x_traj):
        x_ref = x_traj[current_idx:current_idx + N]
        y_ref = y_traj[current_idx:current_idx + N]
        theta_ref = theta_traj[current_idx:current_idx + N]
    else:
        x_ref = np.concatenate((x_traj[current_idx:], x_traj[:N - (len(x_traj) - current_idx)]))
        y_ref = np.concatenate((y_traj[current_idx:], y_traj[:N - (len(x_traj) - current_idx)]))
        theta_ref = np.concatenate((theta_traj[current_idx:], theta_traj[:N - (len(x_traj) - current_idx)]))
    return x_ref, y_ref, theta_ref

def draw_ref_trajectory(carla, world, x_ref, y_ref):
    for i in range(N - 1):
        start_point = carla.Location(x=x_ref[i], y=y_ref[i], z=1.0)
        end_point = carla.Location(x=x_ref[i + 1], y=y_ref[i + 1], z=1.0)
        world.debug.draw_line(start_point, end_point, thickness=0.1, color=carla.Color(255, 0, 0), life_time=dt * 2)

def get_straight_trajectory(x_init, y_init, distance=3000, total_points=1000):
    """
    Создает прямую траекторию, направленную на север (вдоль оси Y), начиная с заданной точки (x_init, y_init).

    :param x_init: Начальная координата X
    :param y_init: Начальная координата Y
    :param distance: Дистанция по Y, которую нужно пройти
    :param total_points: Количество точек для расчета траектории
    :return: Траектория в виде списков x и y, и список референсных скоростей v_ref
    """
    x_traj = np.linspace(x_init + 5, x_init + distance, total_points)
    y_traj = np.full(total_points, y_init + 3)

    # Примерная скорость для каждого шага
    v_ref = [V_REF for _ in range(total_points)]

    return x_traj, y_traj, v_ref

# Функция для расчета поперечного отклонения от траектории
def calculate_lateral_deviation(x, y, x_ref1, y_ref1, x_ref2, y_ref2):
    num = (y_ref2 - y_ref1) * x - (x_ref2 - x_ref1) * y + x_ref2 * y_ref1 - y_ref2 * x_ref1
    denom = ca.sqrt((y_ref2 - y_ref1) ** 2 + (x_ref2 - x_ref1) ** 2)
    if denom > 0.01:
        return num / denom
    else:
        return 0
