import numpy as np
from src.config import FINE_X_COEF, FINE_Y_COEF, FINE_V_COEF, FINE_STEER_COEF, FINE_ACC_COEF, \
    FINE_STEER_DOT_COEF, FINE_ACC_DOT_COEF, N, dt, V_REF


def evaluate_first_step_cost(x, y, v, acc, steer, acc0, steer0, x_ref, y_ref, v_ref, PATH_TOLERANCE_M):
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

    fine_v = FINE_V_COEF * (v - v_ref[0]) ** 2

    fine_steer = FINE_STEER_COEF * steer ** 2
    fine_acc = FINE_ACC_COEF * acc ** 2

    fine_steer_dot = FINE_STEER_DOT_COEF * ((steer - steer0) / dt) ** 2
    fine_acc_dot = FINE_ACC_DOT_COEF * ((acc - acc0) / dt) ** 2

    # Суммарный вклад для первого шага
    total_cost_first_step = fine_x + fine_y + fine_v + fine_steer + fine_acc + fine_steer_dot + fine_acc_dot

    # Возвращаем результат по частям для анализа
    return {
        "fine_x": fine_x,
        "fine_y": fine_y,
        "fine_v": fine_v,
        "fine_steer": fine_steer,
        "fine_acc": fine_acc,
        "fine_steer_dot": fine_steer_dot,
        "fine_acc_dot": fine_acc_dot,
        "total_cost_first_step": total_cost_first_step
    }

def update_reference_point(x0, y0, current_idx, x_traj, y_traj, min_distance=2.0):
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
        world.debug.draw_line(start_point, end_point, thickness=0.1, color=carla.Color(0, 255, 0), life_time=dt * 20)


def get_eight_trajectory(x_init, y_init):
    # Параметры для восьмерки
    a = 25  # Масштаб восьмерки
    T = 2 * np.pi  # Период для траектории
    total_points = 100  # Количество точек для расчета полной траектории

    # Генерация полной траектории "восьмерки"
    t_values = np.linspace(0, T, total_points)
    x_traj = x_init + a * np.sin(t_values)
    y_traj = y_init + a * np.sin(t_values) * np.cos(t_values)
    v_ref = [V_REF + 0.1 * np.sin(i * T / total_points) for i in range(total_points)]  # Примерная скорость для каждого шага

    return x_traj, y_traj, v_ref


def get_ref_trajectory(x_traj, y_traj, current_idx):
    if current_idx + N < len(x_traj):
        x_ref = x_traj[current_idx:current_idx + N]
        y_ref = y_traj[current_idx:current_idx + N]
    else:
        x_ref = np.concatenate((x_traj[current_idx:], x_traj[:N - (len(x_traj) - current_idx)]))
        y_ref = np.concatenate((y_traj[current_idx:], y_traj[:N - (len(x_traj) - current_idx)]))
    return x_ref, y_ref

def draw_ref_trajectory(carla, world, x_ref, y_ref):
    for i in range(N - 1):
        start_point = carla.Location(x=x_ref[i], y=y_ref[i], z=1.0)
        end_point = carla.Location(x=x_ref[i + 1], y=y_ref[i + 1], z=1.0)
        world.debug.draw_line(start_point, end_point, thickness=0.1, color=carla.Color(255, 0, 0), life_time=dt * 20)
