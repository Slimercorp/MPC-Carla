import numpy as np

from src.config import N, V_REF, A

def update_reference_point(x0, y0, current_idx, x_traj, y_traj, min_distance=5.0):
    distance_to_current_point = np.sqrt((x_traj[current_idx] - x0) ** 2 + (y_traj[current_idx] - y0) ** 2)

    if distance_to_current_point < min_distance:
        current_idx = (current_idx + 1) % len(x_traj)

    return current_idx

def get_eight_trajectory(x_init, y_init, total_points=100):
    t = 2 * np.pi

    t_values = np.linspace(0, t, total_points)
    x_traj = x_init + A * np.sin(t_values)
    y_traj = y_init + A * np.sin(t_values) * np.cos(t_values)

    v_ref = [V_REF for _ in range(total_points)]

    cos_angle = np.cos(-np.pi / 4)
    sin_angle = np.sin(-np.pi / 4)

    x_traj_rotated = x_init + cos_angle * (x_traj - x_init) - sin_angle * (y_traj - y_init)
    y_traj_rotated = y_init + sin_angle * (x_traj - x_init) + cos_angle * (y_traj - y_init)

    theta_ref = []
    for i in range(total_points - 1):
        dx = x_traj_rotated[i + 1] - x_traj_rotated[i]
        dy = y_traj_rotated[i + 1] - y_traj_rotated[i]
        theta = np.arctan2(dy, dx)
        theta_ref.append(theta)

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

def get_straight_trajectory(x_init, y_init, distance=3000, total_points=1000):
    x_traj = np.linspace(x_init + 5, x_init + distance, total_points)
    y_traj = np.full(total_points, y_init + 3)

    v_ref = [V_REF for _ in range(total_points)]

    return x_traj, y_traj, v_ref

def calculate_lateral_deviation(x, y, x_ref1, y_ref1, x_ref2, y_ref2):
    num = (y_ref2 - y_ref1) * x - (x_ref2 - x_ref1) * y + x_ref2 * y_ref1 - y_ref2 * x_ref1
    denom = np.sqrt((y_ref2 - y_ref1) ** 2 + (x_ref2 - x_ref1) ** 2)
    if denom > 0.01:
        return num / denom
    else:
        return 0
