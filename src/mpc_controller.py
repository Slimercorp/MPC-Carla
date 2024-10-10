import casadi as ca

from src.help_functions import evaluate_first_step_cost, calculate_lateral_deviation
from src.config import MAX_CONTROL_WHEEL_ANGLE_RAD, MAX_CONTROL_ACCELERATION_M_S_2, MAX_CONTROL_BRAKING_M_S_2, \
    PATH_TOLERANCE_M, FINE_X_COEF, FINE_Y_COEF, FINE_V_COEF, FINE_STEER_COEF, FINE_ACC_COEF, FINE_STEER_DOT_COEF, \
    FINE_ACC_DOT_COEF, N, dt, FINE_THETA_COEF, FINE_LATERAL_COEF
from src.vehicle_model import vehicle_model

# Основная функция MPC контроллера
def mpc_controller(x0, y0, theta0, v0, acc0, steer0, x_ref, y_ref, v_ref, theta_ref):
    # Определение переменных оптимизации
    opti = ca.Opti()  # CasADi optimization environment

    X = opti.variable(4, N + 1)  # Состояние системы: (x, y, theta, v)
    U = opti.variable(2, N)  # Управление: (delta, a)

    # Начальное состояние
    opti.subject_to(X[:, 0] == ca.vertcat(x0, y0, theta0, v0))

    # Ограничения на управление
    for k in range(N):
        opti.subject_to(X[:, k + 1] == vehicle_model(X[:, k], U[:, k], dt))
        opti.subject_to(U[0, k] <= MAX_CONTROL_WHEEL_ANGLE_RAD)
        opti.subject_to(U[0, k] >= -MAX_CONTROL_WHEEL_ANGLE_RAD)
        opti.subject_to(U[1, k] <= MAX_CONTROL_ACCELERATION_M_S_2)
        opti.subject_to(U[1, k] >= MAX_CONTROL_BRAKING_M_S_2)

    # Целевая функция - минимизация отклонения от референсной траектории
    cost = 0
    for k in range(N):
        fine_x = ca.if_else(ca.fabs(X[0, k] - x_ref[k]) > PATH_TOLERANCE_M,
                           FINE_X_COEF * (X[0, k] - x_ref[k]) ** 2, 0)
        fine_y = ca.if_else(ca.fabs(X[1, k] - y_ref[k]) > PATH_TOLERANCE_M,
                           FINE_Y_COEF * (X[1, k] - y_ref[k]) ** 2, 0)
        fine_v = FINE_V_COEF * (X[3, k] - v_ref[k]) ** 2
        fine_steer = FINE_STEER_COEF * U[0, k] ** 2
        fine_acc = FINE_ACC_COEF * U[1, k] ** 2

        fine_theta = FINE_THETA_COEF * (X[2, k] - theta_ref[k]) ** 2

        if k < N - 1:
            lateral_deviation = calculate_lateral_deviation(X[0, k], X[1, k], x_ref[k], y_ref[k], x_ref[k + 1], y_ref[k + 1])
        else:
            lateral_deviation = 0

        fine_lateral = FINE_LATERAL_COEF * lateral_deviation ** 2

        # Добавление штрафов за резкие изменения управления
        fine_steer_dot = 0
        fine_acc_dot = 0
        if k > 0:
            fine_steer_dot += FINE_STEER_DOT_COEF * (U[0, k] - U[0, k - 1]) ** 2
            fine_acc_dot += FINE_ACC_DOT_COEF * (U[1, k] - U[1, k - 1]) ** 2

        # Суммарная стоимость
        cost += fine_x + fine_y + fine_v + fine_steer + fine_acc + fine_steer_dot + fine_acc_dot + fine_theta + fine_lateral

    opti.minimize(cost)

    # Решение оптимизационной задачи с ограничением по времени
    opts = {
        'ipopt.print_level': 0,
        'print_time': 0,
        'ipopt.max_cpu_time': dt  # Ограничение времени решения в секундах (dt)
    }
    opti.solver('ipopt', opts)

    sol = opti.solve()
    optimized_cost = sol.value(cost)

    # Получение первого управляющего действия
    wheel_angle_rad = sol.value(U[0, 0])
    acceleration_m_s_2 = sol.value(U[1, 0])

    # В конце контроллера вызываем эту функцию для первого шага
    first_step_cost = evaluate_first_step_cost(x0, y0, v0, theta0, acceleration_m_s_2, wheel_angle_rad, acc0, steer0, x_ref, y_ref, v_ref, theta_ref, PATH_TOLERANCE_M)

    print(f"First step cost breakdown:")
    print(f"  Deviation X cost: {first_step_cost['fine_x']:.4f}")
    print(f"  Deviation Y cost: {first_step_cost['fine_y']:.4f}")
    print(f"  Deviation V cost: {first_step_cost['fine_v']:.4f}")
    print(f"  Steering control cost: {first_step_cost['fine_steer']:.4f}")
    print(f"  Acceleration control cost: {first_step_cost['fine_acc']:.4f}")
    print(f"  Steering control dot cost: {first_step_cost['fine_steer_dot']:.4f}")
    print(f"  Acceleration control dot cost: {first_step_cost['fine_acc_dot']:.4f}")
    print(f"  Theta cost: {first_step_cost['fine_theta']:.4f}")
    print(f"  Lateral deviation cost: {first_step_cost['fine_lateral']:.4f}")
    print(f"  Total cost for first step: {first_step_cost['total_cost_first_step']:.4f}")
    print(f"Optimized total cost (final cost after optimization): {optimized_cost:.4f}")

    return acceleration_m_s_2, wheel_angle_rad