import casadi as ca

# Параметры модели автомобиля
L = 2.875  # Длина колесной базы автомобиля (метры)
N = 20  # Горизонт предсказания (количество шагов)
dt = 0.01
MAX_CONTROL_WHEEL_ANGLE_RAD = 30 / 180 * ca.pi
MAX_CONTROL_ACCELERATION_M_S_2 = 10
MAX_CONTROL_BRAKING_M_S_2 = -10
MIN_SPEED_M_S = 1
PATH_TOLERANCE_M = 0.1

FINE_X_COEF = 1
FINE_Y_COEF = 1
FINE_STEER_COEF = 0
FINE_ACC_COEF = 0
FINE_STEER_DOT_COEF = 10
FINE_ACC_DOT_COEF = 1
FINE_V_COEF = 10

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

    fine_v = FINE_V_COEF * (v - v_ref) ** 2

    fine_steer = FINE_STEER_COEF * steer ** 2
    fine_acc = FINE_ACC_COEF * acc ** 2

    fine_steer_dot = FINE_STEER_DOT_COEF * (steer - steer0) ** 2
    fine_acc_dot = FINE_ACC_DOT_COEF * (acc - acc0) ** 2

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

# Функция динамики автомобиля
def vehicle_model(x, u, dt):
    x_pos = x[0]
    y_pos = x[1]
    theta = x[2]
    v = x[3]

    delta = u[0]  # Steering angle (управление рулем)
    a = u[1]  # Acceleration (управление дросселем)

    x_pos_next = x_pos + v * ca.cos(theta) * dt
    y_pos_next = y_pos + v * ca.sin(theta) * dt
    theta_next = theta + v / L * ca.tan(delta) * dt
    v_next = v + a * dt

    return ca.vertcat(x_pos_next, y_pos_next, theta_next, v_next)


# Основная функция MPC контроллера
def mpc_controller(x0, y0, theta0, v0, acc0, steer0, x_ref, y_ref, v_ref):
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
        fine_v = FINE_V_COEF * (X[3, k] - v_ref) ** 2
        fine_steer = FINE_STEER_COEF * U[0, k] ** 2
        fine_acc = FINE_ACC_COEF * U[1, k] ** 2
        fine_steer_dot = 0
        fine_acc_dot = 0
        if k > 0:
            fine_steer_dot += FINE_STEER_DOT_COEF * (U[0, k] - U[0, k - 1]) ** 2
            fine_acc_dot += FINE_ACC_DOT_COEF * (U[1, k] - U[1, k - 1]) ** 2

        cost += fine_x + fine_y + fine_v + fine_steer + fine_acc + fine_steer_dot + fine_acc_dot

    opti.minimize(cost)

    # Решение оптимизационной задачи
    opts = {'ipopt.print_level': 0, 'print_time': 0}
    opti.solver('ipopt', opts)

    sol = opti.solve()
    optimized_cost = sol.value(cost)

    # Получение первого управляющего действия
    wheel_angle_rad = sol.value(U[0, 0])
    acceleration_m_s_2 = sol.value(U[1, 0])

    # В конце контроллера вызываем эту функцию для первого шага
    first_step_cost = evaluate_first_step_cost(x0, y0, v0, acceleration_m_s_2, wheel_angle_rad, acc0, steer0, x_ref, y_ref, v_ref, PATH_TOLERANCE_M)

    print(f"First step cost breakdown:")
    print(f"  Deviation X cost: {first_step_cost['fine_x']:.4f}")
    print(f"  Deviation Y cost: {first_step_cost['fine_y']:.4f}")
    print(f"  Deviation V cost: {first_step_cost['fine_v']:.4f}")
    print(f"  Steering control cost: {first_step_cost['fine_steer']:.4f}")
    print(f"  Acceleration control cost: {first_step_cost['fine_acc']:.4f}")
    print(f"  Steering control dot cost: {first_step_cost['fine_steer_dot']:.4f}")
    print(f"  Acceleration control dot cost: {first_step_cost['fine_acc_dot']:.4f}")
    print(f"  Total cost for first step: {first_step_cost['total_cost_first_step']:.4f}")
    print(f"Optimized total cost (final cost after optimization): {optimized_cost:.4f}")

    return acceleration_m_s_2, wheel_angle_rad