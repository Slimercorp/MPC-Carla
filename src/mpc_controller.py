import casadi as ca

# Параметры модели автомобиля
L = 2.5  # Длина колесной базы автомобиля (метры)
N = 20  # Горизонт предсказания (количество шагов)
dt = 0.1
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
def mpc_controller(x0, y0, theta0, v0, x_ref, y_ref):
    # Определение переменных оптимизации
    opti = ca.Opti()  # CasADi optimization environment

    X = opti.variable(4, N + 1)  # Состояние системы: (x, y, theta, v)
    U = opti.variable(2, N)  # Управление: (delta, a)

    # Начальное состояние
    opti.subject_to(X[:, 0] == ca.vertcat(x0, y0, theta0, v0))

    # Ограничения на управление
    delta_max = ca.pi / 4  # Максимальный угол поворота
    a_max = 2.0  # Максимальное ускорение

    for k in range(N):
        opti.subject_to(X[:, k + 1] == vehicle_model(X[:, k], U[:, k], dt))
        opti.subject_to(U[0, k] <= delta_max)
        opti.subject_to(U[0, k] >= -delta_max)
        opti.subject_to(U[1, k] <= a_max)
        opti.subject_to(U[1, k] >= -a_max)

    # Целевая функция - минимизация отклонения от референсной траектории
    cost = 0
    for k in range(N):
        cost += (X[0, k] - x_ref[k]) ** 2 + (X[1, k] - y_ref[k]) ** 2  # Отклонение по x и y
        cost += 0.1 * U[0, k] ** 2 + 0.1 * U[1, k] ** 2  # Штраф на управление

    opti.minimize(cost)

    # Решение оптимизационной задачи
    opts = {'ipopt.print_level': 0, 'print_time': 0}
    opti.solver('ipopt', opts)

    sol = opti.solve()

    # Получение первого управляющего действия
    throttle = sol.value(U[1, 0])
    steer = sol.value(U[0, 0])

    return throttle, steer