import casadi as ca

L = 2.875  # Длина колесной базы автомобиля (метры)
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


