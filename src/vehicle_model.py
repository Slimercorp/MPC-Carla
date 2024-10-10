import casadi as ca

from src.config import L

def vehicle_model(x, u, dt):
    x_pos = x[0]
    y_pos = x[1]
    theta = x[2]
    v = x[3]

    delta = u[0]
    a = u[1]

    x_pos_next = x_pos + v * ca.cos(theta) * dt
    y_pos_next = y_pos + v * ca.sin(theta) * dt
    theta_next = theta + v / L * ca.tan(delta) * dt
    v_next = v + a * dt

    return ca.vertcat(x_pos_next, y_pos_next, theta_next, v_next)


