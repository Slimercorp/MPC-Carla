import numpy as np

MAX_ACCELERATION_M_S_2 = 10
MAX_BRAKING_M_S_2 = -10
MAX_WHEEL_ANGLE_RAD = 30 / 180 * np.pi

N = 7  # Горизонт предсказания (количество шагов)
dt = 0.1
MAX_CONTROL_WHEEL_ANGLE_RAD = 30 / 180 * np.pi
MAX_CONTROL_ACCELERATION_M_S_2 = 10
MAX_CONTROL_BRAKING_M_S_2 = -10
PATH_TOLERANCE_M = 0.5
V_REF = 4

FINE_X_COEF = 15  # Увеличенный штраф за отклонение по X
FINE_Y_COEF = 15  # Увеличенный штраф за отклонение по Y
FINE_STEER_COEF = 5
FINE_ACC_COEF = 1
FINE_STEER_DOT_COEF = 100  # Штраф за резкие изменения угла поворота
FINE_ACC_DOT_COEF = 1  # Штраф за резкие изменения ускорения
FINE_V_COEF = 10  # Увеличенный штраф за отклонение скорости
FINE_THETA_COEF = 0
FINE_LATERAL_COEF = 0