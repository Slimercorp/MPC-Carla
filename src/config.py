import numpy as np

X_INIT_M = 210 # Start X coordinate for vehicle
Y_INIT_M = 362 # Start Y coordinate for vehicle
A = 25 # Size of 8-trajectory
LAPS = 1 #Laps of simulation

MAX_ACCELERATION_M_S_2 = 10 # Vehicle characteristics (from CARLA simulator, see get_physics_control())
MAX_BRAKING_M_S_2 = -4.1 # Vehicle characteristics (from CARLA simulator, see get_physics_control())
MAX_WHEEL_ANGLE_RAD = 70 / 180 * np.pi # Vehicle characteristics (from CARLA simulator, see get_physics_control())
L = 2.875  #The wheelbase length of the vehicle (meters)

### MPC settings
N = 5 # Horizon of planning for MPC controller
dt = 0.1 #Sample time of control
MAX_CONTROL_WHEEL_ANGLE_RAD = 70 / 180 * np.pi # Maximum value which can use MPC controller in terms of steering
MAX_CONTROL_ACCELERATION_M_S_2 = 10 # Maximum value which can use MPC controller in terms of acceleration
MAX_CONTROL_BRAKING_M_S_2 = -4.1 # Maximum value which can use MPC controller in terms of braking
PATH_TOLERANCE_M = 0.5 # Acceptance radius of each waypoint
V_REF = 5 #Speed of vehicle

FINE_X_COEF = 10 #Fine for deviation from reference X coordinate
FINE_Y_COEF = 10 #Fine for deviation from reference Y coordinate
FINE_STEER_COEF = 0 #Fine for use high value of steering
FINE_ACC_COEF = 0 #Fine for use high values of acceleration
FINE_STEER_DOT_COEF = 100 #Fine for rapid steering
FINE_ACC_DOT_COEF = 1 #Fine for rapid changing of acceleration
FINE_V_COEF = 20 #Fine for deviation from reference speed
FINE_THETA_COEF = 0 #Fine for deviation from reference theta
FINE_LATERAL_COEF = 40 #Fine for deviation from path line