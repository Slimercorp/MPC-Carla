import carla
import time
import numpy as np
import matplotlib.pyplot as plt

from src.config import MAX_BRAKING_M_S_2, MAX_ACCELERATION_M_S_2, MAX_WHEEL_ANGLE_RAD, dt, V_REF, N
from src.help_functions import update_reference_point, draw_full_trajectory, draw_ref_trajectory, get_eight_trajectory, \
    get_ref_trajectory, get_straight_trajectory
from src.mpc_controller import mpc_controller

# Переменные для хранения данных для графиков
x_data = []
y_data = []
v_data = []
theta_data = []
x_ref_data = []
y_ref_data = []
v_ref_data = []
theta_ref_data = []
steering_data = []
throttle_data = []
brake_data = []

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
client.load_world('Town02_Opt')
world = client.get_world()

blueprint_library = world.get_blueprint_library()
vehicle_blueprints = blueprint_library.filter('*vehicle*')
spawn_points = world.get_map().get_spawn_points()

# Уничтожаем все машины
for vehicle in world.get_actors().filter('*vehicle*'):
    vehicle.destroy()

# Спауним автомобиль
vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
spawn_location = carla.Location(x=210, y=362, z=0.1)  # Указываем точные координаты
spawn_rotation = carla.Rotation(pitch=0, yaw=0, roll=0)  # Указываем ориентацию (yaw - это угол поворота по вертикали)

# Создаем объект трансформации
spawn_transform = carla.Transform(location=spawn_location, rotation=spawn_rotation)

# Спавним автомобиль
ego_vehicle = world.spawn_actor(vehicle_bp, spawn_transform)

# Установка spectator
spectator = world.get_spectator()
transform = ego_vehicle.get_transform()
x_init = transform.location.x
y_init = transform.location.y
spectator_transform = ego_vehicle.get_transform()
spectator_transform.location += carla.Location(z=50)
spectator_transform.rotation.pitch = -90
spectator.set_transform(spectator_transform)

x_traj, y_traj, v_ref, theta_traj = get_eight_trajectory(x_init, y_init)
current_idx = 0
acceleration_m_s_2 = 0
wheel_angle_rad = 0

# Инициализация буфера управления на горизонте предсказания
control_buffer = {"acceleration": [0] * N, "steering": [0] * N}
buffer_index = 0  # Указатель на текущее управление из буфера

def update_control_buffer(acc_buffer, steer_buffer, new_acc, new_steer):
    """
    Обновляет буфер значений управления, сдвигая их и добавляя новые значения.
    """
    acc_buffer = acc_buffer[1:] + [new_acc]  # Сдвиг и добавление нового значения
    steer_buffer = steer_buffer[1:] + [new_steer]
    return acc_buffer, steer_buffer

try:
    while True:
        start_time = time.time()

        draw_full_trajectory(carla, world, x_traj, y_traj)

        # Получаем текущее положение автомобиля
        transform = ego_vehicle.get_transform()
        x0 = transform.location.x
        y0 = transform.location.y
        theta0 = np.deg2rad(transform.rotation.yaw)
        v0 = np.sqrt(ego_vehicle.get_velocity().x**2 + ego_vehicle.get_velocity().y**2)

        x_ref, y_ref, theta_ref = get_ref_trajectory(x_traj, y_traj, theta_traj, current_idx)
        print(f"x = {x0:.2f}, y = {y0:.2f}, v = {v0:.2f} theta = {theta0 * 180 / np.pi:.2f} x_ref = {x_ref[0]:.2f}, y_ref = {y_ref[0]:.2f}, v_ref = {v_ref[0]:.2f}, theta_ref = {theta_ref[0] * 180 / np.pi:.2f}")
        draw_ref_trajectory(carla, world, x_ref, y_ref)

        # Сохраняем данные для графиков
        x_data.append(x0)
        y_data.append(y0)
        v_data.append(v0)
        theta_data.append(theta0 * 180 / np.pi)
        x_ref_data.append(x_ref[0])
        y_ref_data.append(y_ref[0])
        v_ref_data.append(V_REF)
        theta_ref_data.append(theta_ref[0] * 180 / np.pi)

        try:
            # Передаем референсную траекторию в контроллер и рассчитываем управление
            acceleration_m_s_2, wheel_angle_rad = mpc_controller(x0, y0, theta0, v0, acceleration_m_s_2, wheel_angle_rad, x_ref, y_ref, v_ref, theta_ref)

            # Успешное решение, обновляем буфер
            control_buffer["acceleration"], control_buffer["steering"] = update_control_buffer(
                control_buffer["acceleration"], control_buffer["steering"], acceleration_m_s_2, wheel_angle_rad
            )
            buffer_index = 0  # Сбрасываем индекс буфера на 0, т.к. решение было успешно рассчитано

        except Exception as e:
            print(f"Ошибка или задержка при расчете MPC. Используем предыдущее управление.")
            # Если возникает ошибка, используем следующее значение из буфера
            if buffer_index < N:
                acceleration_m_s_2 = control_buffer["acceleration"][buffer_index]
                wheel_angle_rad = control_buffer["steering"][buffer_index]
                buffer_index += 1  # Увеличиваем индекс для следующего сбоя
            else:
                raise Exception("Буфер управления исчерпан, все ранее рассчитанные значения использованы.")

        print(f"acceleration_m_s_2 = {acceleration_m_s_2:.2f}, wheel_angle_rad = {wheel_angle_rad:.2f}")

        if acceleration_m_s_2 == 0:
            throttle = 0
            brake = 0
        elif acceleration_m_s_2 < 0:
            throttle = 0
            brake = acceleration_m_s_2 / MAX_BRAKING_M_S_2
        else:
            throttle = acceleration_m_s_2 / MAX_ACCELERATION_M_S_2
            brake = 0
        steer = wheel_angle_rad / MAX_WHEEL_ANGLE_RAD
        print(f"throttle = {throttle:.2f}, brake = {brake:.2f}  steer = {steer:.2f}")

        # Сохраняем управление для графиков
        steering_data.append(steer)
        throttle_data.append(throttle)
        brake_data.append(brake)
        ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

        current_idx = update_reference_point(x0, y0, current_idx, x_traj, y_traj)
        print(f"current_idx = {current_idx}\n")

        end_time = time.time()
        mpc_calculation_time = end_time - start_time
        print(f"Время расчета управления MPC: {mpc_calculation_time:.6f} секунд")
        if mpc_calculation_time > dt:
            print(f"Внимание! Контроллер MPC не успевает рассчитать управление за {dt} секунд.")

        time.sleep(max(dt - mpc_calculation_time, 0))

finally:
    ego_vehicle.destroy()
    for vehicle in world.get_actors().filter('*vehicle*'):
        vehicle.destroy()

    # Находим минимальный размер среди всех массивов данных
    min_length = min(len(x_data), len(y_data), len(v_data), len(x_ref_data), len(y_ref_data), len(v_ref_data), len(steering_data), len(throttle_data), len(brake_data))

    # Обрезаем все массивы до минимального размера
    x_data = x_data[:min_length]
    y_data = y_data[:min_length]
    v_data = v_data[:min_length]
    x_ref_data = x_ref_data[:min_length]
    y_ref_data = y_ref_data[:min_length]
    v_ref_data = v_ref_data[:min_length]
    steering_data = steering_data[:min_length]
    throttle_data = throttle_data[:min_length]
    brake_data = brake_data[:min_length]

    # Соответственно обрезаем time_axis
    time_axis = np.linspace(0, min_length * dt, min_length)

    # График для x и x_ref
    plt.figure()
    plt.plot(time_axis, x_data, label='x (car)')
    plt.plot(time_axis, x_ref_data, label='x_ref (reference)', linestyle='--')
    plt.scatter(time_axis, x_data, color='blue', s=10)  # Добавляем точки измерений для x
    plt.scatter(time_axis, x_ref_data, color='red', s=10)  # Добавляем точки измерений для x_ref
    plt.xlabel('Time (s)')
    plt.ylabel('x position (m)')
    plt.legend()
    plt.title('X Position vs Time')

    # График для y и y_ref
    plt.figure()
    plt.plot(time_axis, y_data, label='y (car)')
    plt.plot(time_axis, y_ref_data, label='y_ref (reference)', linestyle='--')
    plt.scatter(time_axis, y_data, color='blue', s=10)  # Добавляем точки измерений для y
    plt.scatter(time_axis, y_ref_data, color='red', s=10)  # Добавляем точки измерений для y_ref
    plt.xlabel('Time (s)')
    plt.ylabel('y position (m)')
    plt.legend()
    plt.title('Y Position vs Time')

    # График для скорости и v_ref
    plt.figure()
    plt.plot(time_axis, v_data, label='v (car)')
    plt.plot(time_axis, v_ref_data, label='v_ref (reference)', linestyle='--')
    plt.scatter(time_axis, v_data, color='blue', s=10)  # Добавляем точки измерений для v
    plt.scatter(time_axis, v_ref_data, color='red', s=10)  # Добавляем точки измерений для v_ref
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.legend()
    plt.title('Speed vs Time')

    # График для theta и theta_ref
    plt.figure()
    plt.plot(time_axis, theta_data, label='theta (car)')
    plt.plot(time_axis, theta_ref_data, label='theta_ref (reference)', linestyle='--')
    plt.scatter(time_axis, theta_data, color='blue', s=10)  # Добавляем точки измерений для theta_data
    plt.scatter(time_axis, theta_ref_data, color='red', s=10)  # Добавляем точки измерений для theta_ref_data
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (deg)')
    plt.legend()
    plt.title('Theta vs Time')

    # График для управления рулем
    plt.figure()
    plt.plot(time_axis, steering_data, label='Steering Control')
    plt.scatter(time_axis, steering_data, color='blue', s=10)  # Добавляем точки измерений для управления рулем
    plt.xlabel('Time (s)')
    plt.ylabel('Steering angle (normalized)')
    plt.legend()
    plt.title('Steering Control vs Time')

    # График для управления педалью газа
    plt.figure()
    plt.plot(time_axis, throttle_data, label='Throttle Control')
    plt.plot(time_axis, brake_data, label='Brake Control', linestyle='--')
    plt.scatter(time_axis, throttle_data, color='blue', s=10)  # Добавляем точки измерений для газа
    plt.scatter(time_axis, brake_data, color='red', s=10)  # Добавляем точки измерений для тормоза
    plt.xlabel('Time (s)')
    plt.ylabel('Throttle/Brake (normalized)')
    plt.legend()
    plt.title('Throttle Control and Brake Control vs Time')

    plt.show()  # Показ всех графиков
