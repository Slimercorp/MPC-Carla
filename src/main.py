import carla
import time
import numpy as np
from src.mpc_controller import mpc_controller, N, dt
MAX_ACCELERATION_M_S_2 = 10
MAX_BRAKING_M_S_2 = -10
MAX_WHEEL_ANGLE_RAD = 30 / 180 * np.pi

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
spawn_rotation = carla.Rotation(pitch=0, yaw=50, roll=0)  # Указываем ориентацию (yaw - это угол поворота по вертикали)

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

# Параметры для восьмерки
a = 25  # Масштаб восьмерки
T = 2 * np.pi  # Период для траектории
total_points = 100  # Количество точек для расчета полной траектории

# Генерация полной траектории "восьмерки"
t_values = np.linspace(0, T, total_points)
x_traj = x_init + a * np.sin(t_values)
y_traj = y_init + a * np.sin(t_values) * np.cos(t_values)
V_ref = 2
current_idx = 0
acceleration_m_s_2 = 0
wheel_angle_rad = 0
def update_reference_point(x0, y0, current_idx, x_traj, y_traj, min_distance=1.0):
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

try:
    while True:
        # Визуализация полной референсной траектории в CARLA
        for i in range(total_points - 1):
            start_point = carla.Location(x=x_traj[i], y=y_traj[i], z=0.2)
            end_point = carla.Location(x=x_traj[i + 1], y=y_traj[i + 1], z=0.2)
            world.debug.draw_line(start_point, end_point, thickness=0.1, color=carla.Color(0, 255, 0), life_time=dt * 20)

        # Получаем текущее положение автомобиля
        transform = ego_vehicle.get_transform()
        x0 = transform.location.x
        y0 = transform.location.y
        theta0 = np.deg2rad(transform.rotation.yaw)
        v0 = np.sqrt(ego_vehicle.get_velocity().x**2 + ego_vehicle.get_velocity().y**2)

        # Формируем референсную траекторию длиной N точек, начиная с current_idx
        if current_idx + N < total_points:
            x_ref = x_traj[current_idx:current_idx + N]
            y_ref = y_traj[current_idx:current_idx + N]
        else:
            # Если достигнут конец массива, циклически возвращаемся к началу
            x_ref = np.concatenate((x_traj[current_idx:], x_traj[:N - (total_points - current_idx)]))
            y_ref = np.concatenate((y_traj[current_idx:], y_traj[:N - (total_points - current_idx)]))

        print(f"x = {x0:.2f}, y = {y0:.2f}, v = {v0:.2f} theta = {theta0 * 180 / np.pi:.2f} x_ref = {x_ref[0]:.2f}, y_ref = {y_ref[0]:.2f}")

        # Визуализация текущей референсной траектории (следующей N точек)
        for i in range(N - 1):
            start_point = carla.Location(x=x_ref[i], y=y_ref[i], z=1.0)
            end_point = carla.Location(x=x_ref[i + 1], y=y_ref[i + 1], z=1.0)
            world.debug.draw_line(start_point, end_point, thickness=0.1, color=carla.Color(255, 0, 0), life_time=dt*20)  # Увеличиваем время жизни

        # Передаем референсную траекторию в контроллер
        acceleration_m_s_2, wheel_angle_rad = mpc_controller(x0, y0, theta0, v0, acceleration_m_s_2, wheel_angle_rad, x_ref, y_ref, V_ref)
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
        print(f"throttle = {throttle:.2f}, brake = {brake:.2f}  steer = {steer:.2f} \n")
        ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

        # Обновляем текущую точку на траектории
        current_idx = update_reference_point(x0, y0, current_idx, x_traj, y_traj)
        time.sleep(dt)

finally:
    ego_vehicle.destroy()
    for vehicle in world.get_actors().filter('*vehicle*'):
        vehicle.destroy()
