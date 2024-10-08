import carla
import time
import numpy as np
import random
from src.mpc_controller import mpc_controller, N, dt

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
spawn_location = carla.Location(x=180, y=362, z=6)  # Указываем точные координаты
spawn_rotation = carla.Rotation(pitch=0, yaw=180, roll=0)  # Указываем ориентацию (yaw - это угол поворота по вертикали)

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
spectator_transform.location += carla.Location(x=-6, z=3)
spectator.set_transform(spectator_transform)

# Параметры восьмерки
a = 50  # Масштаб восьмерки
T = 2 * np.pi  # Период, чтобы описать полный цикл восьмерки

try:
    while True:
        # Получение текущего состояния автомобиля
        transform = ego_vehicle.get_transform()
        velocity = ego_vehicle.get_velocity()

        x0 = transform.location.x
        y0 = transform.location.y
        theta0 = np.deg2rad(transform.rotation.yaw)
        v0 = np.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

        # Генерация обновляемой референсной траектории - скользящая восьмерка
        t_values = np.linspace(0, T, N + 1)  # Параметр t для генерации точек
        x_ref = x0 + a * np.sin(t_values)  # Параметрическая кривая по X относительно текущего положения
        y_ref = y0 + a * np.sin(t_values) * np.cos(t_values)  # Параметрическая кривая по Y

        # Визуализация референсной траектории в CARLA
        for i in range(N):
            start_point = carla.Location(x=x_ref[i], y=y_ref[i], z=transform.location.z + 0.1)
            end_point = carla.Location(x=x_ref[i + 1], y=y_ref[i + 1], z=transform.location.z + 0.1)
            world.debug.draw_line(start_point, end_point, thickness=0.1, color=carla.Color(0, 255, 0), life_time=0.1)

        print(f"x = {x0}, y = {y0}, x_ref = {x_ref[0]}, y_ref = {y_ref[0]}")

        # Управляющие действия MPC
        throttle, steer = mpc_controller(x0, y0, theta0, v0, x_ref, y_ref)
        print(f"throttle = {throttle}, steer = {steer}")

        # Применение управляющих действий
        ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer))

        # Задержка на шаг симуляции
        time.sleep(dt)

finally:
    ego_vehicle.destroy()
    for vehicle in world.get_actors().filter('*vehicle*'):
        vehicle.destroy()
