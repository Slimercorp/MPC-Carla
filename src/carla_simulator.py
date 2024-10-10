import carla
import numpy as np

from src.config import MAX_BRAKING_M_S_2, MAX_WHEEL_ANGLE_RAD, MAX_ACCELERATION_M_S_2


class CarlaSimulator:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.ego_vehicle = None
        for vehicle in self.world.get_actors().filter('*vehicle*'):
            vehicle.destroy()

    def load_world(self, map_name):
        self.client.load_world(map_name)

    def spawn_vehicle(self, vehicle_name, x=0, y=0, z=0, pitch=0, yaw=0, roll=0):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(vehicle_name)[0]
        spawn_location = carla.Location(x, y, z)
        spawn_rotation = carla.Rotation(pitch, yaw,roll)
        spawn_transform = carla.Transform(location=spawn_location, rotation=spawn_rotation)
        self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)

    def set_spectator(self, x=0, y=0, z=0, pitch=0, yaw=0, roll=0):
        spectator = self.world.get_spectator()
        location = carla.Location(x=x, y=y, z=z)
        rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
        spectator_transform = carla.Transform(location, rotation)
        spectator.set_transform(spectator_transform)

    def clean(self):
        for vehicle in self.world.get_actors().filter('*vehicle*'):
            vehicle.destroy()

    def draw_trajectory(self, x_traj, y_traj, height = 0, thickness = 0.1, red = 0, green = 0, blue = 0, life_time = 0.1):
        for i in range(len(x_traj) - 1):
            start_point = carla.Location(x=x_traj[i], y=y_traj[i], z=height)
            end_point = carla.Location(x=x_traj[i + 1], y=y_traj[i + 1], z=height)
            self.world.debug.draw_line(start_point, end_point, thickness=thickness, color=carla.Color(red, green, blue), life_time=life_time)

    def get_main_vehicle_state(self):
        transform = self.ego_vehicle.get_transform()
        x = transform.location.x
        y = transform.location.y
        theta = np.deg2rad(transform.rotation.yaw)
        v = np.sqrt(self.ego_vehicle.get_velocity().x**2 + self.ego_vehicle.get_velocity().y**2)
        return x, y, theta, v

    def apply_control(self, steer, throttle, brake):
        self.ego_vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

    @staticmethod
    def process_control_inputs(wheel_angle_rad, acceleration_m_s_2):
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
        return throttle, brake, steer