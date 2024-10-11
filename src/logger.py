import numpy as np
from matplotlib import pyplot as plt

from src.config import dt


class Logger:
    def __init__(self):
        self.x_data = []
        self.y_data = []
        self.v_data = []
        self.theta_data = []
        self.x_ref_data = []
        self.y_ref_data = []
        self.v_ref_data = []
        self.theta_ref_data = []
        self.steering_data = []
        self.throttle_data = []
        self.brake_data = []

    def log_controller_input(self, x, y, v, theta, x_ref, y_ref, v_ref, theta_ref):
        self.x_data.append(x)
        self.y_data.append(y)
        self.v_data.append(v)
        self.theta_data.append(theta * 180 / np.pi)
        self.x_ref_data.append(x_ref)
        self.y_ref_data.append(y_ref)
        self.v_ref_data.append(v_ref)
        self.theta_ref_data.append(theta_ref * 180 / np.pi)

    def log_controller_output(self, steering, throttle, brake):
        self.steering_data.append(steering)
        self.throttle_data.append(throttle)
        self.brake_data.append(brake)

    def show_plots(self):
        min_length = min(len(self.x_data), len(self.y_data), len(self.v_data), len(self.theta_data), len(self.x_ref_data), len(self.y_ref_data),
                         len(self.v_ref_data), len(self.theta_ref_data), len(self.steering_data), len(self.throttle_data), len(self.brake_data))

        self.x_data = self.x_data[:min_length]
        self.y_data = self.y_data[:min_length]
        self.v_data = self.v_data[:min_length]
        self.theta_data = self.theta_data[:min_length]
        self.x_ref_data = self.x_ref_data[:min_length]
        self.y_ref_data = self.y_ref_data[:min_length]
        self.v_ref_data = self.v_ref_data[:min_length]
        self.theta_ref_data = self.theta_ref_data[:min_length]
        self.steering_data = self.steering_data[:min_length]
        self.throttle_data = self.throttle_data[:min_length]
        self.brake_data = self.brake_data[:min_length]

        time_axis = np.linspace(0, min_length * dt, min_length)

        plt.figure()
        plt.plot(time_axis, self.x_data, label='x (car)')
        plt.plot(time_axis, self.x_ref_data, label='x_ref (reference)', linestyle='--')
        plt.scatter(time_axis, self.x_data, color='blue', s=10)
        plt.scatter(time_axis, self.x_ref_data, color='red', s=10)
        plt.xlabel('Time (s)')
        plt.ylabel('x position (m)')
        plt.legend()
        plt.title('X Position vs Time')

        plt.figure()
        plt.plot(time_axis, self.y_data, label='y (car)')
        plt.plot(time_axis, self.y_ref_data, label='y_ref (reference)', linestyle='--')
        plt.scatter(time_axis, self.y_data, color='blue', s=10)
        plt.scatter(time_axis, self.y_ref_data, color='red', s=10)
        plt.xlabel('Time (s)')
        plt.ylabel('y position (m)')
        plt.legend()
        plt.title('Y Position vs Time')

        plt.figure()
        plt.plot(time_axis, self.v_data, label='v (car)')
        plt.plot(time_axis, self.v_ref_data, label='v_ref (reference)', linestyle='--')
        plt.scatter(time_axis, self.v_data, color='blue', s=10)
        plt.scatter(time_axis, self.v_ref_data, color='red', s=10)
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.legend()
        plt.title('Speed vs Time')

        plt.figure()
        plt.plot(time_axis, self.theta_data, label='theta (car)')
        plt.plot(time_axis, self.theta_ref_data, label='theta_ref (reference)', linestyle='--')
        plt.scatter(time_axis, self.theta_data, color='blue', s=10)
        plt.scatter(time_axis, self.theta_ref_data, color='red', s=10)
        plt.xlabel('Time (s)')
        plt.ylabel('Theta (deg)')
        plt.legend()
        plt.title('Theta vs Time')

        plt.figure()
        plt.plot(time_axis, self.steering_data, label='Steering Control')
        plt.scatter(time_axis, self.steering_data, color='blue', s=10)
        plt.xlabel('Time (s)')
        plt.ylabel('Steering angle (normalized)')
        plt.legend()
        plt.title('Steering Control vs Time')

        plt.figure()
        plt.plot(time_axis, self.throttle_data, label='Throttle Control')
        plt.plot(time_axis, self.brake_data, label='Brake Control', linestyle='--')
        plt.scatter(time_axis, self.throttle_data, color='blue', s=10)
        plt.scatter(time_axis, self.brake_data, color='red', s=10)
        plt.xlabel('Time (s)')
        plt.ylabel('Throttle/Brake (normalized)')
        plt.legend()
        plt.title('Throttle Control and Brake Control vs Time')

        plt.show()