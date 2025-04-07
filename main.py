import numpy as np
import vedo



# 2d dart kinematics

class Dart:
    def __init__(self, mass=0.02, initial_velocity=20, initial_angle=45):
        self.mass = mass  # kg
        self.initial_velocity = initial_velocity  # m/s
        self.initial_angle = np.radians(initial_angle)  # Convert to radians
        self.gravity = 9.81  # m/s^2

    def calculate_trajectory(self, time):
        # Calculate the trajectory of the dart
        x = self.initial_velocity * np.cos(self.initial_angle) * time
        y = (self.initial_velocity * np.sin(self.initial_angle) * time) - (0.5 * self.gravity * time**2)
        return x, y

if __name__ == "__main__":
    # Create a new dart object
    dart = Dart()

    # Set up the plotter
    plotter = vedo.Plotter()
    plotter.show(axes=1, interactive=False)

    # Launch the dart
    for t in np.linspace(0, 5, 100):  # Simulate for 5 seconds
        x, y = dart.calculate_trajectory(t)
        dart_object = vedo.Sphere(pos=(x, y, 0), r=1).color("blue")
        plotter += dart_object
        plotter.show(resetcam=False)
plotter.show(interactive=True)