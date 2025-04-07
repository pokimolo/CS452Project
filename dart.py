import vedo
import numpy as np
import os

class Dart:
    def __init__(self):
        stl_path = os.path.join(os.path.dirname(__file__), "Dart.stl")
        self.object = vedo.load(stl_path) or vedo.Cone(r=5, height=20).color("blue")
        self.object.color("blue")
        self.object.rotate(angle=90, axis=(1, 0, 0), rad=False)

        # Physical properties
        self.mass = 0.02  # kg (20 grams)
        self.drag_coefficient = 0.47  # Approx for cone/cylinder
        self.air_density = 1.225  # kg/m^3
        self.cross_sectional_area = 0.0005  # m² (guess)

        # Initial conditions
        self.position = np.array([0.0, -1.2192, 0.0])  # meters
        self.velocity = np.array([20.0, 0.0, 0.0])     # m/s

        self.g = np.array([0, -9.81, 0])  # gravity (negative y-axis)

    def calculate_trajectory(self, dt, steps):
        positions = []
        velocity = self.velocity.copy()
        position = self.position.copy()

        for _ in range(steps):
            speed = np.linalg.norm(velocity)
            drag_force_mag = 0.5 * self.drag_coefficient * self.air_density * self.cross_sectional_area * speed**2
            drag_force = -drag_force_mag * velocity / speed if speed != 0 else np.zeros(3)
            acceleration = (drag_force / self.mass) + self.g

            # Euler integration
            velocity += acceleration * dt
            position += velocity * dt
            positions.append(position.copy())

        return positions

    def launch_dart(self, time, plotter):
        # Launch the dart and update position using physics
        positions = self.calculate_trajectory(dt=0.05, steps=int(time / 0.05))
        for pos in positions:
            self.object.pos(pos)
            plotter.render()
def main():
    dart = Dart()
    trajectory = dart.calculate_trajectory(dt=0.0005, steps=10000)  # 5 seconds of motion
    for i, pos in enumerate(trajectory[::10]):  # Print every 10th position
        print(f"t={i * 0.5:.1f}s: pos={pos}")

if __name__ == '__main__':
    main()