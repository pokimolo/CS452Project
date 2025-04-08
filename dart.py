import vedo
import numpy as np
import os

class Dart:
    def __init__(self):
        stl_path = os.path.join(os.path.dirname(__file__), "Dart.stl")
        self.object = vedo.load(stl_path) or vedo.Cone(r=5, height=20).color("blue")
        self.object.color("blue")
        self.object.rotate(angle=270, axis=(0, 1, 0), rad=False)

        # Physical properties
        self.mass = 0.02  # kg
        self.drag_coefficient = 0.47
        self.air_density = 1.225  # kg/m^3
        self.cross_sectional_area = 0.0005  # m²

        self.position = np.array([0.0, -1.2192, 0.0])  # meters
        self.velocity = np.array([20.0, 0.0, 0.0])     # m/s

        self.g = np.array([0, -9.81, 0])  # m/s²

    def animate(self, dt):
        speed = np.linalg.norm(self.velocity)
        drag_force_mag = 0.5 * self.drag_coefficient * self.air_density * self.cross_sectional_area * speed**2
        drag_force = -drag_force_mag * self.velocity / speed if speed != 0 else np.zeros(3)
        acceleration = (drag_force / self.mass) + self.g
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        self.object.pos(self.position)

def main():
    dart = Dart()
    plotter = vedo.Plotter()

    # Define a timer callback for animation
    def animate(event):
        nonlocal dart
        dart.animate(dt=0.01)  # 10ms step
        plotter.render()

    plotter.show(dart.object, axes=1, interactive=False)
    plotter.add_callback("timer", animate)
    plotter.timer_callback("create", dt=10)  # milliseconds
    plotter.interactive()

if __name__ == "__main__":
    main()
