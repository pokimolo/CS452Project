import vedo
from vedo import Box, Mesh
import numpy as np
import os

class Dart:
    def __init__(self):
        stl_path = os.path.join(os.path.dirname(__file__), "Dart.stl")
        self.object = vedo.load(stl_path) or vedo.Cone(r=5, height=20).color("blue")

        self.object.scale(0.03)
        self.object.rotate(angle=270, axis=(0, 1, 0), rad=False)
        self.object.color("green")
        self.object.add_trail(n=100)
        self.object.add_shadow('y', -8)

        # Physical properties
        self.mass = 0.02  # kg
        self.drag_coefficient = 0.47
        self.air_density = 1.225  # kg/m^3
        self.cross_sectional_area = 0.0005  # m²

        self.position = np.array([-15.0, 1.5, 2.0])  # meters (start position)
        self.velocity = np.array([40.0, 0.0, -3.0])  # m/s (forward and downward)

        self.g = np.array([0, -9.81, 0])  # gravity

    def animate(self, dt):
        speed = np.linalg.norm(self.velocity)
        drag_force_mag = 0.5 * self.drag_coefficient * self.air_density * self.cross_sectional_area * speed**2
        drag_force = -drag_force_mag * self.velocity / speed if speed != 0 else np.zeros(3)
        acceleration = (drag_force / self.mass) + self.g
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        self.object.pos(self.position)
        self.object.update_trail().update_shadows()


def main():
    dart = Dart()
    plotter = vedo.Plotter(title="Physics-Based Dart Throw", size=(1000, 600))

    # Setup the environment
    world = Box([0, 0, 0], 35, 16, 16).wireframe()

    # Load and transform dartboard
    board = Mesh("Dartboard.stl").c("red")
    board.scale(0.03)
    board.rotate(90, (0, 0, 1))
    board.pos(17, 0, 0)

    # Animate with timer
    def animate(event):
        dart.animate(dt=0.01)  # timestep of 10ms
        plotter.render()

    plotter.show(world, board, dart.object, axes=1, interactive=False)
    plotter.add_callback("timer", animate)
    plotter.timer_callback("create", dt=10)  # 10ms
    plotter.interactive()


if __name__ == "__main__":
    main()
