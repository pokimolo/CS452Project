import vedo
from vedo import Box, Mesh
import numpy as np
import os
import numpy as np
from playsound import playsound
from bvh import * # homebrewed bounding volume hierarchy code from hw4
class Dart:
    def __init__(self):
        stl_path = os.path.join(os.path.dirname(__file__), "Dart.stl")
        self.object = vedo.load(stl_path) or vedo.Cone(r=5, height=20).color("blue")
        self.object.scale(0.03)
        self.object.rotate(angle=270, axis=(0, 1, 0), rad=False)
        self.object.color("green")
        self.object.add_trail(n=100)
        self.object.add_shadow('y', -8)
        self.mass = 0.092  # kg (converted from 0.203 lb)
        self.drag_coefficient = 0.47
        self.air_density = 1.225  # kg/m^3
        self.cross_sectional_area = 0.00635  # m² 
        self.position = np.array([-15.0, 1.5, 2.0])  # meters (start position)
        self.velocity = np.array([40.0, 0.0, -2.0])  # m/s (forward and downward)
        self.angular_velocity = np.array([0, 0, -12.0])  # rad/s (spin)
        self.com_offset = np.array([0.0, 0.0, -0.077] )  # Example COM position (adjust as needed)
        self.moment_of_inertia = np.array([0.15, 0.15, 0.002])  # Simplified moment of inertia
        self.g = np.array([0, -9.81, 0])  # gravity
        self.sounded = False
        self.bb = mesh_to_bounding_box(self.object)

    def update_box(self):
        self.bb = mesh_to_bounding_box(self.object)

    def animate(self, dt, board_bb=None, world_bb=None):
        speed = np.linalg.norm(self.velocity)
        drag_force_mag = 0.5 * self.drag_coefficient * self.air_density * self.cross_sectional_area * speed**2
        drag_force = -drag_force_mag * self.velocity / speed if speed != 0 else np.zeros(3)
        acceleration = (drag_force / self.mass) + self.g
        rotational_acceleration = np.cross(self.angular_velocity, self.moment_of_inertia)
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        self.angular_velocity += rotational_acceleration * dt

        self.object.pos(self.position)
        self.object.rotate(self.angular_velocity[2] * dt, axis=(0, 0, 1))  # Rotate around Z-axis
        self.object.update_trail().update_shadows()
        self.update_box()
        
        if self.bb.is_colliding(board_bb):
            if not self.sounded:
                playsound('dartsound2.m4a')
            self.sounded = True
            self.velocity = np.zeros(3)
            self.angular_velocity = np.zeros(3)
            self.object.color("orange")
            

            

        

def main():
    dart = Dart()
    plotter = vedo.Plotter(title="Physics-Based Dart Throw", size=(1000, 600))
    world = Box([0, 0, 0], 35, 16, 16).wireframe()
    board = Mesh("Dartboard.stl").c("red")
    board.scale(0.03)
    board.rotate(90, (0, 0, 1))
    board.pos(17, 0, 0)

    def animate(event):
        board_bb = mesh_to_bounding_box(board)
        world_bb = mesh_to_bounding_box(world)

        dart.animate(dt=0.01, board_bb=board_bb, world_bb=world_bb)
        # dart.animate(dt=0.01)

        plotter.render()

    plotter.show(world, board, dart.object, axes=1, interactive=False)
    plotter.add_callback("timer", animate)
    plotter.timer_callback("create", dt=10)  # 10ms per frame
    plotter.interactive()

if __name__ == "__main__":
    main()
