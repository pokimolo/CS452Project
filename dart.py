import vedo
from vedo import Box, Mesh
import numpy as np
import os
from playsound import playsound
from bvh import *  # homebrewed bounding volume hierarchy code from hw4

class Dartboard:
    def __init__(self):
        self.ring0 = vedo.load("ring0.stl").scale(0.03)
        self.ring0.rotate(90, (0, 0, 1))
        self.ring0.pos(17, 0, 0)
        self.ring0.c("red")
        
        self.ring1 = vedo.load("ring1.stl").scale(0.03)
        self.ring1.rotate(90, (0, 0, 1))
        self.ring1.pos(17, 0.01, 0)
        self.ring1.c("green")
        
        self.ring2 = vedo.load("ring2.stl").scale(0.03)
        self.ring2.rotate(90, (0, 0, 1))
        self.ring2.pos(17, 0.02, 0)
        self.ring2.c("white")
        
        self.ring3 = vedo.load("ring3.stl").scale(0.03)
        self.ring3.rotate(90, (0, 0, 1))
        self.ring3.pos(17, 0.03, 0)
        self.ring3.c("red")
        
        self.ring4 = vedo.load("ring4.stl").scale(0.03)
        self.ring4.rotate(90, (0, 0, 1))
        self.ring4.pos(17, 0.04, 0)
        self.ring4.c("black")

        # Create bounding boxes for each ring
        self.ring0bb = mesh_to_bounding_box(self.ring0)
        self.ring1bb = mesh_to_bounding_box(self.ring1)
        self.ring2bb = mesh_to_bounding_box(self.ring2)
        self.ring3bb = mesh_to_bounding_box(self.ring3)
        self.ring4bb = mesh_to_bounding_box(self.ring4)
        
        # Store the rings and bounding boxes
        self.rings = [self.ring0, self.ring1, self.ring2, self.ring3, self.ring4]
        self.bbs = [self.ring0bb, self.ring1bb, self.ring2bb, self.ring3bb, self.ring4bb]

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
        self.velocity = np.array([90.0, 0.0, -2.0])  # m/s (forward and downward)
        self.angular_velocity = np.array([0, 0, -12.0])  # rad/s (spin)
        self.com_offset = np.array([0.0, 0.0, -0.077])  # Example COM position (adjust as needed)
        self.moment_of_inertia = np.array([0.15, 0.15, 0.002])  # Simplified moment of inertia
        self.g = np.array([0, -9.81, 0])  # gravity
        self.sounded = False
        self.bb = mesh_to_bounding_box(self.object)

    def set_position(self, y, z):
        self.position[1] = y
        self.position[2] = z

    def update_box(self):
        self.bb = mesh_to_bounding_box(self.object)

    def launch(self, dt, board_bbs=None, world_bb=None):
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

        for i in reversed(range(0, 4)):
            bb = board_bbs[i]
            if self.bb.is_colliding(bb):
                if not self.sounded:
                    playsound('dartsound2.m4a')
                    if i == 0:
                        print("Hit the bullseye!")
                    else:
                        print(f"Collided with ring {i}") 

                self.sounded = True
                self.velocity = np.zeros(3) 
                self.angular_velocity = np.zeros(3)  
                self.object.color("orange")  
                break
                
                

        if not self.bb.is_colliding(world_bb):
            self.velocity = np.zeros(3)
            self.angular_velocity = np.zeros(3)
            self.object.color("blue")
            self.position = np.clip(self.position, world_bb.xmin, world_bb.xmax)

def main():
    dart = Dart()
    plotter = vedo.Plotter(title="Physics-Based Dart Throw", size=(1000, 600))
    world = Box([0, 0, 0], 35, 16, 16).wireframe()
    board = Dartboard()
    board_assembly = vedo.Assembly(board.rings)

    def animate(event):
        # Use the board's bounding boxes
        board_bb = board.bbs
        world_bb = mesh_to_bounding_box(world)
        dart.launch(dt=0.01, board_bbs=board_bb, world_bb=world_bb)
        plotter.render()

    plotter.show(world, board_assembly, dart.object, axes=1, interactive=False)
    plotter.add_callback("timer", animate)
    plotter.timer_callback("create", dt=10)  # 10ms per frame
    plotter.interactive()

if __name__ == "__main__":
    main()
