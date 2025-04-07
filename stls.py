import vedo
import vedo.colors as vc

class Dart:
    def __init__(self):
        self.object = vedo.load('assets/dart.stl')
        self.object.color("blue")
        self.object.rotate(angle=90, axis=(1, 0, 0), rad=False)
        self.mass = 20  # grams
        self.density = 1
        self.initial_velocity = 20  # m/s
        self.initial_x = 0
        self.initial_y = -1219.2
        self.initial_z = 0

    def calculate_trajectory(self, time):
        # Calculate the trajectory of the dart
        x = self.initial_x + self.initial_velocity * time
        y = self.initial_y
        z = self.initial_z
        return (x, y, z)

    def launch_dart(self, time, plotter):
        # Launch the dart and calculate its trajectory
        trajectory = self.calculate_trajectory(time)
        self.object.pos(trajectory)
        plotter.render()

def main():
    room_width = 1219.2
    room_depth = 2438.4
    room_height = 2438.4  # Assuming an 8-foot ceiling
    

    # Create the room as a box
    room = vedo.Box(size=(room_width, room_height, room_depth))
    room.color("lightgray").alpha(0.5)  # Light gray with some transparency

    # Create a new dart object
    dart = Dart()

    # Set up the plotter
    plotter = vedo.Plotter()
    plotter.show(room, dart.object, axes=1, interactive=False)
    plotter.camera.position = (0, 2000, 0)  # Position the camera along the Z-axis
    plotter.camera.look_at = (0, 0, 0)     # Point the camera towards the origin
    plotter.camera.up = (0, 0, 1)  
    plotter+= dart.object
    plotter+= room       
    plotter.show()
  

if __name__ == "__main__":
    main()
