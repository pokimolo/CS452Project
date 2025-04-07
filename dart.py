import vedo
import numpy as np
import time

def throw_dart(velocity, duration, time_step):
    for t in np.arange(0, duration, time_step):
        # Update position
        x = velocity * t
        y = 0  # Assuming straight throw along x-axis
        z = 100 - 9.81 * t**2  # Simple gravity effect
        dart.pos(x, y, z)
        plotter.render()
        time.sleep(time_step)
# Load the dart and dartboard models
dart = vedo.load('assets/Dart.stl').color('blue')
dart.rotate(270, (1, 0, 0))  # Rotate to match the dartboard orientation

dartboard = vedo.load('assets/Dartboard.stl').color('red')

# Initialize the plotter
plotter = vedo.Plotter()



dart_y = 1219.2
dart_x = 0
dart_z = 20

dartboard_y = 0
dartboard_x = 0
dartboard_z = 0

# Set the initial position of the dart
dart.pos(dart_x, dart_y, dart_z)
# Set the position of the dartboard
dartboard.pos(dartboard_x, dartboard_y, dartboard_z)
# Set the camera position
plotter.camera.position = (2000, 0,  0)
# Add models to the plotter
plotter += dartboard
plotter += dart

# Display the scene
plotter.show()