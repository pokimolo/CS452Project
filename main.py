"""Draw the shadows and trailing lines of a dart"""
from vedo import *


world = Box([0,0,0], 35, 16, 16).wireframe()

board = Mesh("Dartboard.stl").c("red")
board.scale(.03)
board.rotate(90, (0, 0, 1))
board.pos(17, 0, 0)

dart = Mesh("Dart.stl").c("green")
dart.scale(0.03)  # Adjust this number to make the model smaller/larger
dart.rotate(-90, (0, 1, 0))  
dart.add_trail(n=100)
dart.add_shadow('y', -8)


# Setup the scene
plt = Plotter(interactive=False)
plt.show(world, dart, board, __doc__, viewup="-y")

for t in np.arange(0, 3.2, 0.04):
    dart.pos(9*t-15, sin(3-t), 2-t) # movement
    dart.update_trail().update_shadows()
    plt.render()



plt.interactive().close()