"""Draw the shadows and trailing lines of a dart"""
from vedo import *
from pynput import mouse


world = Box([0,0,0], 30, 16, 16).wireframe()

plane1 = Mesh(dataurl+"cessna.vtk").c("green")
plane1.add_trail(n=100)
plane1.add_shadow('z', -4).add_shadow('y', 8)


# Setup the scene
plt = Plotter(interactive=False)
plt.show(world, plane1, __doc__, viewup="-x")

def on_click(x, y, button, pressed):
    if pressed:
        print(f'Mouse clicked at ({x}, {y}) with {button}')
        for t in np.arange(0, 3.2, 0.04):
            plane1.pos(9*t-15, 2-t, sin(3-t)) # movement
            plane1.update_trail().update_shadows()
            plt.render()

with mouse.Listener(on_click=on_click) as listener:
    listener.join()

plt.interactive().close()