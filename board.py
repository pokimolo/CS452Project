import vedo
from vedo import Mesh
import numpy as np
import os

def main():
    ring0 = vedo.load("ring0.stl").scale(0.03)
    ring0.rotate(90, (0, 0, 1))
    ring0.pos(17, 0, 0)
    ring0.c("red")
    ring1 = vedo.load("ring1.stl").scale(0.03)
    ring1.rotate(90, (0, 0, 1))
    ring1.pos(17, 0, 0)
    ring1.c("green")
    ring2 = vedo.load("ring2.stl").scale(0.03)
    ring2.rotate(90, (0, 0, 1))
    ring2.pos(17, 0, 0)
    ring2.c("white")
    ring3 = vedo.load("ring3.stl").scale(0.03)
    ring3.rotate(90, (0, 0, 1))
    ring3.pos(17, 0, 0)
    ring3.c("red")
    ring4 = vedo.load("ring4.stl").scale(0.03)
    ring4.rotate(90, (0, 0, 1))
    ring4.pos(17, 0, 0)
    ring4.c("black")


   
    plotter = vedo.Plotter(title="Dartboard", size=(1000, 600))

    # Show the meshes in the plotter
    plotter.show(ring0, ring1, ring2, ring3, ring4, axes=1, interactive=True)

if __name__ == "__main__":
    main()
