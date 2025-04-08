import numpy as np
import vedo

class BoundingBox:
    def __init__(self, xmin, ymin, zmin, xmax, ymax, zmax, object=None):
        self.xmin, self.ymin, self.zmin = xmin, ymin, zmin
        self.xmax, self.ymax, self.zmax = xmax, ymax, zmax
        self.object = object

    def is_colliding(self, other):
        return (self.xmin <= other.xmax and self.xmax >= other.xmin and
                self.ymin <= other.ymax and self.ymax >= other.ymin and
                self.zmin <= other.zmax and self.zmax >= other.zmin)

def mesh_to_bounding_box(mesh):
    vertices = mesh.points  
    xmin, ymin, zmin = np.min(vertices, axis=0)
    xmax, ymax, zmax = np.max(vertices, axis=0)
    return BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)

class BVHNode:
    def __init__(self, box, left=None, right=None):
        self.box = box
        self.left = left
        self.right = right
        self.is_leaf = left is None and right is None

def build_bvh(boxes):
    if len(boxes) == 1:
        return BVHNode(boxes[0])

    boxes.sort(key=lambda b: b.xmin)
    mid = len(boxes) // 2
    left = build_bvh(boxes[:mid])
    right = build_bvh(boxes[mid:])
    xmin = min(left.box.xmin, right.box.xmin)
    ymin = min(left.box.ymin, right.box.ymin)
    zmin = min(left.box.zmin, right.box.zmin)
    xmax = max(left.box.xmax, right.box.xmax)
    ymax = max(left.box.ymax, right.box.ymax)
    zmax = max(left.box.zmax, right.box.zmax)
    parent_box = BoundingBox(xmin, ymin, zmin, xmax, ymax, zmax)
    return BVHNode(parent_box, left, right)

def detect_collisions_bvh(root):
    collisions = set()
    if not root or root.is_leaf:
        return collisions

    stack = [(root.left, root.right)]

    while stack:
        n1, n2 = stack.pop()
        if n1 and n2 and n1.box.is_colliding(n2.box):
            if n1.is_leaf and n2.is_leaf:
                collisions.add((n1.box, n2.box))
            else:
                if not n1.is_leaf:
                    stack.append((n1.left, n2))
                    stack.append((n1.right, n2))
                if not n2.is_leaf:
                    stack.append((n1, n2.left))
                    stack.append((n1, n2.right))
    return collisions