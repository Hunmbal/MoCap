import numpy as np

class Vector3D:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.vector = np.array([x, y, z])

    @property
    def x(self):
        return self.vector[0]

    @property
    def y(self):
        return self.vector[1]

    @property
    def z(self):
        return self.vector[2]

    @staticmethod
    def from_array(array):
        return Vector3D(array[0], array[1], array[2])

    def to_array(self):
        return self.vector

    def __add__(self, other):
        return Vector3D.from_array(self.vector + other.vector)

    def __sub__(self, other):
        return Vector3D.from_array(self.vector - other.vector)

    def __mul__(self, scalar):
        return Vector3D.from_array(self.vector * scalar)

    def __truediv__(self, scalar):
        return Vector3D.from_array(self.vector / scalar)

    def dot(self, other):
        return np.dot(self.vector, other.vector)

    def cross(self, other):
        return Vector3D.from_array(np.cross(self.vector, other.vector))

    def magnitude(self):
        return np.linalg.norm(self.vector)

    def normalized(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector3D()
        return Vector3D.from_array(self.vector / mag)

    def __repr__(self):
        return f"Vector3D(x={self.x}, y={self.y}, z={self.z})"
