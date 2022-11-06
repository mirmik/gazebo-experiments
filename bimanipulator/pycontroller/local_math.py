import math


class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def rotate(self, angle):
        return Vec2(self.x * math.cos(angle) - self.y * math.sin(angle),
                    self.x * math.sin(angle) + self.y * math.cos(angle))

    def cross_by(self, angle):
        return Vec2(-self.y * angle, self.x * angle)

    def __add__(self, oth):
        return Vec2(self.x + oth.x, self.y + oth.y)

    def __sub__(self, oth):
        return Vec2(self.x - oth.x, self.y - oth.y)

    def __mul__(self, other):
        return Vec2(self.x * other, self.y * other)

    def __truediv__(self, other):
        return Vec2(self.x / other, self.y / other)

    def __str__(self):
        return "({}, {})".format(self.x, self.y)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __iter__(self):
        yield self.x
        yield self.y

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def norm(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def dir(self):
        return self / self.norm()

    def length(self):
        return self.norm()


class Screw2:
    def __init__(self, lin=Vec2(0, 0), ang=0):
        self.lin = lin
        self.ang = ang

    def __str__(self):
        return "({}, {})".format(self.lin, self.ang)

    def __mul__(self, other):
        return Screw2(self.lin * other, self.ang * other)

    def __rmul__(self, other):
        return self * other

    def __add__(self, other):
        return Screw2(
            lin=self.lin + other.lin,
            ang=self.ang + other.ang)

    def __sub__(self, other):
        return Screw2(
            lin=self.lin - other.lin,
            ang=self.ang - other.ang)

    def __truediv__(self, other):
        return Screw2(
            lin=self.lin / other,
            ang=self.ang / other)

    def to_pose(self, x):
        return Pose2(
            lin=self.lin * x,
            ang=self.ang * x)

    def kinematic_shift(self, arm):
        return Screw2(
            lin=self.lin + arm.cross_by(self.ang),
            ang=self.ang)

    def rotate(self, ang):
        return Screw2(
            lin=self.lin.rotate(ang),
            ang=self.ang)

    def __iter__(self):
        yield self.lin.x
        yield self.lin.y
        yield self.ang


class Pose2:
    def __init__(self, lin=Vec2(0, 0), ang=0):
        self.lin = lin
        self.ang = ang

    def __mul__(self, oth):
        return Pose2(
            ang=self.ang + oth.ang,
            lin=oth.lin.rotate(self.ang) + self.lin
        )

    def __str__(self):
        return "({}, {})".format(self.lin, self.ang)

    def inverse(self):
        return Pose2(
            lin=(-self.lin).rotate(-self.ang),
            ang=-self.ang
        )

    def to_screw(self):
        return Screw2(
            lin=self.lin,
            ang=self.ang
        )
