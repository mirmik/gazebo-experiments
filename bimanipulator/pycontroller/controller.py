#!/usr/bin/env python3

from copy import deepcopy
import rxsignal
from rxsignal import *
from rxsignal.rxmqtt import *
from rxsignal.filter import *
import json
from dataclasses import dataclass
from rxsignal.flowchart import *
from control_loop import control_loop

DELTA = 0.01


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


@dataclass
class ModelInformation:
    jp: list
    jv: list
    reaction: Screw2


@dataclass
class CargoInformation:
    pose: Pose2


@dataclass
class Information:
    model0: ModelInformation
    model1: ModelInformation
    cargo: CargoInformation


def clamp(v, min, max):
    return min if v < min else max if v > max else v


def correction(U, T, Hs):
    MAX = 0.5
    norm = numpy.linalg.norm(U)
    if norm > MAX:
        U = U * MAX / norm
    return U


def rx_correction(U, T, Hs):
    return rxsignal.zip(U, T, *Hs).map(lambda uhs:
                                       correction(uhs[0],
                                                  uhs[1],
                                                  uhs[2:]))


def rx_svd_backpack(target, signals, t2, s2, f=1):
    z1 = rxsignal.zip(target, *signals)
    z2 = rxsignal.zip(t2, *s2)

    z = z1.zip(z2)

    def foo(T, Hs, T2, Hs2):
        return svd_backpack(
            [*T]+[*T2], (
                [*(Hs[0])]+[*(Hs2[0])],
                [*(Hs[1])]+[*(Hs2[1])]
            ), f=f)

    return z.map(lambda x: foo(x[0][0], x[0][1:], x[1][0], x[1][1:]))


def parse_pose3_to_vec2(str):
    x, y, z, a, b, c, w = str.split(",")
    return Vec2(-float(x), float(z))


def parse_pose3_to_pose2(str):
    x, y, z, a, b, c, w = str.split(",")
    return Pose2(lin=Vec2(-float(x), float(z)), ang=0)


def set_target(a, j, v):
    rxmqtt.publish(f"/m{a+1}/j{j}/target", "%.5f" % v)


def set_targets(index, targets):
    set_target(index, 0, targets[0])
    set_target(index, 1, targets[1])


rxmqtt = mqtt_rxclient("localhost", 1883, "bimanipulator")
rxmqtt.start_spin()

rxmqtt.publish("/worldctr/restart", "")
time.sleep(0.5)

rxmqtt.publish("/m1/j0/regulator", "600,100,100,-1000,1000")
rxmqtt.publish("/m1/j1/regulator", "600,100,100,-1000,1000")
rxmqtt.publish("/m2/j0/regulator", "600,100,100,-1000,1000")
rxmqtt.publish("/m2/j1/regulator", "600,100,100,-1000,1000")

L = 0.8
cargo_vec = rxmqtt.rxsubscribe("/cargo/pose").map(parse_pose3_to_vec2)
S0 = rxmqtt.rxsubscribe("/m1/pose").map(parse_pose3_to_pose2)
S1 = rxmqtt.rxsubscribe("/m2/pose").map(parse_pose3_to_pose2)

# Винты чувствительности в локальном пространстве.
W0 = Screw2(lin=Vec2(0, 0), ang=1)
W1 = Screw2(lin=Vec2(0, 0), ang=1)

# константы звеньев манипулятора
E = Pose2(lin=Vec2(0, 0), ang=0)
C0 = Pose2(lin=Vec2(0, L), ang=0)
C1 = Pose2(lin=Vec2(0, L), ang=0)


I0 = 0
I1 = 0

set_target(0, 0, I0)
set_target(0, 1, I1)
set_target(1, 0, 0)
set_target(1, 1, 0)

time.sleep(1)


def array_to_screw2(arr):
    t2 = json.loads(arr)["t2"]
    f2 = json.loads(arr)["f2"]
    return Screw2(lin=Vec2(f2[0], f2[2]), ang=t2[1])


def subscribe_model(a):
    a = a + 1
    jp0 = rxmqtt.rxsubscribe(f"/m{a}/j0/p/0").map(float)
    jp1 = rxmqtt.rxsubscribe(f"/m{a}/j1/p/0").map(float)
    jv0 = rxmqtt.rxsubscribe(f"/m{a}/j0/v/0").map(float)
    jv1 = rxmqtt.rxsubscribe(f"/m{a}/j1/v/0").map(float)
    jie0 = rxmqtt.rxsubscribe(f"/m{a}/j1/ie/0").map(float)
    jie1 = rxmqtt.rxsubscribe(f"/m{a}/j1/ie/0").map(float)
    reaction = rxmqtt.rxsubscribe(f"/m{a}/j2/reaction_absolute").map(array_to_screw2)
    return jp0, jp1, jv0, jv1, jie0, jie1, reaction


subscriptions0 = subscribe_model(0)
subscriptions1 = subscribe_model(1)


def to_model_information(subscriptions):
    return ModelInformation(
        jp=[subscriptions[0], subscriptions[1]],
        jv=[subscriptions[2], subscriptions[3]],
        reaction=subscriptions[4]
    )


def to_cargo_information(cargo_vec):
    return CargoInformation(
        pose=cargo_vec
    )


def to_information(tpl):
    minfo0, minfo1, cinfo = tpl
    return Information(
        model0=minfo0,
        model1=minfo1,
        cargo=cinfo
    )


minfo0 = rxsignal.zip(*subscriptions0).map(to_model_information)
minfo1 = rxsignal.zip(*subscriptions1).map(to_model_information)
cinfo = rxsignal.zip(cargo_vec).map(to_cargo_information)
info = rxsignal.zip(minfo0, minfo1, cinfo).map(to_information)


targets = info.map(control_loop)


def set_targets(targets):
    set_target(0, 0, targets[0])
    set_target(0, 1, targets[1])
    set_target(1, 0, targets[2])
    set_target(1, 1, targets[3])


targets.subscribe(set_targets)

# def control_model(subindex, postarget, joint0_offset, S, subscriptions):
#     jp, jv, jie, reaction = subscriptions

#     filtered_reactions = reaction
#     # rxsignal.aperiodic_filter(reaction, 0, DELTA, init=Screw2())

#     # Тензоры положений звеньев.
#     L0 = S * jp[0].map(lambda x: W0.to_pose(x))
#     L1 = jp[1].map(lambda x: W1.to_pose(x))
#     P0 = L0 * C0 * L1 * C1

#     # Вектора чувствительности в абсолютной системе:
#     H0 = rxsignal.zip(L0, L1).map(lambda l: W0
#                                   .rotate((E).ang)
#                                   .kinematic_shift((l[0]*C0*l[1]*C1).lin
#                                                    .rotate((E).ang)
#                                                    )
#                                   .lin)
#     H1 = rxsignal.zip(L0, L1).map(lambda l: W1
#                                   .rotate((l[0]*C0).ang)
#                                   .kinematic_shift((l[1]*C1).lin
#                                                    .rotate((l[0]*C0).ang)
#                                                    )
#                                   .lin)

#     # Винт скорости выходгого звена
#     V = rxsignal.zip(jv[0], jv[1]).map(lambda v: W0 * v[0] + W1 * v[1])

#     # Целевой вектор
#     T = P0.zip(postarget).map(lambda pv:
#                               (pv[1] - pv[0].lin) * 2)

#     T = T - filtered_reactions.map(lambda r: r.lin) * 0.005

#     U = rx_svd_backpack(T, (H0, H1),
#                         joint0_offset, (H0 * 0.0001, [0, 0]),
#                         f=0.01)
#     U = rx_correction(U, T, (H0, H1))

#     targets = rxintegral(U, 0.01, init=numpy.array([I0, I1]))
#     targets.subscribe(lambda x: set_targets(subindex, x))


# TRIGGER = rxmqtt.rxsubscribe("/m1/j0/p/0")
# target0_task = cargo_vec + TRIGGER.map(lambda x: Vec2(-0.5, 0))
# target1_task = cargo_vec + TRIGGER.map(lambda x: Vec2(0.5, 0))

# target0 = Commutator(target0_task)
# target1 = Commutator(target1_task)
# control_model(0, postarget=target0, joint0_offset=[-2*L, 0],
#               S=S0, subscriptions=subscriptions0)

# control_model(1, postarget=target1, joint0_offset=[2*L, 0],
#               S=S1, subscriptions=subscriptions1)

# target0_task_2 = cargo_vec + TRIGGER.map(lambda x: Vec2(-0.5, 0))
# target1_task_2 = cargo_vec + TRIGGER.map(lambda x: Vec2(0.5, 0))
# target0_2 = Commutator(target0_task_2)
# target1_2 = Commutator(target1_task_2)
# control_model(0, postarget=target0_2, joint0_offset=[-2*L, 0],
#               S=S0, subscriptions=subscriptions0)

# control_model(1, postarget=target1_2, joint0_offset=[2*L, 0],
#               S=S1, subscriptions=subscriptions1)

# target0_2.unbind()
# target1_2.unbind()


# time.sleep(4)
# print("PHASE2")

# START0 = 0.38
# START1 = L+0.03

# target0_task = TRIGGER.map(lambda x: Vec2(-START0, START1))
# target0.rebind(target0_task)

# target1_task = TRIGGER.map(lambda x: Vec2(START0, START1))
# target1.rebind(target1_task)

# time.sleep(4)
# print("PHASE3")

# curtime = time.time()
# target0_task_2 = TRIGGER.map(lambda x: Vec2(-START0, START1+(time.time()-curtime)*0.1))
# target0_2.rebind(target0_task_2)

# target1_task_2 = TRIGGER.map(lambda x: Vec2(START0, START1+(time.time()-curtime)*0.1))
# target1_2.rebind(target1_task_2)

# time.sleep(4)
# print("PHASE4")
# SY = START1+0.4

# stime = time.time()
# t = TRIGGER.map(lambda x: time.time()-stime)
# target0.rebind(t.map(lambda t: Vec2(-START0+t*0.1, SY)))
# target1.rebind(t.map(lambda t: Vec2(START0+t*0.1, SY)))
# time.sleep(4)

# while(True):

#     stime = time.time()
#     t = TRIGGER.map(lambda x: time.time()-stime)
#     target0.rebind(t.map(lambda t: Vec2(-START0+0.4-t*0.1, SY)))
#     target1.rebind(t.map(lambda t: Vec2(START0+0.4-t*0.1, SY)))
#     time.sleep(8)

#     stime = time.time()
#     t = TRIGGER.map(lambda x: time.time()-stime)
#     target0.rebind(t.map(lambda t: Vec2(-START0-0.4+t*0.1, SY)))
#     target1.rebind(t.map(lambda t: Vec2(START0-0.4+t*0.1, SY)))
#     time.sleep(8)
