from local_math import *
import numpy as np

L = 0.8

# Винты чувствительности в локальном пространстве.
W0 = Screw2(lin=Vec2(0, 0), ang=1)
W1 = Screw2(lin=Vec2(0, 0), ang=1)

# константы звеньев манипулятора
E = Pose2(lin=Vec2(0, 0), ang=0)
C0 = Pose2(lin=Vec2(0, L), ang=0)
C1 = Pose2(lin=Vec2(0, L), ang=0)

I0 = 0
I1 = 0


def solve_svd(A, b):
    # pinv
    x = np.linalg.pinv(A).dot(b)
    return x


def svd_backpack(target, signals, f=1):
    # Коэффициент f штрафут решение за чрезмерную длину полученного вектора
    nt = np.array([*target] + [0])
    ns = [np.array([*s] + [f], dtype=np.float64) for s in signals]

    for s in ns:
        s.shape = (s.shape[0], 1)

    matrix = np.hstack(ns)

    x = solve_svd(matrix, nt.T)
    return x


def correction(U, T, Hs):
    MAX = 0.5
    norm = np.linalg.norm(U)
    if norm > MAX:
        U = U * MAX / norm
    return U


class integral:
    def __init__(self, delta, init=0):
        self.value = init
        self.delta = delta

    def __call__(self, x):
        self.value += x * self.delta
        return self.value

# def rxintegral(signal, delta, init=0):
#    state = FeedbackSubject(init)
#    newstate = (state + signal * delta)
#    state.bind(newstate)
#    return state


integral0 = integral(0.01, init=np.array([I0, I1]))
integral1 = integral(0.01, init=np.array([I0, I1]))


def control_model(subindex, postarget, joint0_offset, S, subscriptions):
    jp, jv, jie, reaction = subscriptions

    filtered_reactions = reaction
    # rxsignal.aperiodic_filter(reaction, 0, DELTA, init=Screw2())

    # Тензоры положений звеньев.
    L0 = W0.to_pose(S * jp[0])
    L1 = W1.to_pose(jp[1])
    P0 = L0 * C0 * L1 * C1

    # Вектора чувствительности в абсолютной системе:
    H0 = (W0.rotate((E).ang)
          .kinematic_shift((L0*C0*L1*C1).lin
                           .rotate((E).ang)
                           )
          .lin)
    H1 = (W1
          .rotate((L0*C0).ang)
          .kinematic_shift((L1*C1).lin
                           .rotate((L0*C0).ang)
                           )
          .lin)

    # Винт скорости выходгого звена
    V = W0 * jv[0] + W1 * jv[1]

    # Целевой вектор
    T = (postarget - P0.lin) * 2
    T = T - filtered_reactions.lin * 0.005

    U = svd_backpack(T, (H0, H1),
                     joint0_offset, (H0 * 0.0001, [0, 0]),
                     f=0.01)
    U = correction(U, T, (H0, H1))

    targets = integral0(U)
    return targets


def control_loop(info):
    print("Control loop")
    return 0, 0, 0, 0
