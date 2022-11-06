from local_math import *

# Винты чувствительности в локальном пространстве.
W0 = Screw2(lin=Vec2(0, 0), ang=1)
W1 = Screw2(lin=Vec2(0, 0), ang=1)


def control_model(subindex, postarget, joint0_offset, S, subscriptions):
    jp, jv, jie, reaction = subscriptions

    filtered_reactions = reaction
    # rxsignal.aperiodic_filter(reaction, 0, DELTA, init=Screw2())

    # Тензоры положений звеньев.
    L0 = S * jp[0].map(lambda x: W0.to_pose(x))
    L1 = jp[1].map(lambda x: W1.to_pose(x))
    P0 = L0 * C0 * L1 * C1

    # Вектора чувствительности в абсолютной системе:
    H0 = rxsignal.zip(L0, L1).map(lambda l: W0
                                  .rotate((E).ang)
                                  .kinematic_shift((l[0]*C0*l[1]*C1).lin
                                                   .rotate((E).ang)
                                                   )
                                  .lin)
    H1 = rxsignal.zip(L0, L1).map(lambda l: W1
                                  .rotate((l[0]*C0).ang)
                                  .kinematic_shift((l[1]*C1).lin
                                                   .rotate((l[0]*C0).ang)
                                                   )
                                  .lin)

    # Винт скорости выходгого звена
    V = rxsignal.zip(jv[0], jv[1]).map(lambda v: W0 * v[0] + W1 * v[1])

    # Целевой вектор
    T = P0.zip(postarget).map(lambda pv:
                              (pv[1] - pv[0].lin) * 2)

    T = T - filtered_reactions.map(lambda r: r.lin) * 0.005

    U = rx_svd_backpack(T, (H0, H1),
                        joint0_offset, (H0 * 0.0001, [0, 0]),
                        f=0.01)
    U = rx_correction(U, T, (H0, H1))

    targets = rxintegral(U, 0.01, init=numpy.array([I0, I1]))
    targets.subscribe(lambda x: set_targets(subindex, x))


def control_loop(info):
    print("Control loop")
    return 0, 0, 0, 0
