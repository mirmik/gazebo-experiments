#!/usr/bin/env python3

from rxsignal import *
from rxsignal.rxmqtt import *

rxmqtt = mqtt_rxclient("localhost", 1883, "bimanipulator")
rxmqtt.start_spin()

rxmqtt.publish("/worldctr/command", "restart")


def subscribe_model(a):
    jp0 = rxmqtt.rxsubscribe(f"/m{a}/j0/p/0").map(float)
    jp1 = rxmqtt.rxsubscribe(f"/m{a}/j1/p/0").map(float)
    jp2 = rxmqtt.rxsubscribe(f"/m{a}/j2/p/0").map(float)
    jv0 = rxmqtt.rxsubscribe(f"/m{a}/j0/v/0").map(float)
    jv1 = rxmqtt.rxsubscribe(f"/m{a}/j1/v/0").map(float)
    jv2 = rxmqtt.rxsubscribe(f"/m{a}/j2/v/0").map(float)
    return [jp0, jp1, jp2], [jv0, jv1, jv2]


jp0, jv0 = subscribe_model(1)
jp1, jv1 = subscribe_model(2)


# def foo(tgt, jp, jv):
#     target = rxconstant(tgt)
#     error_pos = target - jp
#     error_vel = -jv
#     pos_integral = rxintegral(error_pos, 0.005)
#     torque = (
#         error_pos * rxconstant(50) +
#         error_vel * rxconstant(50) +
#         pos_integral * rxconstant(30)
#     )
#     return torque

rxprint(jp1[0])


#torque0 = foo(2, jp0[0], jv0[0])
#torque1 = foo(2, jp0[1], jv0[1])
#rxmqtt.rxpublish("/m1/j0/t/0", torque0)
#rxmqtt.rxpublish("/m1/j1/t/0", torque1)

#torque0 = foo(2, jp1[0], jv1[0])
#torque1 = foo(2, jp1[1], jv1[1])
#rxmqtt.rxpublish("/m2/j0/t/0", torque0)
#rxmqtt.rxpublish("/m2/j1/t/0", torque1)

#i0 = rxinterval(1)

# rxprint(jp0[0])
# rxprint(torque0)
