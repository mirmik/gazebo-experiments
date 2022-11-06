#!/usr/bin/env python3

import zengraph
from zengraph import *
import rxsignal
import rxsignal.rxmqtt
import rxsignal.flowchart
from rxsignal import *
import json
import numpy as np
import time

start_time = time.time()


def get_index_from_json(x, idx):
    return np.array(json.loads(x)[idx])


rxmqtt = rxsignal.rxmqtt.mqtt_rxclient("localhost", 1883)
rxmqtt.start_spin()
m1j2reaction2f = rxmqtt.rxsubscribe("/m1/j2/reaction_absolute").map(lambda x: get_index_from_json(x, "f2"))
m2j2reaction2f = rxmqtt.rxsubscribe("/m2/j2/reaction_absolute").map(lambda x: get_index_from_json(x, "f2"))
t = m1j2reaction2f.map(lambda x: time.time() - start_time)

wdg = rxsignal.flowchart.flowplot_application(t, m1j2reaction2f.map(
    lambda x: x[0]), m2j2reaction2f.map(lambda x: x[0]), interval=20, autoscale=True)
# zengraph.disp(wdg)
# zengraph.show()
