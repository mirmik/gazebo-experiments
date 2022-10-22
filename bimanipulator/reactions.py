#!/usr/bin/env python3

import zengraph
from zengraph import *
import rxsignal
import rxsignal.rxmqtt
import rxsignal.flowchart
from rxsignal import *
import json
import numpy as np


def get_index_from_json(x, idx):
    return np.array(json.loads(x)[idx])


rxmqtt = rxsignal.rxmqtt.mqtt_rxclient("localhost", 1883)
rxmqtt.start_spin()
m1reaction1f = rxmqtt.rxsubscribe("/m1/j2/reaction_absolute").map(lambda x: get_index_from_json(x, "f1"))
m1reaction2f = rxmqtt.rxsubscribe("/m1/j2/reaction_absolute").map(lambda x: get_index_from_json(x, "f2"))
m1reaction1t = rxmqtt.rxsubscribe("/m1/j2/reaction_absolute").map(lambda x: get_index_from_json(x, "t1"))
m1reaction2t = rxmqtt.rxsubscribe("/m1/j2/reaction_absolute").map(lambda x: get_index_from_json(x, "t2"))
m1j1reaction2t = rxmqtt.rxsubscribe("/m1/j1/reaction_absolute").map(lambda x: get_index_from_json(x, "t2"))
link2pose = rxmqtt.rxsubscribe("/m1/link_2_d/pose")
link1pose = rxmqtt.rxsubscribe("/m1/link_3/pose")

rxprint(m1reaction2t)
# rxprint(m1j1reaction2t.zip(m1reaction2t))
# rxprint(link2pose.zip(link1pose))

#t = rxsignal.rxinterval(0.1)
#wdg = rxsignal.flowchart.create_flowchart(t, t)
# zengraph.disp(wdg)
# zengraph.show(standalone=True)
