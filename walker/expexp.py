#!/usr/bin/env python3

import matplotlib.pyplot as plot
import numpy
import math

A = 1
B = -0.5

P = 0.3
Q = 0.3

Time = 5
Step = 0.001
t = numpy.linspace(0,Time,Time/Step)
x = A*numpy.exp(-1/P*t) + B*numpy.exp(-1/Q*t)
y = A*numpy.exp(-1/P*t)
z = B*numpy.exp(-1/Q*t)

plot.plot(t,x,t,y,t,z) 
plot.show()
