#!/usr/bin/env python3

import matplotlib.pyplot as plot
import numpy
import math

t = []
x = []
v = []

X = 0
V = 0
M = 0.33*16
print(M)

pos_integral = 0
spd_integral = 0

pos_target = 1
spd_target = 0.1

Time = 0.003#20
Step = 0.001

#spd_T1 = 0.3
#spd_T2 = 4
spd_ksi = 1
spd_T = 0.2

#spd_T= math.sqrt( spd_T1 * spd_T2 )
#spd_ksi = (spd_T1 + spd_T2) / spd_T / 2
spd_A = 0.33

#pos_T1 = 1
#pos_T2 = 20
pos_ksi = 1
pos_T = 1


spd_Kp = 2.*spd_ksi / spd_T * spd_A
spd_Ki = 1. / spd_T / spd_T * spd_A
#spd_Kp = (1/spd_T1 + 1/spd_T2) * spd_A
#spd_Ki = (1/spd_T1 * 1/spd_T2) * spd_A

pos_Kp = 2.*pos_ksi / pos_T
pos_Ki = 1. / pos_T / pos_T
#pos_Kp = (1/pos_T1 + 1/pos_T2)
#pos_Ki = (1/pos_T1 * 1/pos_T2)

for time in numpy.linspace(0,Time,Time/Step):
	t.append(time)

#	pos_error = pos_target - X
#	pos_integral += pos_error * Step
#	spd_target = pos_Kp * pos_error + pos_Ki * pos_integral

	spd_error = spd_target - V
	spd_integral += spd_error * Step
	signal = spd_Kp * spd_error + spd_Ki * spd_integral

	print("signal", signal)
	print("spd_integral", spd_integral)
	print("spd_error", spd_error)

	X = X + V * Step
	V = V + signal / M * Step

	x.append(X)
	v.append(V)

#plot.plot(t,x,t,v)
#plot.plot(t,v)
#plot.show()
