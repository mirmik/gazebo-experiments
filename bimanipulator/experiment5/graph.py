#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy

def plot_arg(arg, i, j):
	print (arg)
	manip1_file = open(arg + "/manip1.txt")
	manip1_forces_text = manip1_file.readlines()
	manip1_forces = [ float(s.strip()) for s in manip1_forces_text ]

	manip2_file = open(arg + "/manip2.txt")
	manip2_forces_text = manip2_file.readlines()
	manip2_forces = [ float(s.strip()) for s in manip2_forces_text ]

	cargo_file = open(arg + "/cargo.txt")
	cargo_forces_text = cargo_file.readlines()
	cargo_forces = [ float(s.strip()) for s in cargo_forces_text ]
	n = len(manip1_forces_text)

	t = numpy.array(range(n))*0.001

	START = 13000
	FINISH = 13250

	plt.subplot(2,2,i)
	plt.plot(t[START:FINISH], manip1_forces[START:FINISH], 'k', linewidth=1)

	plt.subplot(2,2,j)
	plt.plot(t[START:FINISH], cargo_forces[START:FINISH], 'k', linewidth=1)

plot_arg("0.01", 1, 3)
plot_arg("0.001", 2, 4)
#plot_arg("0.0001", 5, 7)
#plot_arg("0.00001", 6, 8)

plt.show()
