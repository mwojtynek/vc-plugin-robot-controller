from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy.polynomial.polynomial import polyval
import matplotlib.pyplot as plot
import os.path

current = 1;

while True:
	path = 'trajectory/path'+str(current)+'.txt'
	if not os.path.isfile(path):
		break

	data = np.loadtxt(path,skiprows=1)
	
	with open(path, 'r') as f:
		first_line = f.readline()
		names = first_line.split(" ")

	
	plot.figure(current)
	plot.gcf().canvas.set_window_title("Joint " + str(current))

	length = len(names)
	for i in range(length - 1):
		plot.plot(data[:,0],data[:,i+1], label= names[i+1])
	plot.legend(bbox_to_anchor=(-0.05, 1.015, 1.1, .102), ncol=4, mode="expand", borderaxespad=0.)
	current = current + 1
plot.show()