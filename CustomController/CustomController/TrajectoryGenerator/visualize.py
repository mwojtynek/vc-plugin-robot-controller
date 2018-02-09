from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy.polynomial.polynomial import polyval
import matplotlib.pyplot as plot

data = np.loadtxt('path.txt',skiprows=1)

with open('path.txt', 'r') as f:
	first_line = f.readline()
	names = first_line.split(" ")
length = len(names)
for i in range(length - 1):
	plot.plot(data[:,0],data[:,i+1], label= names[i+1])
plot.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plot.show()