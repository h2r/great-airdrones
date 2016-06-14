import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt

data = genfromtxt("_slash_vrpn_client_node_slash_ardrone_slash_pose.csv",
        delimiter=(','))

rectifiedZ = data[:, 11] - data[0, 11]

time = data[:, 4] + data[:, 5] * 1e-9

print("Average = %f" % (np.average(rectifiedZ)))
print("Standard Deviation = %f" % (np.std(rectifiedZ)))

plt.plot(time, rectifiedZ)
plt.show()
