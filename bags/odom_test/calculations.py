import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import genfromtxt

# Read data
raw_odom = genfromtxt("_slash_ardrone_slash_odometry.csv", delimiter=(','))
vrpn = genfromtxt("_slash_ardrone_slash_true_position.csv", delimiter=(','))

odom_x = raw_odom[:, 11]
odom_y = raw_odom[:, 12]
odom_z = raw_odom[:, 13]

vrpn_x = vrpn[:, 9]
vrpn_y = vrpn[:, 10]
vrpn_z = vrpn[:, 11]

# Zero data
odom_x = [x - odom_x[0] for x in odom_x]
odom_y = [y - odom_y[0] for y in odom_y]
odom_z = [z - odom_z[0] for z in odom_z]

vrpn_x = [x - vrpn_x[0] for x in vrpn_x]
vrpn_y = [y - vrpn_y[0] for y in vrpn_y]
vrpn_z = [z - vrpn_z[0] for z in vrpn_z]

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(odom_x, odom_y, odom_z)
ax.scatter(vrpn_x, vrpn_y, vrpn_z, c="r")

# Calc Error

plt.show()
