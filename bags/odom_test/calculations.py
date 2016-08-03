import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt

# Read data
raw_odom = genfromtxt("_slash_ardrone_slash_odometry.csv", delimiter=(','))
vrpn = genfromtxt("_slash_ardrone_slash_true_position.csv", delimiter=(','))

odom_time = raw_odom[:, 4]
odom_x = raw_odom[:, 11]
odom_y = raw_odom[:, 12]
odom_z = raw_odom[:, 13]

vrpn_time = vrpn[:, 4]
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

odom_x = np.array(odom_x)
odom_y = np.array(odom_y)
odom_z = np.array(odom_z)

odom_dist = odom_x * odom_x + odom_y * odom_y + odom_z * odom_z
odom_dist = np.sqrt(odom_dist)

vrpn_x = np.array(vrpn_x)
vrpn_y = np.array(vrpn_y)
vrpn_z = np.array(vrpn_z)

vrpn_dist = vrpn_x * vrpn_x + vrpn_y * vrpn_y + vrpn_z * vrpn_z
vrpn_dist = np.sqrt(vrpn_dist)

print vrpn_time
print odom_time

diff = []

for i in range(0, len(odom_time)):
    found = False
    for j in range(0, len(vrpn_time)):
        if odom_time[i] == vrpn_time[j] and not found:
            diff.append(abs(odom_dist[i] - vrpn_dist[j]))
            found = True

print np.var(diff)


# Plotting
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(odom_x, odom_y, odom_z)
# ax.scatter(vrpn_x, vrpn_y, vrpn_z, c="r")

# Calc Error

plt.plot(diff)

plt.show()
