import csv
import numpy as np
import matplotlib.pyplot as plt

def to_np(arr):
    return np.array(arr)

kpx = []
kpy = []
kdx = []
kdy = []
x_variance = []
y_variance = []
x_max = []
y_max = []
x_min = []
y_min = []
x_mean = []
y_mean = []
x_std = []
y_std = []

f = open("logfile1.csv")
csv_f = csv.reader(f)

for row in csv_f:
    kpx.append(float(row[0]))
    kpy.append(float(row[1]))
    kdx.append(float(row[2]))
    kdy.append(float(row[3]))
    x_variance.append(float(row[4]))
    y_variance.append(float(row[5]))
    x_max.append(float(row[6]))
    y_max.append(float(row[7]))
    x_min.append(float(row[8]))
    y_min.append(float(row[9]))
    x_mean.append(float(row[10]))
    y_mean.append(float(row[11]))
    x_std.append(float(row[12]))
    y_std.append(float(row[13]))

kpx = to_np(kpx)
kpy = to_np(kpy)
kdx = to_np(kdx)
kdy = to_np(kdy)
x_variance = to_np(x_variance)
y_variance = to_np(y_variance)
x_max = to_np(x_max)
y_max = to_np(y_max)
x_min = to_np(x_min)
y_min = to_np(y_min)
x_mean = to_np(x_mean)
y_mean = to_np(y_mean)
x_std = to_np(x_std)
y_std = to_np(y_std)

x = kpx * 10 + np.abs(kdx)
y = kpy * 10 + np.abs(kdy)

y_variance_line = plt.plot(y, y_variance, label="y_variance")

y_max_line = plt.plot(y, y_max, label="y_max")

y_min_line = plt.plot(y, y_min, label="y_min")

y_mean_line = plt.plot(y, y_mean, label="y_mean")

y_std_line = plt.plot(y, y_std, label="y_std")

plt.legend(bbox_to_anchor=(0, 0), loc='upper left', ncol=1)
plt.grid(True)

print "y_std"
print np.argmin(np.abs(y_std))

print y[np.argmin(np.abs(y_std))]

plt.show()
