from numpy import pi, cos, sin, arccos, arange
import mpl_toolkits.mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as pp
import numpy as np

num_pts = 50
# num_pts = 1000
indices = arange(0, num_pts, dtype=float) + 0.5

fig = pp.figure()
ax = fig.add_subplot(111, projection='3d')

phi = arccos(1 - 2*indices/num_pts)
theta = pi * (1 + 5**0.5) * indices

x_sampled, y_sampled, z_sampled = cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi);

# pp.figure().add_subplot(111, projection='3d').scatter(x_sampled, y_sampled, z_sampled);
# fig.add_subplot(111, projection='3d').scatter(x_sampled, y_sampled, z_sampled);
# pp.show()

# Make data for the circle 
u = np.linspace(0, 2 * np.pi, 50)
v = np.linspace(0, np.pi, 50)
x = 0.98 * np.outer(np.cos(u), np.sin(v))
y = 0.98 * np.outer(np.sin(u), np.sin(v))
z = 0.98 * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot the surface
ax.plot_surface(x, y, z, color='r', cstride=2, rstride=2)
ax.scatter(x_sampled, y_sampled, z_sampled)

print("x_sampled ", x_sampled)
print("y_sampled ", y_sampled)
print("z_sampled ", z_sampled)

pp.show()