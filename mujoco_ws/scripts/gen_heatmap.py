import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# Define numbers of generated data points and bins per axis.
N_numbers = 100000
N_bins = 100

string = "front"
# string = "right_parallel"

# data = np.loadtxt("../logs/expt_z_tol/matrix_perp_data_"+ string +".txt", delimiter = ",")
# data = np.loadtxt("../logs/dual_planes_exp/matrix_data_"+ string +".txt", delimiter = ",")
data = np.loadtxt("../logs/dual_planes_exp/heatmap_1.000000_sols_logs_23-02-2021_23-59-28.txt", delimiter = ",")
print("data ", data)

data_last_col = data[:, 2]
print("sum(data_last_col) ", sum(data_last_col)/81)

data_reshape = np.reshape(data_last_col, (-1, 9))
print("data_reshape ", data_reshape)

data_rt = np.transpose(data_reshape)
print("data_rt ", data_rt)

data_ud = np.flipud(data_rt)
print("data_ud")
print(data_ud)

plt.imshow(data_ud, cmap='hot', interpolation='nearest')
plt.colorbar()

# Add title and labels to plot.
plt.title(string + "plane - goal vector perpendicular to the line segment")
# plt.title("Heatmap of reachability for "+string+" plane ")
if string == "right":
	plt.xlabel('x axis')
elif string == "front":
	plt.xlabel('y axis')

plt.ylabel('z axis')
ax = plt.gca()
ax.axes.xaxis.set_ticks([])
ax.axes.yaxis.set_ticks([])
# Show the plot.
# plt.savefig("/home/yashoza/Pictures/robocop/perp"+string+"_heatmap.png")
plt.savefig("/home/yashoza/Pictures/robocop/"+string+"_heatmap_1.000000_sols_logs_23-02-2021_23-59-28.png")
# plt.show()