import numpy as np
import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt
import pdb
# import pandas as pd

# line segment count logs 
line_segment_count_logs = "../logs/testbed_ls_count_10-11-2022_15-53-55.txt"

# number of coloumns 
num_coloumns = 9

ls_count_data = np.genfromtxt(line_segment_count_logs, delimiter=',')

data_last_col = ls_count_data[:,-1]

print("sum(data_last_col) ", sum(data_last_col)/len(data_last_col))
# pdb.set_trace()
data_reshape = np.reshape(data_last_col, (num_coloumns, -1))
# data_reshape = np.reshape(data_last_col, (-1, num_coloumns))
print("data_reshape ", data_reshape)

data_rt = np.transpose(data_reshape)
print("data_rt ", data_rt)

data_ud = np.flipud(data_rt)
print("data_ud")
print(data_ud)

plt.imshow(data_ud, cmap='hot', interpolation='nearest')
plt.colorbar()

plt.ylabel('z axis')
ax = plt.gca()
ax.axes.xaxis.set_ticks([])
ax.axes.yaxis.set_ticks([])
# Show the plot.
# plt.savefig("execution_"+str(time_limit)+"_avg_heatmap.png")
plt.savefig("ls_count_heatmap.png")
# plt.show()
