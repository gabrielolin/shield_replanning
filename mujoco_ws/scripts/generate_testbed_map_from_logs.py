import numpy as np
import matplotlib
matplotlib.use("agg")
import matplotlib.pyplot as plt

# import pandas as pd

# exectution time file name 
execution_times_logs = "../logs/testbed_data_10-11-2022_15-53-55.txt"
# line segment count logs 
line_segment_count_logs = "../logs/testbed_ls_count_10-11-2022_15-53-55.txt"
# execution time limit for heatmap generation 
time_limit = 5.0
# number of coloumns 
num_coloumns = 9

valid_plans_count_list = []
valid_plans_avg_time_list = []

ls_count_data = np.genfromtxt(line_segment_count_logs, delimiter=',')
line_counter = 0

with open(execution_times_logs) as f:
    lines=f.readlines()
    for line in lines:
        myarray = np.fromstring(line, dtype=float, sep=',')
        print("myarray ", myarray)
        data_size = myarray.shape[0] - 3
        data_with_total = myarray[-data_size:]
        print("data_with_total ", data_with_total)
        data = data_with_total[0:-1]
        print("data ", data)
        # total_ls = len(data)
        total_ls = ls_count_data[line_counter, -1]
        print("total_ls = ", total_ls)
        valid_plans_count = np.where(data < time_limit)[0].shape[0]
        print("valid_plans_count ", valid_plans_count)
        valid_plans_count_list.append(valid_plans_count)
        if (total_ls == 0):
            valid_plans_avg_time_list.append(-1.0)
        else:
            valid_plans_avg_time_list.append(valid_plans_count/total_ls)
        line_counter += 1
        
        
# print(valid_plans_count_list)
# data_last_col = np.array(valid_plans_count_list)

print("valid_plans_avg_time_list ", valid_plans_avg_time_list)
data_last_col = np.array(valid_plans_avg_time_list)

print("sum(data_last_col) ", sum(data_last_col)/len(data_last_col))

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
plt.savefig("testbed_"+str(time_limit)+"_avg_heatmap.png")
# plt.show()
