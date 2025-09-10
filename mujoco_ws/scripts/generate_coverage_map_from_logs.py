import numpy as np
import matplotlib
import matplotlib.pyplot as plt
# import pandas as pd

# exectution time file name 
execution_times_logs = "../logs/execution_time_31-01-2023_23-27-57.txt"
# line segment count logs 
line_segment_count_logs = "../logs/ls_count.txt"
# execution time limit for heatmap generation 
time_limit = 5.0
# number of coloumns 
num_coloumns = 9

valid_plans_count_list = []
valid_plans_avg_time_list = []

ls_count_data = np.genfromtxt(line_segment_count_logs, delimiter=',')
line_counter = 0

total_ls_sum = 0
all_times = []
with open(execution_times_logs) as f:
    lines=f.readlines()
    for line in lines:
        print(line)
        if not line.strip():
            continue
        myarray = np.fromstring(line, dtype=float, sep=',')
        print("myarray ", myarray)
        data_size = myarray.shape[0] - 3
        data_with_total = myarray[-data_size:]
        print("data_with_total ", data_with_total)
        total_ls = ls_count_data[line_counter, -1]
        total_ls_sum = total_ls_sum + total_ls
        # total_ls = data_with_total[-1]
        data = data_with_total[0:-1]
        all_times.extend(list(data))
        print("data ", data)
        valid_plans_count = np.where(data < time_limit)[0].shape[0]
        print("valid_plans_count ", valid_plans_count)
        print("total_ls = ", total_ls)
        valid_plans_count_list.append(valid_plans_count)
        valid_plans_avg_time_list.append(valid_plans_count/total_ls)
        line_counter += 1
        # import ipdb; ipdb.set_trace()

print("mean time", np.mean(all_times))
print("std time", np.std(all_times))
# print(valid_plans_count_list)
# data_last_col = np.array(valid_plans_count_list)
print("Total coverage ", sum(valid_plans_count_list)/total_ls_sum)
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
plt.clim(0,1)

plt.ylabel('z axis')
ax = plt.gca()
ax.axes.xaxis.set_ticks([])
ax.axes.yaxis.set_ticks([])

for i in range(data_ud.shape[0]):
    for j in range(data_ud.shape[1]):
        text = ax.text(j, i, "{:.2f}".format(data_ud[i,j]),
                       ha="center", va="center", color="g")
# Show the plot.
# plt.savefig("execution_"+str(time_limit)+"_avg_heatmap.png")
plt.savefig("coverage_"+str(time_limit)+"_avg_heatmap.png")
# plt.show()
