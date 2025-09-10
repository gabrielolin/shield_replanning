# Import libraries
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd 
data_dict = {}
 
# data_dict['category'] = ['MoveIt','SMPL']
# data_dict['lower'] = [0.0073,0.015]
# data_dict['upper'] = [0.054,0.077]
# plt.title("Repeatibility test - error in end effector 3D position (in meters)")

# data_dict['category'] = ['MoveIt','SMPL']
# data_dict['lower'] = [0.301,0.572]
# data_dict['upper'] = [0.324,0.577]
# plt.title("Difference between profiler time and full trajectory execution time (in seconds)")

# data_dict['category'] = ['MoveIt','SMPL']
# data_dict['lower'] = [-0.78,-0.19]
# data_dict['upper'] = [-0.67,-0.12]
# plt.title("Difference between SMPL profiler/Moveit Exec (top) and Moveit profiler/SMPL Exec (bottom) (in seconds)")

# data_dict['category'] = ['MoveIt','SMPL']
# data_dict['lower'] = [0.03,0.07]
# data_dict['upper'] = [0.04,0.09]
# plt.title("End effector position error with respect to goal FK (in meters)")

# data_dict['category'] = ['MoveIt','SMPL']
# data_dict['lower'] = [-0.058,0.133]
# data_dict['upper'] = [-0.054,0.135]
# plt.title("Difference between profiler time and time taken during execution to reach within 0.05m of goal in EE space (in meters)")


dataset = pd.DataFrame(data_dict)
for lower,upper,y in zip(dataset['lower'],dataset['upper'],range(len(dataset))):
    plt.plot((lower,upper),(y,y),'ro-',color='orange')
plt.yticks(range(len(dataset)),list(dataset['category']))

plt.show()