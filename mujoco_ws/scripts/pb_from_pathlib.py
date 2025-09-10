#! /usr/bin/env python
import os
import random

import roslib
import rospy
import actionlib
import pickle
import numpy as np

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

nq = 6
# dt = 4e-3
# inv_dt = 15e-3
dt = 1e-2
inv_dt = 4e-2

# Directory containing your files
directory = '/home/shield/backup/insat_logs_1obs_wide_nz/logs/paths_library/'

# Prefix to filter files
prefix = 'path_'

# List all files in the directory that start with the specified prefix
matching_files = [file for file in os.listdir(directory) if file.startswith(prefix)]

# Check if there are any matching files
if not matching_files:
    print(f"No files found in {directory} starting with '{prefix}'.")
else:
    # Select a random file from the matching files
    # random_file = random.choice(matching_files)
    random_file = 'path_2248'

    # Load the file as a NumPy matrix, excluding the first line
    file_content = np.loadtxt(os.path.join(directory, random_file), skiprows=1)

    # Now you have the content of the random file in the 'file_content' variable
    print(f"Random file selected: {random_file}")


goal = FollowJointTrajectoryGoal()
goal_traj = JointTrajectory()
goal_traj.header.frame_id = 'odom_combined'
goal_traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

j = 0
for r in file_content:
    p = JointTrajectoryPoint()
    p.time_from_start.secs = int(j*dt)
    p.time_from_start.nsecs = int(rospy.Time.from_sec(j*dt).to_nsec() % 1e9)
    p.positions = [0,0,0,0,0,0]

    for i in range(nq):
        p.positions[i] = r[i]
    goal_traj.points.append(p)
    j+=1

goal.trajectory = goal_traj

inv_file_content = file_content[::-1]
home = FollowJointTrajectoryGoal()
home_traj = JointTrajectory()
home_traj.header.frame_id = 'odom_combined'
home_traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

j = 0
for r in inv_file_content:
    p = JointTrajectoryPoint()
    p.time_from_start.secs = int(j*inv_dt)
    p.time_from_start.nsecs = int(rospy.Time.from_sec(j*inv_dt).to_nsec() % 1e9)
    p.positions = [0,0,0,0,0,0]

    for i in range(nq):
        p.positions[i] = r[i]
    home_traj.points.append(p)
    j+=1

home.trajectory = home_traj

# print(goal)

if __name__ == '__main__':
    rospy.init_node('pb_from_pathlib')
    client = actionlib.SimpleActionClient('egm/joint_velocity_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    # client = actionlib.SimpleActionClient('egm/joint_position_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration.from_sec(1.0)):
        print("joint_trajectory_action server not available")
    print("Connected to follow_joint_trajectory server")

    client.send_goal_and_wait(goal)
    input("Hit enter to fuck more")    
    client.send_goal_and_wait(home)
