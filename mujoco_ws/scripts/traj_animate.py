#! /usr/bin/env python

import os
import mujoco as mp
import mujoco.viewer
from mujoco import MjData, MjModel
import mujoco.viewer
from time import sleep
import numpy as np
import collections
from dm_control.utils.inverse_kinematics import qpos_from_site_pose
from dm_control import mujoco
import pickle
import math
import time
import transformations as tf
import pdb

# ROS
import rospy
import rospkg
import actionlib
# from pydrake.all import BsplineTrajectory, KinematicTrajectoryOptimization, Solve
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState

sim = True

VIS = False

# model
model_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"abb/irb_1600/")
if VIS:
    mjcf = 'irb1600_6_12_realshield_obs_vis.xml'
else:
    mjcf = 'irb1600_6_12_realshield_obs.xml'

# viewer params
viz_dt = 1

# dm_control
manip_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# calib_target_ori_range = np.array([[np.pi, 0, -np.pi/4], [np.pi/6, np.pi/6, np.pi/6]])
calib_target_ori_range = np.array([[3.098, 0.692, -0.850], [np.pi/6, np.pi/6, np.pi/6]])
model = MjModel.from_xml_path(os.path.join(model_dir, mjcf))
data = MjData(model)
phy = mujoco.Physics.from_xml_path(os.path.join(model_dir, mjcf))

paused = False
def key_callback(keycode):
  if chr(keycode) == ' ':
    global paused
    paused = not paused

if sim:
    viewer = mujoco_viewer.MujocoViewer(model, data)
    # viewer = mp.viewer.launch(model, data)
    # .launch_passive(model, data, key_callback=key_callback)


# planner params
nq = 6
qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])
ddqmin = -10*np.ones(nq)
ddqmax = 10*np.ones(nq)
dddqmin = -100*np.ones(nq)
dddqmax = 100*np.ones(nq)
dt = 4e-3


# ROS subscriber
curr_state = JointTrajectoryControllerState()
curr_state.actual.positions = np.zeros(nq) # initialize with zeros

home_state = np.array([-0.5236, 0, 0, 0, 0, 0])

# Mujoco viewer

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qw, qx, qy, qz]

def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

def findLocalTransform(T_W_A_pos, T_W_A_quat, T_W_B_pos, T_W_B_quat):
    T_W_A_pos_inv = np.zeros(3)
    T_W_A_quat_inv = np.zeros(4)
    mp.mju_negPose(T_W_A_pos_inv, T_W_A_quat_inv, T_W_A_pos, T_W_A_quat)

    T_A_B_pos = np.zeros(3)
    T_A_B_quat = np.zeros(4)
    mp.mju_mulPose(T_A_B_pos, T_A_B_quat, T_W_A_pos_inv, T_W_A_quat_inv, T_W_B_pos, T_W_B_quat)
    return T_A_B_pos, T_A_B_quat

def getSiteTransformWRTWorld(site_name):
    id = mp.mj_name2id(model, mp.mjtObj.mjOBJ_SITE, site_name)
    T_W_site_pos = data.site_xpos[id]
    T_W_site_quat = np.zeros(4)
    mp.mju_mat2Quat(T_W_site_quat, data.site_xmat[id])
    return T_W_site_pos, T_W_site_quat
          
def transform_matrix(translation, quaternion):
    # Convert quaternion to rotation matrix
    rotation_matrix = tf.quaternion_matrix(quaternion)[:3, :3]

    # Create a 4x4 homogeneous transformation matrix
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = translation

    return transform

def parseArgs():
    import argparse
    parser = argparse.ArgumentParser(description='Visualize Trajectory')
    parser.add_argument('--num', type=int, default=1, help='ID of the Traj')
    parser.add_argument('--dir', type=str, default='/home/hanlany/code/mujoco_ws/logs/', help='Directory to load the data')
    parser.add_argument('--ours', type=bool, default=False, help='We have different format for ours/baselines')
    parser.add_argument('--slow', type=int, default=2, help='Slow down the visualization by a factor of x')
    parser.add_argument('--planner', type=str, default='ours')
    args = parser.parse_args()
    return args

def getCommonTraj(args):
    logs_dir = args.dir
    planner = ['epase', 'ours', 'rrtc']
    traj_nums = []
    # Find all the traj num that are common
    for p in planner:
        traj_dir = os.path.join(logs_dir, 'trajs_' + p + '_full')
        traj_files = os.listdir(traj_dir)
        traj_nums.append(set([int(f.split('_')[-1]) for f in traj_files]))
    
    common_traj = traj_nums[0].intersection(traj_nums[1], traj_nums[2])
    
    return np.array([int(t) for t in common_traj])

def initializeData(tpva, proj_pv):
    data.qpos[:6] = tpva[0][1:model.nq+1-7]
    data.qpos[6:9] = proj_pv[0:3]
    # data.qpos[11] = 2.0944
    data.qvel[6:9] = proj_pv[3:6]
    mp.mj_kinematics(model, data)

if __name__ == "__main__":

    # rospy.init_node('traj_vis')
    
    # Load selected Traj to execute
    args = parseArgs()
    
    # File System
    traj_dir = os.path.join(args.dir, 'trajs_' + args.planner + '_full')
    
    # Find common traj
    traj_ind = getCommonTraj(args)
    traj_file = os.path.join(traj_dir, 'traj_' + str(traj_ind[args.num]))
    proj_file = os.path.join(args.dir, 'rand_projs.txt')
    
    print("Pick traj: ", traj_file)
    
    tpva = np.loadtxt(traj_file, dtype=float)
    projs = np.loadtxt(proj_file, dtype=float)
    # for i in range(1, 10):
    #     visualize(tpva, args.slow)
    
    proj_pv = projs[traj_ind[args.num]-1]
    
    # Proj 146
    # proj_pv = np.array([6.20222,3.24538,0,-5.24919,-2.77376,6.26131])
    # proj_pv = np.array([0,5,0,0,-1,6.26131])
    
    # Transform the proj_pv to the base frame
    initializeData(tpva, proj_pv)
    T_W_Base = getSiteTransformWRTWorld('base')
    T_Base_W_pos = np.zeros(3)
    T_Base_W_quat = np.zeros(4)
    mp.mju_negPose(T_Base_W_pos, T_Base_W_quat, T_W_Base[0], T_W_Base[1])
    T_Base_W_mat = transform_matrix(T_Base_W_pos, T_Base_W_quat)
    T_W_Base_mat_p = transform_matrix(T_W_Base[0], T_W_Base[1])
    T_W_Base_mat_v = transform_matrix(np.zeros((3)), T_W_Base[1])
    proj_p_wrt_world = np.matmul(T_W_Base_mat_p, np.concatenate((proj_pv[:3], [1])))[:3]
    proj_v_wrt_world = np.matmul(T_W_Base_mat_v, np.concatenate((proj_pv[3:], [1])))[:3]
    proj_pv_wrt_world = np.concatenate((proj_p_wrt_world, proj_v_wrt_world))


    # import faulthandler; faulthandler.enable()
    # import pdb;
    # pdb.set_trace()
    input_s = "0"
    while input_s != "q":
        initializeData(tpva, proj_pv_wrt_world)
        viewer.render()
        viewer._paused = True
        
        while viewer._paused:
            viewer.render()
        
        start = time.time()
        motion = False
        motion_start = time.time()
        data.qpos[:6] = tpva[-1][1:model.nq+1-7]
        while viewer.is_alive and time.time() - start < 5:
            step_start = time.time()
            # print(step_start - start)
            # Timestep per mj_step is 2ms, to make it 60 fps, step 8 times
            for i in range(8):
                mujoco.mj_step(model, data)
            
            # Calculate the current pose of the proj
            T_W_proj_pos = data.qpos[6:9]
            proj_p_wrt_base = np.matmul(T_Base_W_mat, np.concatenate((T_W_proj_pos, [1])))[:3]
            
            # print("Current Pos: ", proj_p_wrt_base)
            if not motion:
                data.qpos[:6] = tpva[0][1:model.nq+1-7]
                if proj_p_wrt_base[0] < 4.5:
                    motion = True
                    motion_start = time.time()
            else:
                if (step_start - motion_start) > tpva[-1][0]*args.slow:
                    data.qpos[:6] = tpva[-1][1:model.nq+1-7]
                else:
                    data.qpos[:6] = tpva[int((step_start - motion_start)//(dt*args.slow))][1:model.nq+1-7]


            # Sanity check - see if the goal pose will block or not
            # data.qpos[:6] = tpva[-1][1:model.nq+1-7]   

            mp.mj_kinematics(model, data)
                
            viewer.render()
            # Rudimentary time keeping, will drift relative to wall clock.
            # time_until_next_step = 4*dt - (time.time() - step_start)
            # print(time_until_next_step)
            # if time_until_next_step > 0:
            time.sleep((args.slow-1)*4*dt)
        
        input_s = input("Press Q+Enter to quit, Press Enter to continue...")

    # import pdb; pdb.set_trace()



