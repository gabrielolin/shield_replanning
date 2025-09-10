import os
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import mujoco
from mujoco import MjModel, MjData
import numpy as np
import mujoco.viewer
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from dm_control.utils.inverse_kinematics import qpos_from_site_pose
from dm_control.utils.transformations import euler_to_quat
from dm_control import mujoco as dm_mj
#import transforms3d as tf
import re
import random
from testbed_compute_projectile import computeParabolaEqn
#from ruckig import InputParameter, Ruckig, Trajectory, Result

qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -1.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 1.0071, 6.9813])

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
    mujoco.mju_negPose(T_W_A_pos_inv, T_W_A_quat_inv, T_W_A_pos, T_W_A_quat)

    T_A_B_pos = np.zeros(3)
    T_A_B_quat = np.zeros(4)
    mujoco.mju_mulPose(T_A_B_pos, T_A_B_quat, T_W_A_pos_inv, T_W_A_quat_inv, T_W_B_pos, T_W_B_quat)
    return T_A_B_pos, T_A_B_quat

def getSiteTransformWRTWorld(name, model, data):
    id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)  
    T_W_site_pos = data.xpos[id]
    T_W_site_quat = np.zeros(4)
    mujoco.mju_mat2Quat(T_W_site_quat, data.xmat[id])
    return T_W_site_pos, T_W_site_quat
          
def transform_matrix(translation, quaternion):
    # Convert quaternion to rotation matrix
    #rotation_matrix = tf.quaternions.quat2mat(quaternion)[:3, :3]

    # Create a 4x4 homogeneous transformation matrix
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = translation

    return transform

def check_projectile_collision(id1,id2):
    """Return True if a contact between projectile and shield is detected."""
    #print(data.ncon)
    for i in range(data.ncon):
      # Note that the contact array has more than `ncon` entries,
      # so be careful to only read the valid entries.
      contact = data.contact[i]
      #print('contact', i)
      if(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom[0]) == "projectile" or mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom[1]) == "projectile"):
         print("geom1", mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom[0]))
         print("geom2", mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom[1]))

def getProjectileDiscretizationPts(
    projectile_vel_x,
    projectile_vel_y,
    projectile_vel_z, 
    projectile_start_3d_x, 
    projectile_start_3d_y, 
    projectile_start_3d_z):

    time_interval = 0.004

    sx = []
    sy = []
    sz = []

    t = 0.0

    while t < 1.5:
        height_check = projectile_start_3d_z + projectile_vel_z*t - (1.0/2)*9.81*t*t
        if (height_check > -0.05):
            sx.append(projectile_start_3d_x + projectile_vel_x*t + (1.0/2)*0.0*t*t)
            sy.append(projectile_start_3d_y + projectile_vel_y*t + (1.0/2)*0.0*t*t)
            sz.append(height_check)
        t += time_interval
    
    #print("computed parabola ")
    #for i in range(len(sx)):
        #print(sx[i], sy[i], sz[i])
    return sx,sy,sz


def load_traj_from_txt(filename):
  try:
    # Load the matrix from the text file
    matrix = np.loadtxt(filename)
    return matrix

  except Exception as e:
    print(f"Error: {e}")
    return None

def get_files_in_folder(folder_path):
    try:
        # Initialize an empty list to store file names
        files = []
        
        # Iterate through all the files in the folder
        for root, dirs, files_list in os.walk(folder_path):
            for file in files_list:
                # Get the full file path
                file_path = os.path.join(root, file)
                files.append(file_path)

        # Define a sorting function that extracts numbers from filenames
        def extract_number(file_path):
            file_name = os.path.basename(file_path)  # Extract file name only
            match = re.search(r'\d+', file_name)  # Extract first number in filename
            return int(match.group()) if match else float('inf')  # Convert to int, or place non-numbered files at the end

        # Sort files based on the extracted number
        sorted_files = sorted(files, key=extract_number)

        return sorted_files

    except Exception as e:
        print(f"Error: {e}")
        return None
    
def get_intercept_point(projectile):
    x,y,z = projectile[0:3]
    vx,vy,vz = projectile[3:6]
    sx,sy,sz = getProjectileDiscretizationPts(vx,vy,vz,x,y,z)
    sx = np.array(sx).reshape(len(sx),1)
    sz = np.array(sz).reshape(len(sz),1)
    a,b,c = computeParabolaEqn(sx,sz)
    a, b, c = a.item(), b.item(), c.item()
    x_i = 1 # intercept plane in world frame
    z_i = a*x_i**2+b*x_i+c
    y_i = y # y is const
    slope = 2*a + 2*b
    theta = np.arctan(slope)
    world_quat = euler_to_quat([0, theta-np.pi/2, 0])
    world_pos = np.array([x_i,y_i,z_i])
    return world_pos, world_quat

def get_intercept_point_from_dynamics(projectile, x_target=1.2, gravity=9.81):
    x0, y0, z0 = projectile[0:3]
    vx, vy, vz = projectile[3:6]

    if vx == 0:
        raise ValueError("vx = 0 → projectile will never reach x = {:.2f}".format(x_target))

    # Time at which x = x_target
    t_i = (x_target - x0) / vx

    # Intercept y and z positions at that time
    y_i = y0 + vy * t_i
    z_i = z0 + vz * t_i - 0.5 * gravity * t_i**2

    # Slope of z with respect to x at that point (dz/dx = dz/dt ÷ dx/dt)
    vz_i = vz - gravity * t_i
    dz_dx = vz_i / vx

    theta = np.arctan(dz_dx)
    quat = euler_to_quat([0, theta - np.pi / 2, 0])
    pos = np.array([x_target, y_i, z_i])

    return pos, quat



def get_qpos_from_proj(projectile, dm_model):
    world_pos, world_quat = get_intercept_point_from_dynamics(projectile)
    res = qpos_from_site_pose(dm_model,"shield_site",world_pos[:3], world_quat)
    res2 = qpos_from_site_pose(dm_model,"shield_site",world_pos[:3])
    print("joint config for projectile",res.qpos[:6])
    print("success: ", res.success)
    print("error: ", res.err_norm)
    if res.success:
        return np.clip(res.qpos[:6], qmin,qmax), res.success
    else: 
        return np.clip(res2.qpos[:6], qmin,qmax), res2.success

def get_qpos_from_proj_noisy(projectile, dm_model, error):
    world_pos, world_quat = get_intercept_point(projectile)
    world_pos = world_pos + error
    res = qpos_from_site_pose(dm_model,"shield_site",world_pos[:3], world_quat)
    print("joint config for projectile",res.qpos[:6])
    print("success: ", res.success)
    print("error: ", res.err_norm)
    return np.clip(res.qpos[:6], qmin,qmax)

def get_qpos_from_point(position):
    res = qpos_from_site_pose(dm_model,"shield_site",position[:3])
    return np.clip(res.qpos[:6], qmin,qmax), res.success

def replan(vel_lim,acc_lim,jerk_lim,data,vel,acc,target_pos,target_vel):
    inp = InputParameter(6)
 
    inp.current_position=data.qpos[:6]
    inp.current_velocity = vel
    inp.current_acceleration = acc
 
    inp.target_position = target_pos
    inp.target_velocity = target_vel
    
    inp.max_velocity = vel_lim
    inp.max_acceleration = acc_lim
    inp.max_jerk = jerk_lim

    otg = Ruckig(6)
    trajectory = Trajectory(6)
    result = otg.calculate(inp, trajectory)
    if result == Result.ErrorInvalidInput:
        raise Exception('Invalid input!')
    print(f'Trajectory duration: {trajectory.duration:0.4f} [s]')
    new_traj = []
    duration = int(trajectory.duration/0.004)
    for i in range(duration):
        waypoint = np.hstack(trajectory.at_time(i*0.004)[:3])
        new_traj.append(waypoint)
    return np.array(new_traj)

# Load MuJoCo Model
"""
model_dir = '/home/golin/Research/mujoco_ws/abb/irb_1600/'
mjcf = 'irb1600_6_12_shield_projectile.xml'
#mjcf = 'irb1600_6_12_realshield.xml'
model = mujoco.MjModel.from_xml_path(os.path.join(model_dir, mjcf))
dm_model = dm_mj.Physics.from_xml_path(os.path.join(model_dir, mjcf))
data = MjData(model)
nq = 6
dt = 4e-3

# Velocity and Acceleration Limits
vel_limits = np.array([2.618,2.7925,2.967,5.585,6.9813,7.854])*1.5
acc_limits = np.array([50,50,50,50,50,50])
jerk_limits = np.array([1e3,1e3,1e3,1e3,1e3])

data.qpos[:6] =np.array([-0.5236, 0, 0, 0, 0, 0])
# Compute forward kinematics (updates positions, velocities, and other kinematic properties)
mujoco.mj_forward(model, data)
# Get the body ID of the end-effector
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_6") 
base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base") 
# Retrieve the world-frame position of the end-effector
last_ee_pos = data.xpos[ee_id]
#print("End-effector position:", last_ee_pos)
base_pos_world = data.xpos[base_id]  # Base position in world frame
# Compute EE position relative to base
ee_pos_base = last_ee_pos - base_pos_world
#print("End-effector position (base):", ee_pos_base)

# Get projectile body id using the new interface
proj_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "projectile")
shield_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "shield")
link2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_1")
# Get the starting index for the projectile's free joint:
proj_jntadr = model.body_jntadr[proj_id]
proj_dofadr = model.body_dofadr[proj_id]

proj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../logs/replan_projs_old.txt")
proj_traj = np.loadtxt(proj_path, dtype=float)
rand_int = random.randint(0, len(proj_traj) - 1)
projectile = proj_traj[rand_int]
print(rand_int)

T_W_Base = getSiteTransformWRTWorld('base')
T_Base_W_pos = np.zeros(3)
T_Base_W_quat = np.zeros(4)
mujoco.mju_negPose(T_Base_W_pos, T_Base_W_quat, T_W_Base[0], T_W_Base[1])
T_Base_W_mat = transform_matrix(T_Base_W_pos, T_Base_W_quat)
T_W_Base_mat_p = transform_matrix(T_W_Base[0], T_W_Base[1])
T_W_Base_mat_v = transform_matrix(np.zeros((3)), T_W_Base[1])
proj_p_wrt_world = np.matmul(T_W_Base_mat_p, np.concatenate((projectile[:3], [1])))[:3]
proj_v_wrt_world = np.matmul(T_W_Base_mat_v, np.concatenate((projectile[3:], [1])))[:3]
proj_pv_wrt_world = np.concatenate((proj_p_wrt_world, proj_v_wrt_world))
print("base wrt world: ", base_pos_world)
print("base to world mat: ", T_Base_W_mat)

motion  = False
replanned=0
qpos = get_qpos_from_proj(projectile,  dm_model)
error = np.array([0.0, 0.0, 0.0])  # Original array
error[2] = -0.5
error[1] = -0.1
qpos_noisy = get_qpos_from_proj_noisy(projectile,dm_model,error)
optimal_replan_states = np.loadtxt("../optimal_states.txt") 
#qpos_noisy_optimized = np.concatenate((optimal_replan_states[rand_int][:3], np.zeros(3)))
#qdot_noisy_optimized = np.concatenate((optimal_replan_states[rand_int][3:6], np.zeros(3)))
qpos_mean = np.array([-7.37242802e-01,-1.79983689e-01, -4.01748320e-01, -7.86235816e-04
 -3.91122494e-02,  7.06452372e-17])



if __name__ == '__main__':
    #traj = replan(vel_limits,acc_limits,jerk_limits,data,np.zeros(6),np.zeros(6),qpos_noisy_optimized,qdot_noisy_optimized)
    traj = replan(vel_limits,acc_limits,jerk_limits,data,np.zeros(6),np.zeros(6),qpos_noisy,np.zeros(6))
    data.qpos[:6] =np.array([-0.5236, 0, 0, 0, 0, 0])
    data.qpos[proj_jntadr:proj_jntadr+7] = np.array([*projectile[0:3], 1.0, 0.0, 0.0, 0.0])
    data.qvel[proj_dofadr:proj_dofadr+6] = np.array([*projectile[3:6], 0.0, 0.0, 0.0])
    with mujoco.viewer.launch_passive(model, data) as viewer:
        time.sleep(5)
        start_time = data.time
        while viewer.is_running():
            T_W_proj_pos = data.qpos[6:9]
            proj_p_wrt_base = np.matmul(T_Base_W_mat, np.concatenate((T_W_proj_pos, [1])))[:3]
            #print(proj_p_wrt_base[0])
            for i in range(8):
                mujoco.mj_step(model, data)
                time.sleep(model.opt.timestep)

            check_projectile_collision(link2_id,proj_id)

            if not motion:
                data.qpos[:6] = traj[0][:6]
                if (data.time-start_time) >= 0.2564285:    #proj_p_wrt_base[0] < 4.5:
                    motion = True
                    motion_start = data.time
            else:
                if(replanned == 0): 
                    idx = min(len(traj)-1,int((data.time-motion_start)// 0.004))
                    data.qpos[:6] = traj[idx][:6] 
                    if(data.time-start_time) >= 0.6064285:                                         #proj_p_wrt_base[0] < 2.1):
                        traj_replan = replan(vel_limits,acc_limits,jerk_limits,data,traj[idx][6:12],traj[idx][12:18],qpos,np.zeros(6))
                        replanned = 1
                        replan_start = data.time
                        #print(traj_replan[0][:6])
                        #print(traj[idx][:6])
                        print("traj 2")
                elif(replanned ==1):
                    idx_replan = min(len(traj_replan)-1,int((data.time-replan_start)// 0.004))
                    data.qpos[:6] = traj_replan[idx_replan][:6]
    
            mujoco.mj_kinematics(model, data)
            viewer.sync()
"""