import os
import mujoco
from mujoco import MjModel, MjData
import numpy as np
import mujoco.viewer
import time

def load_traj_from_txt(filename):
  try:
    # Load the matrix from the text file
    matrix = np.loadtxt(filename)
    return matrix

  except Exception as e:
    print(f"Error: {e}")
    return None

def controller(model,data,q_ref,dq_ref,Kp,Kd):
    position_error = data.qpos[:6]-q_ref
    velocity_error = data.qvel[:6]-dq_ref
    torque_cmd = -Kp*position_error -Kd*data.qvel[:6]
    data.ctrl[:6]=torque_cmd

def interpolate(t, time_stamps, data_array):
    """Linear interpolation of trajectory data."""
    if t <= time_stamps[0]: return data_array[0]
    if t >= time_stamps[-1]: return data_array[-1]
    
    i = np.searchsorted(time_stamps, t) - 1
    alpha = (t - time_stamps[i]) / (time_stamps[i+1] - time_stamps[i])
    return (1-alpha)*data_array[i] + alpha*data_array[i+1]

def get_files_in_folder(folder_path):
  try:
    # Initialize an empty list to store file names
    files_with_mtime = []
    # Iterate through all the files in the folder
    for root, dirs, files in os.walk(folder_path):
      for file in files:
        # Get the full file path
        file_path = os.path.join(root, file)
        # Get the modification time of the file
        modified_time = os.path.getmtime(file_path)
        # Append the tuple (file_name, modified_time) to the list
        files_with_mtime.append((file_path, modified_time))
    # Sort the list of tuples by modified time
    files_with_mtime.sort(key=lambda x: x[1])
    # Extract the file names from the sorted list
    sorted_files = [file_mtime[0] for file_mtime in files_with_mtime]
    return sorted_files
  except Exception as e:
    print(f"Error: {e}")
    return None

# Load MuJoCo Model
model_dir = '/home/golin/Research/mujoco_ws/abb/irb_1600/'
mjcf_arm = 'irb1600_6_12_realshield_actuated.xml'
model = mujoco.MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
data = MjData(model)
nq = 6

traj_dir = '../logs/trajs_ours'
files = get_files_in_folder(traj_dir)
way_pts_raw = load_traj_from_txt(files[0])
time_stamps = 5*way_pts_raw[:, 6]  # Extract time values (7th column)
traj = way_pts_raw[:, 0:6]  # Extract joint positions (6 DOF)
traj_vel = np.gradient(traj, axis=0) / np.gradient(time_stamps, axis=0)[:, None]  # Compute velocity from time
traj_acc = np.gradient(traj_vel, axis=0) / np.gradient(time_stamps, axis=0)[:, None]  # Compute velocity from time

# Velocity and Acceleration Limits
vel_limits = np.array([2.618,2.7925,2.961,5.585,6.9813,7.854])
acc_limits = np.array([50,50,50,50,50,50])


Kp = np.diag([500, 500, 500, 500, 500, 500])  # Stiffness gains
Kd = np.diag([20, 20, 20, 20, 20, 20])        # Damping gains

for i in range(6):
   model.actuator_gainprm[i,0]=1

current_time = 0.0
dt = model.opt.timestep  # Simulation timestep
time_index = 0  # Start at the first waypoints
real_time_start = time.time()
sim_time_start = data.time
# Simulation loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        #if current_time > time_stamps[-1]:
           #break
        q_ref = interpolate(current_time, time_stamps, traj)
        dq_ref = interpolate(current_time, time_stamps, traj_vel)
        ddq_ref = interpolate(current_time, time_stamps, traj_acc)

        q = np.array(data.qpos[:6])
        dq = np.array(data.qvel[:6])

        # 3. Compute tracking error
        e_q = q - q_ref
        e_dq = dq - dq_ref

        CG= data.qfrc_bias  # Coriolis & centrifugal forces and Gravity compensation

        M = np.zeros((6, 6))
        mujoco.mj_fullM(model, M, data.qM)  # Get Mass Matrix

        # Apply control
        tau = M @ (-Kp @ e_q - Kd @ e_dq + ddq_ref) + CG
        print(current_time)
        data.ctrl[:] = tau

        # Forward simulation step
        mujoco.mj_step(model, data)
        sim_time_elapsed = data.time - sim_time_start
        real_time_elapsed = time.time() - real_time_start
        # If simulation is ahead of real time, pause
        if sim_time_elapsed > real_time_elapsed:
            time.sleep(sim_time_elapsed - real_time_elapsed)

        # Update time
        current_time += dt
        # Render frame
        viewer.sync()

