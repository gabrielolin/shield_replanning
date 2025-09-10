
import os
import mujoco as mp
from mujoco import MjData, MjModel
import mujoco_viewer
from time import sleep
import numpy as np
from numpy import genfromtxt
import re

planner_name = 'epase'
# planner_name = 'rrtc'
# planner_name = 'ours'

model_dir = '../../../parallel_search/third_party/mujoco-2.3.2/model/abb/irb_1600'
mjcf = 'irb1600_6_12_realshield_obs.xml'
mjcf_arm = 'irb1600_6_12_shield.xml'
traj_dir = '../logs/trajs_' + planner_name + '_full'

# just using arm model for calculating ee traj
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)

arm_model.opt.timestep = 4e-3
# arm_model.time_step = 4e-3
nq = arm_model.nq

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


def balltraj(end_pos, len, max_len, dx):
  xy_end = np.array([end_pos[0], end_pos[1], 0])
  u = end_pos/np.linalg.norm(xy_end)
  ball_traj = np.tile(end_pos, [max_len, 1])
  for i in range(len):
    ball_traj[i] = end_pos + (len-i)*u*dx
  quat_traj = np.tile([1,0,0,0], [max_len, 1])
  ball_traj = np.hstack((ball_traj, quat_traj))
  return ball_traj


ins = 48
files = get_files_in_folder(traj_dir)
# print(files[ins])

rms_torq = np.zeros((len(files), nq))
sk = []

if files is not None:
  time = np.zeros((0))
  j=0
  for f in files:
    way_pts_raw = load_traj_from_txt(f)
    q = way_pts_raw[:, 1:1+nq]
    dq = way_pts_raw[:, 1+nq:1+2*nq]
    ddq = way_pts_raw[:, 1+2*nq:1+3*nq]

    tau = np.zeros(np.shape(q))
    for i in range(np.shape(q)[0]):
      arm_data.qpos[:] = q[i, :]
      arm_data.qvel[:] = dq[i, :]
      arm_data.qacc[:] = ddq[i, :]

      mp.mj_inverse(arm_model, arm_data)
      tau[i, :] = arm_data.qfrc_inverse[:]
    rmst = np.sqrt(np.mean(tau ** 2, axis=0))
    if np.any(rmst > 1000):
      skip_id = last_number = re.findall(r'\d+$', f)[-1]
      sk.append(int(skip_id))
      continue
    rms_torq[j, :] = rmst
    j=j+1

print(planner_name)
print(sk)
print("mean")
mean_rms_torq = np.mean(rms_torq, axis=0)
print(mean_rms_torq)
print("std")
std_rms_torq = np.std(rms_torq, axis=0)
print(std_rms_torq)
print("median")
median_rms_torq = np.median(rms_torq, axis=0)
print(median_rms_torq)

