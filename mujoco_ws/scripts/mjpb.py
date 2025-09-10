
import os
import mujoco as mp
from mujoco import MjData, MjModel
import mujoco.viewer
from time import sleep
import numpy as np
from numpy import genfromtxt

# planner_name = 'test'
# planner_name = 'insat'
# planner_name = 'pinsat'
# planner_name = 'rrt'
# planner_name = 'rrtconnect'
# planner_name = 'rrtc'
planner_name = 'epase'
# planner_name = 'gepase'

# static_planner = True if not (planner_name=='insat' or planner_name=='pinsat' or planner_name=='test') else False
# static_planner = False
static_planner = True

if static_planner:
  # dt = 1e-2
  dt = 0.05
  # dt = 6e-3
else:
  # dt = 6e-3
  dt = 4e-3
  # dt = 0.5
  # dt = 1

model_dir = '/home/golin/Research/mujoco_ws/abb/irb_1600/'
mjcf_arm = 'irb1600_6_12_realshield.xml'
# traj_dir = '../logs/trajs_' + planner_name + '_full'
#traj_dir = '../logs/trajs_' + planner_name

# just using arm model for calculating ee traj
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)
viewer = mujoco.viewer.MujocoViewer(arm_model, arm_data)

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

def upsampleTraj(traj, dx=0.1):
  col = np.shape(traj)[1]
  uptraj = np.empty((0, col))
  for i in range(np.shape(traj)[0]-1):
    if np.array_equal(traj[i,:], -1*np.ones((arm_model.nq,))):
      uptraj = np.append(uptraj, traj[i,:][np.newaxis,:], axis=0)
      continue
    if np.array_equal(traj[i+1,:], -1*np.ones((arm_model.nq,))):
      continue

    x1 = traj[i,:]
    x2 = traj[i+1,:]
    dist = np.linalg.norm(x2-x1)
    N = int(dist/dx)
    samp = np.linspace(x1, x2, N)
    uptraj = np.append(uptraj, samp, axis=0)

  return uptraj

ins = 204
files = get_files_in_folder(traj_dir)
print(files[ins])
way_pts_raw = load_traj_from_txt(files[ins])

if static_planner:
  traj = way_pts_raw[:, :nq]
  traj = upsampleTraj(traj, 1e-2)
else:
  traj = way_pts_raw[:, 1:nq+1]

skp = 0
i=0
while i <= np.shape(traj)[0]:
  if i == np.shape(traj)[0]:
    i=0
    continue
  skp += 1
  # if skp % 2 == 0:
  #   continue

  if np.array_equal(traj[i,:], -1*np.ones((arm_model.nq,))):
    sleep(2)
    i+=1
    continue
  if viewer.is_alive:
    arm_data.qpos[:] = traj[i,:]
    # print(traj[i,:])
    mp.mj_step(arm_model, arm_data)
    viewer.render()
    sleep(dt)
  else:
      break
  i+=1

# close
viewer.close()

