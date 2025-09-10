
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time
import os

ta.setup_logging("INFO")

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

def upsampleTraj(traj, dx=0.05):
    col = np.shape(traj)[1]
    uptraj = np.empty((0, col))
    for i in range(np.shape(traj)[0]-1):
      if np.array_equal(traj[i,:], -1*np.ones((traj.shape[1],))):
        uptraj = np.append(uptraj, traj[i,:][np.newaxis,:], axis=0)
        continue
      if np.array_equal(traj[i+1,:], -1*np.ones((traj.shape[1],))):
        continue

      x1 = traj[i,:]
      x2 = traj[i+1,:]
      dist = np.linalg.norm(x2-x1)
      N = int(dist/dx)
      samp = np.linspace(x1, x2, N)
      uptraj = np.append(uptraj, samp, axis=0)

    return uptraj

def db():
    import pdb
    pdb.set_trace()
    
def update_time(time, planner):
    inv_ind = [2, 3, 4, 11, 12, 13, 14, 15, 16, 18, 21, 28, 31, 32, 34, 36, 37, 39, 40, 41, 43, 45, 46, 49, 51, 52, 53, 57, 59, 61, 62, 63, 64, 65, 67, 71, 74, 75, 77, 78, 79, 88, 89, 92, 96, 98, 99, 100, 102, 104, 106, 107, 109, 110, 111, 113, 114, 116, 119, 130, 132, 134, 135, 136, 139, 140, 143, 144, 147, 149, 151, 152, 154, 155, 156, 158, 160, 162, 163, 164, 166, 168, 169, 171, 172, 175, 176, 178, 180, 183, 184, 186, 190, 191, 193, 195, 204, 206, 209, 211, 215, 216, 218, 220, 221, 223, 224, 227, 228, 232, 235, 236, 238, 240, 243, 245, 247, 248, 249, 250, 252, 254, 257, 259, 260, 264, 265, 266, 267, 268, 270, 271, 273, 275, 279, 280, 281, 282, 288, 291, 293, 298, 299, 300, 309, 310, 311, 312, 313, 314, 315, 316, 319, 322, 323, 325, 327, 328, 329, 332, 333, 336, 338, 340, 341, 342, 345, 348, 350, 356, 357, 358, 359, 360, 361, 362, 365, 368, 370, 372, 380, 381, 384, 385, 387, 389, 390, 394, 396, 397, 399]
    inv_ind = [num - 1 for num in inv_ind]
    rname =  "../logs/exp_"+planner+".txt"
    wname =  "../logs/exp_"+planner+"_full.txt"
    m = load_traj_from_txt(rname)
    t = 0
    for ind, r in enumerate(m):
        if r[0] != -1 and (ind not in inv_ind):
            m[ind][2] = time[t]
            t = t + 1
    np.savetxt(wname, m)


def profile_traj(planner):
    vlims = np.array([2.618,2.7925,2.961,5.585,6.9813,7.854])*2
    alims = np.array([60,60,60,60,60,60])
    inv_ind = [2, 3, 4, 11, 12, 13, 14, 15, 16, 18, 21, 28, 31, 32, 34, 36, 37, 39, 40, 41, 43, 45, 46, 49, 51, 52, 53, 57, 59, 61, 62, 63, 64, 65, 67, 71, 74, 75, 77, 78, 79, 88, 89, 92, 96, 98, 99, 100, 102, 104, 106, 107, 109, 110, 111, 113, 114, 116, 119, 130, 132, 134, 135, 136, 139, 140, 143, 144, 147, 149, 151, 152, 154, 155, 156, 158, 160, 162, 163, 164, 166, 168, 169, 171, 172, 175, 176, 178, 180, 183, 184, 186, 190, 191, 193, 195, 204, 206, 209, 211, 215, 216, 218, 220, 221, 223, 224, 227, 228, 232, 235, 236, 238, 240, 243, 245, 247, 248, 249, 250, 252, 254, 257, 259, 260, 264, 265, 266, 267, 268, 270, 271, 273, 275, 279, 280, 281, 282, 288, 291, 293, 298, 299, 300, 309, 310, 311, 312, 313, 314, 315, 316, 319, 322, 323, 325, 327, 328, 329, 332, 333, 336, 338, 340, 341, 342, 345, 348, 350, 356, 357, 358, 359, 360, 361, 362, 365, 368, 370, 372, 380, 381, 384, 385, 387, 389, 390, 394, 396, 397, 399]
    folderName = "../logs/trajs_" + planner +"/"
    files = get_files_in_folder(folderName)
    max_velos = np.zeros(6)
    max_accels = np.zeros(6)
    if files is not None:
        # print(files[0])
        time = np.zeros((0))

        for f in files:
        # if True:
            # f = files[48]
            print(f)
            traj_f = f.split("/")[-1]
            traj_n = traj_f.split("_")[-1]
            if int(traj_n) in inv_ind:
                print("Continuing")
                continue

            way_pts_raw = load_traj_from_txt(f)

            _, ind = np.unique(way_pts_raw, axis=0, return_index=True)
            way_pts = way_pts_raw[np.sort(ind)]
            # db()
            # ss = way_pts[:,-1]
            way_pts = way_pts[:,:-1]
            way_pts = upsampleTraj(way_pts)
            ss = np.linspace(0, 1, way_pts.shape[0])
            ################################################################################
            # Define the geometric path and two constraints.
            path = ta.SplineInterpolator(ss, way_pts)
            pc_vel = constraint.JointVelocityConstraint(vlims)
            pc_acc = constraint.JointAccelerationConstraint(alims)


            instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeSpline")
            jnt_traj = instance.compute_trajectory()
            ################################################################################
            # The output trajectory is an instance of
            # :class:`toppra.interpolator.AbstractGeometricPath`.
            # ts_sample = np.linspace(0, jnt_traj.duration, 100)
            ts_sample = np.arange(0, jnt_traj.duration, 0.004)
            np.append(ts_sample, jnt_traj.duration)
            qs_sample = jnt_traj(ts_sample)
            qds_sample = jnt_traj(ts_sample, 1)
            qdds_sample = jnt_traj(ts_sample, 2)
            max_velos = np.maximum(max_velos, np.abs(qds_sample))
            max_accels = np.maximum(max_accels, np.abs(qdds_sample))
            save = np.concatenate((ts_sample.reshape(ts_sample.shape[0],-1), qs_sample, qds_sample, qdds_sample), axis=1)
            save_f = f.replace(planner, (planner+"_full"))
            time = np.append(time, jnt_traj.duration)
            np.savetxt(save_f, save)

            #db()
            '''            fig, axs = plt.subplots(4, 1, sharex=True)
            for i in range(path.dof):
                # plot the i-th joint trajectory
                axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
                axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
                axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
                axs[3].plot(ss, way_pts[:, i], c="C{:d}".format(i))
            axs[2].set_xlabel("Time (s)")
            axs[0].set_ylabel("Position (rad)")
            axs[1].set_ylabel("Velocity (rad/s)")
            axs[2].set_ylabel("Acceleration (rad/s2)")
            plt.show()
            #db()'
            '''
        print(np.mean(time))

        update_time(time, planner)

        ################################################################################
        # Optionally, we can inspect the output.
        # instance.compute_feasible_sets()
        # instance.inspect()
        # db()
    print("max velos",max_velos)
    print("max accels",max_accels)



if __name__ == "__main__":
    planner = "ours"
    profile_traj(planner)