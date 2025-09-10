import numpy as np
import os
import mujoco
from mujoco import MjModel, MjData
import time
import math
import tkinter
from mpl_toolkits import mplot3d
from dm_control.utils.inverse_kinematics import qpos_from_site_pose
from dm_control import mujoco as dm_mj
import transforms3d as tf
import re
import random
from testbed_compute_projectile import computeParabolaEqn
from scipy.optimize import differential_evolution
from ruckig import InputParameter, Ruckig, Trajectory, Result
import matplotlib
matplotlib.use("TkAgg", force=True)
import matplotlib.pyplot as plt

"""
# --- Surrogate time-to-go function for a single double integrator ---
def T_double_int(x0, v0, xf, vf, amax):
    # Shift coordinates relative to target (xf, vf)
    x_rel = x0 - xf
    v_rel = v0 - vf
    
    sigma = np.sign(x_rel + np.sign(v_rel)*(v_rel**2)/(2*amax))
    lambd = np.sqrt(np.abs(sigma * x_rel + (v_rel**2)/(2*amax)))

    T = (2 * lambd)/np.sqrt(amax) + (sigma * v_rel)/amax
    return T

def cost_function(decision_state, goal_states, amax=1.0, time_goal_thresh=0.1, init_time_thresh=0.05, penalty_weight=100.0):
    q, qdot = decision_state[:3], decision_state[3:]

    total_cost = 0.0

    # Goal reachability cost
    for goal_q in goal_states:
        joint_times = [T_double_int(q[j], qdot[j], goal_q[j], 0, amax) for j in range(3)]
        max_joint_time = max(joint_times)
        excess_time = max(0, max_joint_time - time_goal_thresh)
        total_cost += excess_time**2

    # Decision state distance cost (from initial 0,0)
    joint_times_from_start = [T_double_int(0, 0, q[j], qdot[j], amax) for j in range(3)]
    max_time_from_start = max(joint_times_from_start)

    if max_time_from_start < init_time_thresh:
        penalty = (init_time_thresh - max_time_from_start)**2
        total_cost += penalty_weight * penalty  # Heavily penalized

    return total_cost
"""



def replan_cost(vel_lim,acc_lim,jerk_lim,cur_pos, cur_vel,target_pos,target_vel):
    inp = InputParameter(6)
 
    #inp.current_position = data.qpos[:6]
    inp.current_position=cur_pos
    inp.current_velocity = cur_vel
 
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
    return trajectory.duration, trajectory

def sample_points_on_ellipse(n_points, a=0.05, b=0.5):
    """
    Generates points uniformly distributed along the boundary of an ellipse defined by:
      (y/a)^2 + (z/b)^2 = 1

    Parameters:
      n_points: int, number of points to sample
      a: float, semi-axis length along y (default 0.05)
      b: float, semi-axis length along z (default 0.2)

    Returns:
      points: numpy array of shape (n_points, 2), where each row is [y, z].
    """
    # Generate n_points random angles uniformly between 0 and 2π
    angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    
    # Compute the corresponding (y, z) points on the ellipse perimeter
    y = a * np.cos(angles)
    z = b * np.sin(angles)
    
    # Combine y and z into a (n_points x 2) array
    points = np.column_stack((y, z))
    return points


def sample_points_in_ellipse(n_points, a=0.05, b=0.2):
    """
    Generates points uniformly distributed inside an ellipse defined by:
      (y/a)^2 + (z/b)^2 <= 1

    Parameters:
      n_points: int, number of points to sample
      a: float, semi-axis length along y (default 0.05)
      b: float, semi-axis length along z (default 0.2)

    Returns:
      points: numpy array of shape (n_points, 2), where each row is [y, z].
    """
    # Sample angles uniformly between 0 and 2*pi
    angles = np.random.uniform(0, 2*np.pi, n_points)
    # Sample radii with correct weighting (sqrt transformation gives uniform area distribution)
    radii = np.sqrt(np.random.uniform(0, 1, n_points))
    
    # Compute y and z coordinates by scaling the unit circle to the ellipse
    y = a * radii * np.cos(angles)
    z = b * radii * np.sin(angles)
    
    # Combine y and z into a (n_points x 2) array.
    points = np.column_stack((y, z))
    return points

def objective(decision_var, reachability_samples):
    """
    decision_var: 12D array where decision_var[:6] is position and decision_var[6:] is velocity.
    reachability_samples: a list (or array) of reachability samples. 
    Returns:
      Total penalty cost: for each sample, if replan_cost > 0.4, add quadratic penalty or if cost from start > 0.3, add quadratic penalty
    """
    position = decision_var[:6]
    velocity = decision_var[6:]
    total_penalty = 0.0
    for sample in reachability_samples:
        cost = replan_cost(vel_limits,acc_limits,jerk_limits,position, velocity, sample, np.zeros(6))[0]
        cost_reachable = replan_cost(vel_limits,acc_limits,jerk_limits,[-0.5236, 0, 0, 0, 0, 0], np.zeros(6), position, velocity)[0]
        if cost > 0.4:
            total_penalty += (cost - 0.4) ** 2
        if cost_reachable > 0.3:
            total_penalty += (cost_reachable-0.3) ** 2
    return total_penalty

def get_qpos_from_proj_noisy(projectile, dm_model, error):
    world_pos =  world_pos = get_intercept_point(projectile) + error
    res = qpos_from_site_pose(dm_model,"shield_site",world_pos[:3])
    print("joint config for projectile",res.qpos[:6])
    print("success: ", res.success)
    print("error: ", res.err_norm)
    return res.qpos[:6], res.success


model_dir = '/home/golin/Research/mujoco_ws/abb/irb_1600/'
mjcf = 'irb1600_6_12_shield_projectile.xml'
model = mujoco.MjModel.from_xml_path(os.path.join(model_dir, mjcf))
dm_model = dm_mj.Physics.from_xml_path(os.path.join(model_dir, mjcf))

from projectile_test import get_qpos_from_proj, get_intercept_point

proj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../logs/replan_projs.txt")
projectiles = np.loadtxt(proj_path, dtype=float)
ik_samples = []
points = sample_points_on_ellipse(4)
#for proj in projectiles:
#    reachability_samples = [get_qpos_from_proj_noisy(proj, dm_model, [0, point[0], point[1]]) for point in points]
 #   ik_samples.append(reachability_samples)


vel_limits = np.array([2.618,2.7925,2.967,5.585,6.9813,7.854])
acc_limits = np.array([60,60,60,60,60,60])*0.8
jerk_limits = np.array([1e3,1e3,1e3,1e3,1e3])

qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])
# Combine bounds: first positions then velocities
lower_bounds = np.concatenate([qmin, dqmin])
upper_bounds = np.concatenate([qmax, dqmax])
# Create the list of tuples for differential evolution
bounds = list(zip(lower_bounds, upper_bounds))

if __name__ == '__main__':
    uncertainty_ik = []
    idx = 8
    uncertainty = sample_points_on_ellipse(20)
    noisy_intercept_point = get_intercept_point(projectiles[idx])
    for error in uncertainty:
        res = get_qpos_from_proj_noisy(projectiles[idx],dm_model,[0, error[0], error[1]])
        if res[1] == True:
            uncertainty_ik.append(res[0])
    predicted_ik = [get_qpos_from_proj(projectiles[idx],dm_model)]
    mean_uncertainty = np.mean(uncertainty_ik, axis=0)
    predicted_ik = np.array(predicted_ik).flatten()

    print("mean",mean_uncertainty)
    print("predicted",predicted_ik)
    """
    nominal_pos = get_qpos_from_proj(projectiles[0],dm_model)
    nominal_traj = replan_cost(vel_limits,acc_limits,jerk_limits,[-0.5236, 0, 0, 0, 0, 0], np.zeros(6), nominal_pos, np.zeros(6))[1]
    pos_guess = nominal_traj.at_time(0.3)[0][:6]
    vel_guess = nominal_traj.at_time(0.3)[1][:6]

    result = differential_evolution(
    func=lambda x: objective(x, ik_samples[0]),
    bounds=bounds,
    strategy='best1bin',   # optional: choose your strategy
    maxiter=10,          # optional: maximum iterations
    popsize=15,            # optional: population size multiplier
    tol=1e-6,              # optional: convergence tolerance
    mutation=(0.5, 1),     # optional: mutation constant range
    recombination=0.7,     # optional: recombination constant
    disp=True              # optional: print progress
)"
"   

    print("Best solution (12D decision variable):", result.x)
    print("Objective function value:", result.fun)"
    """
    # Number of points to generat

    # Plot the points to visualize the ellipse
    plt.figure()
    plt.scatter(noisy_intercept_point[1]+uncertainty[:, 0], noisy_intercept_point[2]+uncertainty[:, 1], s=10, alpha=0.5)
    plt.plot(noisy_intercept_point[1], noisy_intercept_point[2], 'ro', markersize=5, label='Intercept Point')  # red circle
    plt.xlabel('y')
    plt.ylabel('z')
    plt.title('Uncertainty in task space intercept location')
    plt.axis('equal')

    joint_indices = np.arange(6)  # indices 0 to 5 for 6 joints

    plt.figure(figsize=(8, 4))

    # Plot horizontal line for mean uncertainty per joint
    for i in joint_indices:
        plt.hlines(y=mean_uncertainty[i], xmin=i-0.4, xmax=i+0.4, color='blue', linewidth=2, label='Mean IK' if i == 0 else '')

    # Plot horizontal line for predicted configuration per joint
    for i in joint_indices:
        plt.hlines(y=predicted_ik[i], xmin=i-0.4, xmax=i+0.4, color='red', linestyles='dashed', linewidth=2, label='Predicted IK' if i == 0 else '')

    plt.xticks(joint_indices, [f'Joint {i+1}' for i in joint_indices])
    plt.xlabel('Joint')
    plt.ylabel('Joint Angle (rad)')
    plt.title('Mean Joint Configuration within Uncertainty Bounds vs. Predicted Configuration')
    plt.legend()
    plt.grid(True)
    plt.show()
    