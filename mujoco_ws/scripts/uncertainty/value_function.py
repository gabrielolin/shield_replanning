import numpy as np
import mujoco
from mujoco import MjModel, MjData
from mpl_toolkits import mplot3d
from dm_control.utils.inverse_kinematics import qpos_from_site_pose
from dm_control import mujoco as dm_mj
import transforms3d as tf
import os
from functools import partial
import pickle
from ruckig import InputParameter, Ruckig, Trajectory, Result
from projectile_test import get_intercept_point, get_qpos_from_point
from prefix_opt import sample_points_in_ellipse, replan_cost
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Enables 3D projection

def min_time_ruckig(vel_lim,acc_lim,jerk_lim,cur_pos,cur_vel,cur_accel,target_pos,target_vel,target_accel):
    inp = InputParameter(6)
 
    #inp.current_position = data.qpos[:6]
    inp.current_position=cur_pos
    inp.current_velocity = cur_vel
    inp.current_acceleration = cur_accel
 
    inp.target_position = target_pos
    inp.target_velocity = target_vel
    inp.target_acceleration = target_accel
 
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

def discretize_projectiles():
    """
    Discretizes possible realizations of projectile passing through front plane at x=1 (in world frame)
    Returns grid in x,y,theta
    x: x position world frame
    y: y position world frame
    theta: ccw angle about y axis (determines pose which positions shield normal to proj)
    """
    return None

def compute_pose_from_proj(projectile):
    """
    Calls dm-control IK solver 
    'Introduction to inverse kinematics with jacobian
    transpose, pseudoinverse and damped least squares methods.'

    Local method -> closest configuration to desired pose, may not converge
    """
    return None

def build_intercept_cost_fn():
    """
    Returns:
        intercept_cost_fn(y, z, theta): function that maps any (y, z, theta) → cost
    """

    # Define panel bounds
    panels = [
        {"base_cost": 100.0, "y_center": -0.2,  "z_center": 0.75, "y_half": 0.3,  "z_half": 0.3,  "theta_modulate": True},   # red
        {"base_cost": 100.0, "y_center": -1.3,  "z_center": 0.75, "y_half": 0.3,  "z_half": 0.3,  "theta_modulate": True},   # red
        {"base_cost": 50.0,  "y_center": -0.75, "z_center": 0.75, "y_half": 0.25, "z_half": 0.3,  "theta_modulate": False},  # yellow
    ]

    def intercept_cost_fn(y, z, theta):
        """
        Args:
            y, z, theta: float (world coordinates)
        Returns:
            cost: float
        """
        for panel in panels:
            y_lo = panel["y_center"] - panel["y_half"]
            y_hi = panel["y_center"] + panel["y_half"]
            z_lo = panel["z_center"] - panel["z_half"]
            z_hi = panel["z_center"] + panel["z_half"]

            if y_lo <= y <= y_hi and z_lo <= z <= z_hi:
                base = panel["base_cost"]
                if panel["theta_modulate"]:
                    if theta > 0:
                        mod = 1 + np.log1p(theta)
                    else:
                        mod = 1 + 0.5 * theta
                    mod = max(mod, 0.01)  # always positive
                    return base * mod
                else:
                    return base

        return 0.0  # low cost if no panel is hit

    return intercept_cost_fn


def sample_proj_from_distribution(mean, covariance):
    """
    Samples projectile from gaussian distribution N(mean, variance)
    """
    return None

def plot_cost_surface(cost_fn, y_bounds, z_bounds, theta, y_res=50, z_res=50):
    y_vals = np.linspace(y_bounds[0], y_bounds[1], y_res)
    z_vals = np.linspace(z_bounds[0], z_bounds[1], z_res)
    Y, Z = np.meshgrid(y_vals, z_vals)
    Cost = np.zeros_like(Y)

    for i in range(Z.shape[0]):
        for j in range(Z.shape[1]):
            Cost[i, j] = cost_fn(Y[i, j], Z[i, j], theta)

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot_surface(Y, Z, Cost, cmap='hot', edgecolor='k', linewidth=0.2, alpha=0.9)

    ax.set_title(f"Cost Surface at θ = {theta:.2f} rad")
    ax.set_xlabel("y")
    ax.set_ylabel("z")
    ax.set_zlabel("Cost")

    ax.set_xlim(y_bounds)
    ax.set_ylim(z_bounds)
    ax.set_zlim(bottom=0)

    plt.tight_layout()
    plt.show()

def interactive_cost_3d(cost_fn, y_bounds, z_bounds):
    interact(
        lambda theta: plot_cost_surface(cost_fn, y_bounds, z_bounds, theta),
        theta=(-np.pi / 4, np.pi / 4, 0.05)
    )

qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
vel_limits = np.array([2.618,2.7925,2.967,5.585,6.9813,7.854])
acc_limits = np.array([60,60,60,60,60,60])*0.8
jerk_limits = np.array([1e3,1e3,1e3,1e3,1e3])

start = np.array([[-0.5236, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
cost_fn = build_intercept_cost_fn()
for theta in np.linspace(-np.pi/4, np.pi/4, 10):
    plot_cost_surface(cost_fn, y_bounds=(-2.0, 0.5), z_bounds=(0.0, 2.0), theta=theta)



