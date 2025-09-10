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

import matplotlib.pyplot as plt
from pydrake.all import (
    DiagramBuilder,
    DynamicProgrammingOptions,
    FittedValueIteration,
    LeafSystem,
    LinearSystem,
    LogVectorOutput,
    Simulator,
)

from minimum_time_utils import (
    create_animation,
    simulate_and_plot,
)
def get_closest_meshpoint(goal, mesh):
    """
    Given a goal state and a mesh grid, returns the closest mesh point to the goal.

    Parameters:
        goal (list or np.array): The goal state [q_goal, qdot_goal].
        mesh (dict): A dictionary containing 'q_grid' and 'qdot_grid'.

    Returns:
        closest_point (tuple): The closest mesh point (q_closest, qdot_closest).
    """
    q_closest = mesh["q_grid"][np.argmin(np.abs(mesh["q_grid"] - goal[0]))]
    qdot_closest = mesh["qdot_grid"][np.argmin(np.abs(mesh["qdot_grid"] - goal[1]))]
    goal = np.array([q_closest, qdot_closest])
    return goal 

def get_double_integrator():
    A = np.array([[0, 1], [0, 0]])
    B = np.array([[0], [1]])
    C = np.eye(2)
    D = np.zeros((2, 1))
    return LinearSystem(A, B, C, D)

def cost_function(context,goal):
    # Modify here to get the correct state vector value from context.
    # Hint: Once you get a BasicVector in Drake, then call CopyToVector() to get a
    # numpy array.
    x = context.get_continuous_state_vector().CopyToVector()
    #x = np.round(x)
    #goal = np.array([0,0])
    state_cost = 1
    if np.isclose(np.linalg.norm(x - goal), 0):
        state_cost = 0
    return state_cost 

def run_value_iteration(cost_function, goal, mesh, max_iter=10000):
    # to create an animation, we store the values of
    # the cost to go and the optimal policy for each
    # iteration of the value-iteration algorithm
    J_grid = []
    pi_grid = []

    # callback from the value-iteration algorithm
    # that saves the intermediate values of J and pi
    # and that ensures we do not exceed max_iter
    # (iteration number i starts from 1)
    def callback(i, unused, J, pi):
        # check max iter is not exceeded
        if i > max_iter:
            raise RuntimeError(
                f"Value-iteration algorithm did not converge within {max_iter} iterations."
            )

        # store cost to go for iteration i
        # the 'F' order facilitates the plot phase
        J_grid.append(np.reshape(J, (mesh["n_q"], mesh["n_qdot"]), order="F"))
        pi_grid.append(np.reshape(pi, (mesh["n_q"], mesh["n_qdot"]), order="F"))

    # set up a simulation
    simulator = Simulator(get_double_integrator())

    # grids for the value-iteration algorithm
    state_grid = [set(mesh["q_grid"]), set(mesh["qdot_grid"])]
    input_grid = [set(mesh["u_grid"])]

    # add custom callback function as a visualization_callback
    options = DynamicProgrammingOptions()
    options.visualization_callback = callback
    cost_function_with_goal = partial(cost_function, goal=goal)
    # run value-iteration algorithm
    policy, cost_to_go = FittedValueIteration(
        simulator,
        cost_function_with_goal,
        state_grid,
        input_grid,
        mesh["timestep"],
        options,
    )

    # recast J and pi from lists to 3d arrays
    J_grid = np.dstack(J_grid)
    pi_grid = np.dstack(pi_grid)

    return policy, cost_to_go, J_grid, pi_grid

def setup_and_run_value_iteration(goal,bounds):
    # discretization mesh of state space, input space,
    # and time for the value-iteration algorithm
    mesh = {}

    # number of knot points in the grids
    # odd to have a point in the origin
    mesh["n_q"] = 51  # do not exceed ~51/101
    mesh["n_qdot"] = 51  # do not exceed ~51/101
    mesh["n_u"] = 11  # don't exceed ~11/21

    # grid limits
    mesh["q_lim"] = [bounds[0][0], bounds[0][1]]
    mesh["qdot_lim"] = [bounds[1][0], bounds[1][1]]
    mesh["u_lim"] = [-50.0, 50.0]  # do not change

    
    # axis discretization
    for s in ["q", "qdot", "u"]:
        mesh[f"{s}_grid"] = np.linspace(*mesh[f"{s}_lim"], mesh[f"n_{s}"])

    approximate_goal = get_closest_meshpoint(goal,mesh)

    assert approximate_goal[0] in mesh[f"q_grid"]
    assert approximate_goal[1] in mesh[f"qdot_grid"]

    # time discretization in the value-iteration algorithm
    mesh["timestep"] = 0.005

    policy, cost_to_go, J_grid, pi_grid = run_value_iteration(cost_function,approximate_goal, mesh)
    #animation = create_animation(J_grid, pi_grid, mesh)
    return mesh, J_grid

def optimize_over_grid(J_grid_forward, J_grid_backwards, Q_mesh, T_forward, T_backward):
    """
    Finds the exact optimal (Q, Qdot) by searching over the discretized mesh grid.

    Parameters:
        J_grid_forward (numpy.ndarray): Forward cost-to-go grid.
        J_grid_backwards (numpy.ndarray): Backward cost-to-go grids for each goal.
        Q_mesh (dict): Dictionary containing 'q' and 'qdot' grid arrays.

    Returns:
        tuple: (optimal_Q, optimal_Qdot, min_cost)
    """
    q_grid = Q_mesh["q_grid"]
    qdot_grid = Q_mesh["qdot_grid"]
    
    min_cost = float("inf")
    optimal_Q = None
    optimal_Qdot = None

    # Iterate over all (q, qdot) pairs in the discretized grid
    for q_idx, Q in enumerate(q_grid):
        for qdot_idx, Qdot in enumerate(qdot_grid):
            J_fwd = J_grid_forward[q_idx, qdot_idx]

            # Constraint: J_forward(Q, Qdot) < T_forward
            if J_fwd > T_forward:
                continue  # Skip this state if constraint is violated

            # Compute cost function: sum_i max(J_grid_backwards(Q,Qdot,goal_i) - Tbackward, 0)
            max_terms = [
                max(J_grid_backwards[goal_i, q_idx, qdot_idx] - T_backward, 0)**2
                for goal_i in range(J_grid_backwards.shape[0])
            ]
            cost = sum(max_terms)

            # Update the optimal state
            if cost < min_cost:
                min_cost = cost
                optimal_Q = Q
                optimal_Qdot = Qdot

    return optimal_Q, optimal_Qdot

if __name__ == "__main__":
    qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
    qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
    dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
    dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])

    model_dir = '/home/golin/Research/mujoco_ws/abb/irb_1600/'
    mjcf = 'irb1600_6_12_shield_projectile.xml'
    model = mujoco.MjModel.from_xml_path(os.path.join(model_dir, mjcf))
    dm_model = dm_mj.Physics.from_xml_path(os.path.join(model_dir, mjcf))

    from projectile_test import get_intercept_point, get_qpos_from_point
    from prefix_opt import sample_points_on_ellipse
    proj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../logs/replan_projs.txt")
    projectiles = np.loadtxt(proj_path, dtype=float)
    points = sample_points_on_ellipse(4)
    start = np.array([[-0.5236, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
    start_meshes = [None]*3
    start_J_grids = [None]*3
    goal_meshes = [[[None for _ in range(3)] for _ in range(4)] for _ in range(len(projectiles))]
    goal_j_grids = [[[None for _ in range(3)] for _ in range(4)] for _ in range(len(projectiles))]
    process = False
    if(process):
        for i in range(3):
            bounds = np.array([[qmin[i],qmax[i]],[dqmin[i],dqmax[i]]])
            start_mesh,start_J_grid = setup_and_run_value_iteration(start[:,i],bounds)
            start_meshes[i]=start_mesh
            start_J_grids[i]=start_J_grid

        for proj_idx, proj in enumerate(projectiles):  # Track projectile index
            noisy_intercept = get_intercept_point(proj) - np.array([0, 0, 0.5])
            for goal_idx, point in enumerate(points):  # Track goal index
                res = get_qpos_from_point(noisy_intercept + [0, point[0], point[1]])
                if res[1] == True or res[1] == False:
                    for dof_idx in range(3):  # Iterate over DoFs
                        bounds = np.array([[qmin[dof_idx], qmax[dof_idx]], [dqmin[dof_idx], dqmax[dof_idx]]])
                        # Run value iteration
                        goal_mesh, goal_J_grid = setup_and_run_value_iteration((res[0][dof_idx], 0), bounds)
                        goal_meshes[proj_idx][goal_idx][dof_idx] = goal_mesh
                        goal_j_grids[proj_idx][goal_idx][dof_idx] = goal_J_grid

        with open("value_iteration_data.pkl", "wb") as f:
            pickle.dump((start_meshes, start_J_grids, goal_meshes, goal_j_grids), f)

    with open("value_iteration_data.pkl", "rb") as f:
        (start_meshes_loaded, 
        start_J_grids_loaded, 
        goal_meshes_loaded, 
        goal_j_grids_loaded) = pickle.load(f)


    print("Loaded shapes:", 
        len(start_meshes_loaded),
        len(goal_j_grids_loaded))  
    Tf = 0.4
    Tb = 0.2
    optimal_states = np.zeros((len(projectiles), 6)) 
    for proj_idx,_ in enumerate(projectiles):
        proj_data = goal_j_grids_loaded[proj_idx] 
        for i in range(3):
            mesh = start_meshes_loaded[i]
            J_grid_forward = start_J_grids_loaded[i][:, :, -1]
            #J_grid_backwards = goal_j_grids_loaded[proj_idx, :, i, :, :, -1]
            dof_data = [proj_data[goal_idx][i][:,:,-1] for goal_idx in range(len(proj_data))]
            dof_data = np.stack(dof_data, axis=0)
            J_grid_backwards = dof_data[:,:,:]
            optimal_q, optimal_qdot = optimize_over_grid(J_grid_forward,J_grid_backwards, mesh, Tf, Tb)
            optimal_states[proj_idx, i] = optimal_q       
            optimal_states[proj_idx, i + 3] = optimal_qdot 

    np.savetxt("optimal_states.txt", optimal_states, fmt="%.6f", delimiter=" ")


