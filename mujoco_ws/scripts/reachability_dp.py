import math
from collections import deque, defaultdict
import numpy as np
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
from ruckig import InputParameter, Ruckig, Trajectory, Result
import os

def build_forward_adjacency(
    pos_vals, vel_vals, acc_vals, dt
):
    """
    Build a dictionary that for each (p_idx, v_idx) stores:
    [(next_p_idx, next_v_idx), (next_p_idx, next_v_idx), ...]
    for all feasible accelerations a in acc_vals.

    pos_vals, vel_vals are lists (e.g. discrete positions/velocities).
    acc_vals is a list of possible accelerations.
    dt is the time-step (float or 1).
    """
    adjacency = defaultdict(list)
    
    # Precompute to map from (p_idx, v_idx) -> (p, v)
    # and also back from (p, v) -> (p_idx, v_idx)
    pos_index = {p: i for i, p in enumerate(pos_vals)}
    vel_index = {v: i for i, v in enumerate(vel_vals)}
    
    num_p = len(pos_vals)
    num_v = len(vel_vals)

    for i, p in enumerate(pos_vals):
        for j, v in enumerate(vel_vals):
            # For each possible acceleration, compute next state
            for a in acc_vals:
                p_next = p + v * dt + 0.5 * a * (dt**2)
                v_next = v + a * dt
                # Check if next state is within bounds
                if pos_vals[0] <= p_next <= pos_vals[-1] and vel_vals[0] <= v_next <= vel_vals[-1]:
                    # Snap to nearest grid index
                    # (assuming pos_vals, vel_vals are uniform grids)
                    # If you have a uniform grid, you can do something like:
                    # i_next = round((p_next - p_min)/pos_step)
                    # j_next = round((v_next - v_min)/vel_step)
                    # For simplicity, let's do a "find closest" approach:
                    i_next = min(range(num_p), key=lambda x: abs(pos_vals[x] - p_next))
                    j_next = min(range(num_v), key=lambda y: abs(vel_vals[y] - v_next))
                    
                    adjacency[(i, j)].append((i_next, j_next))
    return adjacency


def reverse_adjacency(forward_adj):
    """
    Given a forward adjacency dict: node -> [neighbors],
    produce a reverse adjacency that for each node, 
    tells which nodes can go forward into it (parents).
    """
    rev_adj = defaultdict(list)
    for src, nbrs in forward_adj.items():
        for dst in nbrs:
            rev_adj[dst].append(src)
    return rev_adj


def bfs_shortest_steps(start_nodes, adjacency, max_steps=999999):
    """
    Standard BFS on a graph where edges have cost=1 step.
    Returns a dict: dist[node] = number of steps from the set of start_nodes to reach 'node'.
    If a node is unreachable, it won't appear in dist (or we can store dist[node] = inf).
    
    start_nodes: list of nodes (or a single node).
    adjacency: dict[node] -> list of neighbors
    """
    dist = {}
    queue = deque()
    
    # Initialize the queue with start_nodes at distance 0
    for s in start_nodes:
        dist[s] = 0
        queue.append(s)
    
    while queue:
        current = queue.popleft()
        cur_d = dist[current]
        # If we've reached max_steps, we can skip going further
        if cur_d >= max_steps:
            continue
        
        # Explore neighbors
        for nxt in adjacency[current]:
            if nxt not in dist:  # not visited
                dist[nxt] = cur_d + 1
                queue.append(nxt)
    
    return dist


def find_best_intermediate_state(
    start, goals, dist_from_start, dist_to_goal,
    G, T
):
    """
    Among all states in the grid, find (p^*, v^*) that:
      - is reachable from start in <= G steps
      - can reach as many goals as possible in <= T steps each
    If no state can reach all N goals, pick the one with the highest coverage (# of goals).
    Ties can be broken arbitrarily or by secondary criteria.

    Inputs:
    - start: node index (p_idx, v_idx) for the start
    - goals: list of node indices for the N goals
    - dist_from_start: dictionary with BFS distances from the start node
    - dist_to_goal: list of dictionaries, each is BFS distances "to" a single goal
                    (i.e., dist_to_goal[i][node] = steps from node to goal i).
                    This can be computed by BFS in the reverse adjacency from goal i.
    - G, T: time-step constraints (integers).
    """
    best_node = None
    best_coverage = -1
    
    all_nodes = set(dist_from_start.keys())
    # Potentially, you might want all nodes in the entire graph, 
    # but we only consider nodes that are at least reachable from start
    # within <= G, because if it's not reachable or exceeds G, no point checking.

    for node in all_nodes:
        if dist_from_start[node] <= G:
            # Check how many goals are reachable within T from this node
            coverage = 0
            for g_idx, goal_dist_map in enumerate(dist_to_goal):
                d = goal_dist_map.get(node, math.inf)
                if d <= T:
                    coverage += 1
            
            if coverage > best_coverage:
                best_coverage = coverage
                best_node = node
    
    return best_node, best_coverage


def compute_replan_state(goal_list,start,Tf,Tb, bounds):
    # ------------------------------
    # 1) Define your discrete state space
    # ------------------------------
    p_min, p_max = bounds[0][0], bounds[0][1]
    v_min, v_max = bounds[1,0], bounds[1][1]
    # step sizes for position and velocity
    p_step = 0.1
    v_step = 0.1
    
    # Build lists of discrete positions/velocities
    pos_vals = [p_min + i * p_step for i in range(int((p_max - p_min) / p_step) + 1)]
    vel_vals = [v_min + j * v_step for j in range(int((v_max - v_min) / v_step) + 1)]
    num_p = len(pos_vals)
    num_v = len(vel_vals)

    # Discrete accelerations
    a_min, a_max = -60, 60
    acc_vals = [a_min, 0, a_max]
    
    dt = 0.01  # time step in seconds (for discrete updates)
    
    # ------------------------------
    # 2) Build adjacency (forward) for BFS
    # ------------------------------
    forward_adj = build_forward_adjacency(pos_vals, vel_vals, acc_vals, dt)
    
    # Build reverse adjacency (so BFS from goals can easily compute "dist_to_goal")
    rev_adj = reverse_adjacency(forward_adj)
    

    # ------------------------------
    # 3) Define start & goals
    #    (We pick some arbitrary examples)
    # ------------------------------
    start_p, start_v = start[0],start[1]
    i_p_start = min(range(num_p), key=lambda x: abs(pos_vals[x] - start_p))
    i_v_start = min(range(num_v), key=lambda y: abs(vel_vals[y] - start_v))
    start_node = (i_p_start,i_v_start)
    # Suppose we have 3 goal states

    i_p_goals = [min(range(num_p), key=lambda x: abs(pos_vals[x] - goal)) for goal in goal_list]
    i_v_goals = [min(range(num_v), key=lambda y: abs(vel_vals[y] - 0)) for goal in goal_list]

    goals = [(i_p_goal, i_v_goal) for i_p_goal, i_v_goal in zip(i_p_goals, i_v_goals)]

    """
    goals = [
        (pos_index[5.0], vel_index[0.0]),
        (pos_index[-3.0], vel_index[1.0]),
        (pos_index[8.0], vel_index[-1.0])
    ]
    """
    # ------------------------------
    # 4) BFS from the start in the forward adjacency
    #    to get dist_from_start[node]
    # ------------------------------
    dist_from_start = bfs_shortest_steps([start_node], forward_adj, max_steps=9999)
    
    # ------------------------------
    # 5) BFS from each goal in reverse adjacency
    #    to get dist_to_goal[i][node] (distance from node to goal_i)
    # ------------------------------
    dist_to_goal = []
    for g in goals:
        dist_map = bfs_shortest_steps([g], rev_adj, max_steps=9999)
        dist_to_goal.append(dist_map)
    
    # ------------------------------
    # 6) We define G and T
    #    and search for the best intermediate node
    # ------------------------------
    G = Tf  # must reach the candidate node within 5 steps from the start
    T = Tb # must reach each goal from the candidate node within 5 steps (to count it)
    
    candidate_node, coverage = find_best_intermediate_state(
        start_node, goals, dist_from_start, dist_to_goal, G, T
    )
    
    # ------------------------------
    # 7) Print results
    # ------------------------------
    if candidate_node is None:
        print("No node is reachable from start within G steps. No solution found.")
    else:
        cp_idx, cv_idx = candidate_node
        cp = pos_vals[cp_idx]
        cv = vel_vals[cv_idx]
        print("Best intermediate node found:")
        print(f"  Position = {cp}, Velocity = {cv}")
        print(f"  Coverage = {coverage} out of {len(goals)} goals (within T steps)")
        print(f"  Steps from start to node: {dist_from_start[candidate_node]}")
        return cp,cv
        # If coverage < len(goals), it's a "least violating" solution
        # because we can't satisfy all goals within T steps from any single node.


if __name__ == "__main__":
    qmin = np.array([-3.14159, -1.0995, -4.1015, -3.4906, -2.0071, -6.9813])
    qmax  = np.array([3.14159, 1.9198, 0.9599, 3.4906, 2.0071, 6.9813])
    dqmin = np.array([-2.618, -2.7925, -2.967, -5.585, -6.9813, -7.854])
    dqmax = np.array([2.618, 2.7925, 2.967, 5.585, 6.9813, 7.854])
    accelerations = np.array([-60,60,60])

    model_dir = '/home/golin/Research/mujoco_ws/abb/irb_1600/'
    mjcf = 'irb1600_6_12_shield_projectile.xml'
    model = mujoco.MjModel.from_xml_path(os.path.join(model_dir, mjcf))
    dm_model = dm_mj.Physics.from_xml_path(os.path.join(model_dir, mjcf))

    from projectile_test import get_intercept_point, get_qpos_from_point
    from prefix_opt import sample_points_on_ellipse
    proj_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../logs/replan_projs.txt")
    projectiles = np.loadtxt(proj_path, dtype=float)
    points = sample_points_on_ellipse(20)
    noisy_intercept_points = []
    replan_states = []
    start = np.array([[-0.5236, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])

    for proj in projectiles:
        noisy_intercept = get_intercept_point(proj)-np.array([0,0,-0.2])
        noisy_intercept_points.append(noisy_intercept)
        reachability_samples = []
        for point in points:
            res = get_qpos_from_point(noisy_intercept+[0,point[0],point[1]])
            if res[1] == True:
                reachability_samples.append(res[0])
        reachability_samples = np.array(reachability_samples)
        replan_state = []
        for i in range(3):
            bounds = np.array([[qmin[i],qmax[i]],[dqmin[i],dqmax[i]]])
            q,qdot = compute_replan_state(reachability_samples[:,i],start[:,i],0.3,0.3,bounds)
            replan_state.append([q, qdot])
        replan_state = np.array(replan_state).T
        replan_states.append(replan_state)

    # Flatten each 2x6 array into a 1x12 row
    flattened_states = np.array([state.flatten() for state in replan_states])
    # Save to text file
    np.savetxt("../logs/replan_states.txt", flattened_states, fmt="%.6f")

    print(replan_states)

    

