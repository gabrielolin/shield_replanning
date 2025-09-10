import numpy as np
import pdb
from collections import defaultdict


def relevant_indices(data, cost_thresh):
    # pdb.set_trace()
    idx = np.array(np.where( (data["success"] == 1) & ((data["time"] <= 6)) & (data['cost'] > cost_thresh) )).squeeze()
    success = np.count_nonzero(data["success"] == 1)/float(data["cost"].shape[0])
    success_idx = np.array(np.where(data["success"] == 1))
    return idx, success, success_idx


def prune_failures(data):
    # pdb.set_trace()
    idx = np.array(np.where( (data["success"] == 1) & ((data["time"] <= 6)))).squeeze()
    success = np.count_nonzero(data["success"] == 1)/float(data["cost"].shape[0])
    success_idx = np.array(np.where(data["success"] == 1))
    return idx, success, success_idx

def print_stats_1(cost_thresh):
    print("\n\n--------------- Cost thresh: {} ---------------".format(cost_thresh))
    wastar = np.loadtxt("../abb_logs/timeout_2/logs_wastar/data_dump.txt")
    epase = np.loadtxt("../abb_logs/timeout_2/logs_wastar/data_dump.txt")

    d = {}

    d['wastar'] = {'id' : wastar[:, 0], 'success' : wastar[:, 1], 'time' : wastar[:, 2], 'cost' : wastar[:, 3], 'state_expansions' : wastar[:, 4], 'edge_expansions' : wastar[:, 5]} 
    d['epase'] = {'id' : epase[:, 0], 'success' : epase[:, 1], 'time' : epase[:, 2], 'cost' : epase[:, 3], 'state_expansions' : epase[:, 4], 'edge_expansions' : epase[:, 5]} 


    wastar_idx, wastar_success, wastar_success_idx = relevant_indices(d['wastar'], cost_thresh)
    epase_idx, epase_success, epase_success_idx = relevant_indices(d['epase'], cost_thresh)

    stats = {'wastar': {}, 'epase': {}}

    stats['wastar']['num_success'] = wastar_idx.shape[0]
    stats['wastar']['success_rate'] = wastar_success
    stats['wastar']['mean_cost'] = np.mean(d['wastar']['cost'][wastar_idx])
    stats['wastar']['mean_time'] = np.mean(d['wastar']['time'][wastar_idx])
    stats['wastar']['max_time'] = np.max(d['wastar']['time'][wastar_idx])
    stats['wastar']['mean_state_expansions'] = np.mean(d['wastar']['state_expansions'][wastar_idx])
    stats['wastar']['mean_edge_expansions'] = np.mean(d['wastar']['edge_expansions'][wastar_idx])

    stats['epase']['num_success'] = epase_idx.shape[0]
    stats['epase']['success_rate'] = epase_success
    stats['epase']['mean_cost'] = np.mean(d['epase']['cost'][epase_idx])
    stats['epase']['mean_time'] = np.mean(d['epase']['time'][epase_idx])
    stats['epase']['max_time'] = np.max(d['epase']['time'][epase_idx])
    stats['epase']['mean_state_expansions'] = np.mean(d['epase']['state_expansions'][epase_idx])
    stats['epase']['mean_edge_expansions'] = np.mean(d['epase']['edge_expansions'][epase_idx])

    for alg, stats in stats.items():
        print("------------------")
        print(alg)
        print("------------------")

        for stat, val in stats.items():
            print('{}: {}'.format(stat, val))


def print_stats_2(cost_thresh):
    print("\n\n--------------- Cost thresh: {} ---------------".format(cost_thresh))
    wastar = np.loadtxt("../abb_logs/timeout_2/logs_wastar/data_dump.txt")
    epase = np.loadtxt("../abb_logs/timeout_2/logs_wastar/data_dump.txt")

    d = {}

    d['wastar'] = {'id' : wastar[:, 0], 'success' : wastar[:, 1], 'time' : wastar[:, 2], 'cost' : wastar[:, 3], 'state_expansions' : wastar[:, 4], 'edge_expansions' : wastar[:, 5]} 
    d['epase'] = {'id' : epase[:, 0], 'success' : epase[:, 1], 'time' : epase[:, 2], 'cost' : epase[:, 3], 'state_expansions' : epase[:, 4], 'edge_expansions' : epase[:, 5]} 
    wastar_idx, wastar_success, wastar_success_idx = relevant_indices(d['wastar'], cost_thresh)
    epase_idx, epase_success, epase_success_idx = relevant_indices(d['epase'], cost_thresh)


    relevant_ids = d['wastar']['id'][wastar_idx]
    epase_filtered = []

    for data in epase:
       if np.isin(data[0], relevant_ids):
           epase_filtered.append(data)

    # for relevant_id in relevant_ids:
    #    idx = np.array(np.where(np.isin(relevant_id, epase[:,0]) == True))[0]
    # pdb.set_trace()
    #   if idx.shape[0] == 1:
    #      epase_filtered.append(epase[idx, :])
    # elif idx.shape[0] > 1:
    #    print("WTF")
    #    pdb.set_trace()
    # else:
    #    print("{} not in epase".format(relevant_id))
    #    pdb.set_trace()


    pdb.set_trace()

    epase = np.array(epase_filtered)
    d['epase'] = {'id' : epase[:, 0], 'success' : epase[:, 1], 'time' : epase[:, 2], 'cost' : epase[:, 3], 'state_expansions' : epase[:, 4], 'edge_expansions' : epase[:, 5]} 
    epase_idx, _, _ = prune_failures(d['epase'])


    stats = {'wastar': {}, 'epase': {}}
    stats['wastar']['num_success'] = wastar_idx.shape[0]
    stats['wastar']['success_rate'] = wastar_success
    stats['wastar']['mean_cost'] = np.mean(d['wastar']['cost'][wastar_idx])
    stats['wastar']['mean_time'] = np.mean(d['wastar']['time'][wastar_idx])
    stats['wastar']['max_time'] = np.max(d['wastar']['time'][wastar_idx])
    stats['wastar']['mean_state_expansions'] = np.mean(d['wastar']['state_expansions'][wastar_idx])
    stats['wastar']['mean_edge_expansions'] = np.mean(d['wastar']['edge_expansions'][wastar_idx])

    stats['epase']['num_success'] = epase_idx.shape[0]
    stats['epase']['success_rate'] = epase_success
    stats['epase']['mean_cost'] = np.mean(d['epase']['cost'][epase_idx])
    stats['epase']['mean_time'] = np.mean(d['epase']['time'][epase_idx])
    stats['epase']['max_time'] = np.max(d['epase']['time'][epase_idx])
    stats['epase']['mean_state_expansions'] = np.mean(d['epase']['state_expansions'][epase_idx])
    stats['epase']['mean_edge_expansions'] = np.mean(d['epase']['edge_expansions'][epase_idx])

    for alg, stats in stats.items():
        print("------------------")
        print(alg)
        print("------------------")

        for stat, val in stats.items():
            print('{}: {}'.format(stat, val))

np.set_printoptions(suppress=True)
print_stats_2(0)
print_stats_2(1000)
print_stats_2(5000)

pdb.set_trace()
