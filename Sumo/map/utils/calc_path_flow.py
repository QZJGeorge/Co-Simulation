# -*- coding: utf-8 -*-
import gurobipy as gp
import numpy as np
from matplotlib import pyplot as plt

from utils.calc_incidence_matrix import calc_incidence_matrix
from utils.network import SumoNetwork
from utils.xml_io import parse_route, write_route_file, load_lane_capacity_df


def solve_flow_assignment(P, desire_capacity_ratio, link_capacity_list, weight_list):
    link_num, path_num = P.shape
    d = np.ones(link_num) * desire_capacity_ratio
    c = np.array(link_capacity_list)
    w = np.array(weight_list)

    model = gp.Model('Path Flow Assignment')
    f = model.addVars(path_num, lb=0, name='path_flow')
    objective = gp.quicksum(((gp.quicksum(P[i, j] * f[j]
                                          for j in range(path_num)) / c[i] - d[i]) * w[i]) ** 2
                            for i in range(link_num))
    model.setObjective(objective, gp.GRB.MINIMIZE)
    model.optimize()

    optimal_flows = None
    if model.status == gp.GRB.OPTIMAL:
        optimal_flows = [f[j].x for j in range(path_num)]
        print("Optimal Path Flows:", optimal_flows)
    else:
        print("No optimal solution found.")
    return optimal_flows


def plot_assign_result(link_capacity_ratio_list, desire_capacity_ratio, link_id_list):
    link_num = len(link_capacity_ratio_list)

    fig, ax = plt.subplots(figsize=(20, 5), dpi=200)
    ax.scatter(range(link_num), link_capacity_ratio_list,
               s=5, label='link capacity ratio')
    ax.axhline(desire_capacity_ratio,
               color='red', linestyle='--', label='desire capacity ratio')
    ax.set_xticks(range(link_num))
    ax.set_xticklabels(link_id_list, rotation=90)
    ax.set_xlabel('lane id')
    ax.set_xlim(-1, link_num)
    ax.set_ylabel('capacity ratio')
    ax.legend()
    ax.grid(linestyle='--', which='major', alpha=0.3)
    fig.tight_layout()
    plt.show()


def calc_link_capacity_ratio_list(P, link_capacity_list, path_flow):
    link_num, path_num = P.shape
    link_capacity_ratio_list = []
    for i in range(link_num):
        link_flow = sum([P[i, j] * path_flow[j] for j in range(path_num)])
        link_capacity_ratio_list.append(link_flow / link_capacity_list[i])
    return link_capacity_ratio_list


def update_path_flow(net_filename, route_filename, desire_flow_ratio):
    sumo_net = SumoNetwork(net_filename)
    _, edge_list_by_route = parse_route(route_filename)
    incidence_matrix, lane_id_list, route_id_list = calc_incidence_matrix(sumo_net, edge_list_by_route)
    df = load_lane_capacity_df()
    c_by_lane_id, m_by_lane_id, w_by_lane_id = (df.set_index('lane_id')['capacity'].to_dict(),
                                                df.set_index('lane_id')['manual_adjustment'].to_dict(),
                                                df.set_index('lane_id')['weight'].to_dict())
    lane_capacity_list = [c_by_lane_id[lane_id] * m_by_lane_id[lane_id] for lane_id in lane_id_list]
    weight_list = [w_by_lane_id[lane_id] for lane_id in lane_id_list]
    path_flow = solve_flow_assignment(P=incidence_matrix,
                                      desire_capacity_ratio=desire_flow_ratio,
                                      link_capacity_list=lane_capacity_list,
                                      weight_list=weight_list)
    if path_flow is not None:
        link_capacity_ratio_list = calc_link_capacity_ratio_list(P=incidence_matrix,
                                                                 link_capacity_list=lane_capacity_list,
                                                                 path_flow=path_flow)
        plot_assign_result(link_capacity_ratio_list,
                           desire_capacity_ratio=desire_flow_ratio,
                           link_id_list=lane_id_list)
        path_flow_by_route_id = dict(zip(route_id_list, path_flow))
        print(path_flow_by_route_id)
        write_route_file(path_flow_by_route_id, route_filename)
    else:
        print('No solution found.')
        link_capacity_ratio_list = None

    return lane_id_list, link_capacity_ratio_list, dict(zip(lane_id_list, lane_capacity_list))


if __name__ == '__main__':
    update_path_flow(net_filename='sumo_map/mcity.net.xml',
                     route_filename='sumo_map/mcity.route.xml',
                     desire_flow_ratio=0.2)
