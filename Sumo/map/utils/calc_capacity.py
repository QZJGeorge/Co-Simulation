# -*- coding: utf-8 -*-
import os
from collections import defaultdict

import numpy as np
import pandas as pd
import sumolib

from utils.xml_io import load_lane_data_to_df, load_lane_capacity_df

net = sumolib.net.readNet('sumo_map/mcity.net.xml', withPrograms=True)
default_capacity = 1800  # veh/hour/lane
stop_sign_capacity = 720  # veh/hour/lane


def calc_capacity_for_all_lanes():
    capacity_by_lane = {}
    lane_id_list_by_dest_node_type = defaultdict(list)
    dest_node_id_by_lane_id = {}
    for node in net.getNodes():
        node_type = node.getType()
        if node_type == 'traffic_light':
            _capacity_by_lane = calc_lane_capacity_for_traffic_light_node(node)
            capacity_by_lane.update(_capacity_by_lane)
        elif node_type == 'priority':
            _capacity_by_lane = calc_lane_capacity_for_priority_node(node)
            capacity_by_lane.update(_capacity_by_lane)
        elif node_type == 'priority_stop':
            _capacity_by_lane = calc_lane_capacity_for_priority_stop_node(node)
            capacity_by_lane.update(_capacity_by_lane)
        elif node_type == 'allway_stop':
            _capacity_by_lane = calc_lane_capacity_for_allway_stop_node(node)
            capacity_by_lane.update(_capacity_by_lane)
        elif node_type == 'dead_end':
            _capacity_by_lane = calc_lane_capacity_for_dead_end_node(node)
            capacity_by_lane.update(_capacity_by_lane)
        else:
            raise NotImplementedError(f'Unknown node type: {node_type}')
        lane_id_list_by_dest_node_type[node_type].extend(list(_capacity_by_lane.keys()))
        for lane_id in _capacity_by_lane.keys():
            dest_node_id_by_lane_id[lane_id] = node_type

    return capacity_by_lane, dest_node_id_by_lane_id, lane_id_list_by_dest_node_type


def calc_lane_capacity_for_traffic_light_node(node):
    capacity_by_lane = {}
    tls = net.getTLS(node.getID())
    program = tls.getPrograms()['0']
    green_time_by_connection_idx = defaultdict(int)
    cycle = 0
    for phase in program.getPhases():
        cycle += phase.duration
        for conn_idx, state in enumerate(phase.state):
            if state in ('G', 'g', 'y'):
                green_time_by_connection_idx[conn_idx] += phase.duration
    for connection in tls.getConnections():
        incoming_lane, _, conn_idx = connection
        capacity_by_lane[incoming_lane.getID()] = green_time_by_connection_idx[conn_idx] / cycle * default_capacity

    return capacity_by_lane


def generate_priority_coef(priority_set):
    num = len(priority_set)
    if num == 4:
        coef_list = [0.4, 0.3, 0.2, 0.1]
    elif num == 3:
        coef_list = [0.6, 0.3, 0.1]
    elif num == 2:
        if -1 in priority_set:
            coef_list = [0.8, 0.2]
        else:
            coef_list = [0.6, 0.4]
    elif num == 1:
        coef_list = [0.25, 0.25, 0.25, 0.25]
    else:
        raise ValueError(f'Invalid priority list: {priority_set}')
    return coef_list


def calc_lane_capacity_for_priority_node(node):
    capacity_by_lane = {}
    priority_set = set()
    for edge in node.getIncoming():
        priority_set.add(edge.getPriority())

    priority_list = sorted(priority_set)
    coef_list = generate_priority_coef(priority_set)
    for edge in node.getIncoming():
        coef = coef_list[priority_list.index(edge.getPriority())]
        for lane in edge.getLanes():
            capacity_by_lane[lane.getID()] = default_capacity * coef
    return capacity_by_lane


def calc_lane_capacity_for_priority_stop_node(node):
    capacity_by_lane = {}
    max_priority = -100
    for edge in node.getIncoming():
        max_priority = max(max_priority, edge.getPriority())

    num_minor = 0
    edge_num = 0
    for edge in node.getIncoming():
        edge_num += 1
        if edge.getPriority() != max_priority:
            num_minor += 1
    if num_minor == 0:
        num_minor = edge_num

    for edge in node.getIncoming():
        if edge.getPriority() != max_priority:
            for lane in edge.getLanes():
                capacity_by_lane[lane.getID()] = stop_sign_capacity / num_minor
        else:
            for lane in edge.getLanes():
                capacity_by_lane[lane.getID()] = default_capacity * (1 - 1 / num_minor)
    return capacity_by_lane


def calc_lane_capacity_for_allway_stop_node(node):
    capacity_by_lane = {}
    num_edges = len(node.getIncoming())
    for edge in node.getIncoming():
        for lane in edge.getLanes():
            capacity_by_lane[lane.getID()] = stop_sign_capacity / num_edges
    return capacity_by_lane


def calc_lane_capacity_for_dead_end_node(node):
    capacity_by_lane = {}
    for edge in node.getIncoming():
        for lane in edge.getLanes():
            capacity_by_lane[lane.getID()] = default_capacity
    return capacity_by_lane


def estimate_capacity_by_rule_based_method():
    c_by_lane_id, type_by_lane_id, _ = calc_capacity_for_all_lanes()
    data = [(lane_id, type_by_lane_id[lane_id], c_by_lane_id[lane_id]) for lane_id in c_by_lane_id]
    df = pd.DataFrame(data, columns=['lane_id', 'dest. node type', 'rule-based capacity'])
    df.set_index('lane_id', inplace=True)
    df.sort_index(inplace=True)
    df.to_csv('sumo_outputs/rule_based_capacity.csv')


def estimate_max_volume_by_simulation(lane_data_filename):
    df = load_lane_data_to_df(lane_data_filename)
    df['avg_volume'] = df['speed'] * 3.6 * df['density']
    df['volume_begin'] = df['entered'] * 3600 / (df['end'] - df['begin'])
    df['volume_end'] = df['left'] * 3600 / (df['end'] - df['begin'])
    df['volume'] = df[['volume_begin', 'volume_end']].max(axis=1)
    # Group by lane_id and find the maximum volume
    df = pd.DataFrame(df[df['end'] > 1 * 3600].groupby('lane_id').agg({
        'avg_volume': 'max',
        'volume_begin': 'max',
        'volume_end': 'max',
        'volume': 'max'})['volume']).reset_index(names='lane_id').set_index('lane_id')
    return df


def estimate_max_volume_among_diff_scale(data_dir):
    column_list = []
    df_list = []
    for s in np.arange(0.1, 1.7, 0.1).tolist() + [2.0, 3.0, 5.0, 10.0]:
        s_str = f's={s:.1f}'
        path = f'{data_dir}/{s_str}/mcity.lane.xml'
        if not os.path.exists(path):
            continue
        column_list.append(s_str)
        lane_data_filename = f'{data_dir}/{s_str}/mcity.lane.xml'
        df = estimate_max_volume_by_simulation(lane_data_filename)
        df_list.append(df)
    combined_dataframe = pd.concat([df for df in df_list], axis=1)
    combined_dataframe.columns = column_list
    combined_dataframe['max_volume'] = combined_dataframe.max(axis=1)
    combined_dataframe[['max_volume']].to_csv(f'{data_dir}/max_volume.csv')
    return combined_dataframe[['max_volume']]


def search_max_volume():
    df1 = estimate_max_volume_among_diff_scale('sumo_outputs/initial_settings')
    df2 = estimate_max_volume_among_diff_scale('sumo_outputs/single_prob')
    df3 = estimate_max_volume_among_diff_scale('sumo_outputs/random_trip')
    sim_df = pd.concat([df1, df2, df3], axis=1)
    sim_df.columns = ['initial_settings', 'single_prob', 'random_trip']
    sim_df['sim_based_capacity'] = sim_df.max(axis=1)
    rule_df = pd.read_csv('sumo_outputs/rule_based_capacity.csv').set_index('lane_id')
    rule_df.merge(sim_df[['sim_based_capacity']], left_index=True, right_index=True, how='left').to_csv(
        'sumo_outputs/estimated_lane_capacity.csv')
    rule_df.merge(sim_df[['initial_settings', 'single_prob', 'random_trip']], left_index=True, right_index=True,
                  how='left').to_csv('sumo_outputs/estimated_lane_capacity.csv')


if __name__ == '__main__':
    output_dir = 'sumo_outputs/optimization/iter_003'
    df = estimate_max_volume_by_simulation('sumo_outputs/mcity.lane.xml')
    print()