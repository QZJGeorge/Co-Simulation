# -*- coding: utf-8 -*-
import numpy as np
from matplotlib import pyplot as plt

from utils.bottleneck_analysis import sort_lane_df_by_congested_level, plot_bottleneck_lane_on_map, plot_relative_speed
from utils.calc_path_flow import update_path_flow
from utils.network import SumoNetwork
from utils.run_sim import run_sim


def plot_capacity_ratio_and_relative_speed(suffix, capacity_ratio_list, relative_speed_list):
    fig, ax = plt.subplots()
    ax.scatter(capacity_ratio_list, 1 - np.array(relative_speed_list), s=5)
    ax.set_xlabel('capacity ratio')
    ax.set_ylabel('1 - relative speed')
    ax.grid(linestyle='--', which='major', alpha=0.3)
    ax.set_aspect('equal')
    ax.plot((0, 1), (0, 1), color='red', linestyle='--')
    ax.set_xlim(-0.1, 1.1)
    ax.set_ylim(-0.1, 1.1)
    plt.show()
    fig.tight_layout()
    fig.savefig(f'outputs/figures/capacity_ratio_and_relative_speed_{suffix}.png', dpi=300)


def do_a_iteration(iter_idx, dry_run):
    net_filename = 'sumo_map/mcity.net.xml'
    net = SumoNetwork(net_filename)
    # Update the path flow
    lane_id_list, link_capacity_ratio_list, capacity_by_lane_id = update_path_flow(net_filename=net_filename,
                                                                                   route_filename='sumo_map/mcity.route.xml',
                                                                                   desire_flow_ratio=0.5875)

    # Run simulation
    if not dry_run:
        lane_data_dir = run_sim(sumo_conf='sumo_map/mcity.sumocfg',
                                output_dir='sumo_outputs/optimization',
                                iter_idx=iter_idx,
                                end_time=8*3600)
    else:
        lane_data_dir = f"sumo_outputs/optimization/iter_{iter_idx:03d}/"
    lane_data_filename = f'{lane_data_dir}/mcity.lane.xml'
    # Evaluate bottleneck
    top_n = 5
    lane_df = sort_lane_df_by_congested_level(lane_data_filename=lane_data_filename, by='speedRelative')
    relative_speed_by_lane_id = dict(zip(lane_df['lane_id'], lane_df['speedRelative']))
    capacity_ratio_by_lane_id = dict(zip(lane_id_list, link_capacity_ratio_list))
    lane_relative_speed_list = [relative_speed_by_lane_id.get(lane_id, 0) for lane_id in lane_id_list]
    lane_df['lane_length'] = lane_df['lane_id'].map(lambda lane_id: net.sumo_lanes[lane_id].getLength())
    lane_df['volume'] = lane_df['lane_id'].map(lambda lane_id: capacity_by_lane_id[lane_id] * capacity_ratio_by_lane_id[lane_id])
    lane_df['capacity'] = lane_df['lane_id'].map(lambda lane_id: capacity_by_lane_id[lane_id])
    lane_df['v/c ratio'] = lane_df['lane_id'].map(lambda lane_id: capacity_ratio_by_lane_id[lane_id])
    lane_df = lane_df[['lane_id', 'v/c ratio'] + lane_df.columns.tolist()[1:-1]]
    lane_df.to_csv(f'outputs/tables/lane_df_{iter_idx}.csv')
    plot_relative_speed(suffix=f'iter_{iter_idx}',
                        lane_id_list=lane_id_list,
                        relative_speed_list=lane_relative_speed_list)
    plot_bottleneck_lane_on_map(suffix=f'iter_{iter_idx}_top_{top_n}', lane_id_list=lane_df['lane_id'].tolist()[:top_n])
    plot_capacity_ratio_and_relative_speed(suffix=f'iter_{iter_idx}',
                                           capacity_ratio_list=link_capacity_ratio_list,
                                           relative_speed_list=lane_relative_speed_list)



if __name__ == '__main__':
    do_a_iteration(iter_idx=4, dry_run=False)
