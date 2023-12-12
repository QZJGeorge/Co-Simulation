# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

from utils.net_const import LinkLayer
from utils.network import SumoNetwork
from utils.plotter import draw_network, draw_links
from utils.xml_io import load_lane_data_to_df

PERIOD = 300


def sort_lane_df_by_congested_level(lane_data_filename, by):
    df = load_lane_data_to_df(lane_data_filename)
    df['avg_trajs_num'] = df['sampledSeconds'] / PERIOD
    df['total_travel_distance'] = df['sampledSeconds'] * df['speed']

    def customized_mean(group):
        _dict = {}
        for metric in ['speed', 'speedRelative', 'occupancy']:
            _dict[metric] = (group[metric] * group['avg_trajs_num']).sum() / group['avg_trajs_num'].sum()
        for metric in ['laneDensity', 'sampledSeconds', 'waitingTime', 'timeLoss', 'total_travel_distance']:
            _dict[metric] = group[metric].mean()
        return pd.Series(_dict)

    result = pd.DataFrame(
        df[df['end'] > 1 * 3600].groupby('lane_id').apply(customized_mean).sort_values(by=by).reset_index())
    return result


def plot_relative_speed(suffix, lane_id_list, relative_speed_list):
    fig, ax = plt.subplots(figsize=(20, 5), dpi=200)
    ax.scatter(range(len(lane_id_list)), 1 - np.array(relative_speed_list),
               s=5, label='1-relative speed')
    ax.set_xticks(range(len(lane_id_list)))
    ax.set_xticklabels(lane_id_list, rotation=90)
    ax.set_xlabel('lane id')
    ax.set_xlim(-1, len(lane_id_list))
    ax.set_ylabel('relative speed')
    ax.legend()
    ax.grid(linestyle='--', which='major', alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'outputs/figures/relative_speed_{suffix}.png', dpi=300)


def plot_bottleneck_lane_on_map(suffix, lane_id_list):
    net = SumoNetwork("sumo_map/mcity.net.xml")

    fig, ax = plt.subplots(figsize=(7, 11), dpi=200)
    draw_network(ax, net, with_node_labels=False, link_layer=LinkLayer.LANE)

    draw_links(ax, net, link_layer=LinkLayer.LANE, link_list=lane_id_list, color="r")

    fig.savefig(f'outputs/figures/bottleneck_lane_{suffix}.png', dpi=200)


if __name__ == '__main__':
    for n in range(1, 11):
        lane_df = sort_lane_df_by_congested_level('sumo_outputs/initial_settings/s=1.0/mcity.lane.xml',
                                                  by='speedRelative')
        plot_bottleneck_lane_on_map(suffix=f'top_{n}', lane_id_list=lane_df['lane_id'].tolist()[:n])
