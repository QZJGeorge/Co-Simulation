import os
import xml.etree.ElementTree as ET
from collections import defaultdict

import pandas as pd


def load_lane_data_to_df(path_to_lane_data_xml):
    tree = ET.parse(path_to_lane_data_xml)
    root = tree.getroot()
    data_list = []
    for interval in root:
        begin, end = float(interval.attrib['begin']), float(interval.attrib['end'])
        for edge in interval:
            edge_id = edge.attrib['id']
            for lane in edge:
                lane_id = lane.attrib['id']
                data = {'begin': begin, 'end': end, 'edge_id': edge_id, 'lane_id': lane_id}
                data.update({k: float(v) for k, v in lane.attrib.items() if k != 'id'})
                data_list.append(data)
    return pd.DataFrame(data_list)


def parse_route(route_filename):
    tree = ET.parse(route_filename)
    root = tree.getroot()
    route_list_by_edge = defaultdict(list)
    edge_list_by_route = defaultdict(list)
    for element in root:
        if not (element.tag == 'route' and element.attrib['id'].startswith('r_') and element.attrib['id'] != 'r_CAV'):
            continue
        route_id = element.attrib['id']
        edge_list_by_route[route_id] = element.attrib['edges'].split(' ')
        for edge in edge_list_by_route[route_id]:
            route_list_by_edge[edge].append(route_id)
    return route_list_by_edge, edge_list_by_route


def write_route_file(vph_by_route, route_filename):
    tree = ET.parse(route_filename)
    root = tree.getroot()
    for element in root:
        if not (element.tag == 'flow'):
            continue
        route_id = element.attrib['route']
        vph = vph_by_route[route_id]
        element.attrib['vehsPerHour'] = str(int(vph))
    tree.write(route_filename)


def load_lane_capacity_df():
    df = pd.read_csv('sumo_outputs/lane_capacity.csv')
    return df
