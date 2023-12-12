# -*- coding: utf-8 -*-
import numpy as np

from utils.network import SumoNetwork
from utils.xml_io import parse_route


def lane_has_outgoing_to_edge(lane, edge_id):
    for next_lane in lane.getOutgoingLanes():
        if next_lane.getEdge().getID() == edge_id:
            return True
    return False


def lane_has_incoming_from_edge(lane, edge_id):
    for prev_lane in lane.getIncoming():
        if prev_lane.getEdge().getID() == edge_id:
            return True
    return False


def calc_incidence_matrix(net, edge_id_list_by_route):
    # default sorted not working when an edge has 10 more lanes
    lane_id_list = sorted(net.sumo_lanes.keys())
    route_id_list = sorted(edge_id_list_by_route.keys())
    n_row = len(lane_id_list)
    n_col = len(route_id_list)
    incidence_matrix = np.zeros((n_row, n_col))
    for i, lane_id in enumerate(lane_id_list):
        lane = net.sumo_lanes[lane_id]
        edge = lane.getEdge()
        edge_id = edge.getID()
        for j, route_id in enumerate(route_id_list):
            edge_id_list = edge_id_list_by_route[route_id]
            idx = edge_id_list.index(edge_id) if edge_id in edge_id_list else -1
            if idx == -1:
                continue
            if idx == 0:
                # verify downstream edge
                if not lane_has_outgoing_to_edge(lane, edge_id_list[idx + 1]):
                    continue
                this_edge_in_route_lane_num = 0
                for _lane in edge.getLanes():
                    if lane_has_outgoing_to_edge(_lane, edge_id_list[idx + 1]):
                        this_edge_in_route_lane_num += 1
                incidence_matrix[i, j] = 1 / this_edge_in_route_lane_num
                continue
            if idx == len(edge_id_list) - 1:
                # verify upstream edge
                if not lane_has_incoming_from_edge(lane, edge_id_list[idx - 1]):
                    continue
                this_edge_in_route_lane_num = 0
                for _lane in edge.getLanes():
                    if lane_has_incoming_from_edge(_lane, edge_id_list[idx - 1]):
                        this_edge_in_route_lane_num += 1
                incidence_matrix[i, j] = 1 / this_edge_in_route_lane_num
                continue
            # verify both upstream and downstream edges
            if (not lane_has_outgoing_to_edge(lane, edge_id_list[idx + 1]) or
                not lane_has_incoming_from_edge(lane, edge_id_list[idx - 1])):
                continue
            this_edge_in_route_lane_num = 0
            for _lane in edge.getLanes():
                if (lane_has_incoming_from_edge(_lane, edge_id_list[idx - 1]) and
                    lane_has_outgoing_to_edge(_lane, edge_id_list[idx + 1])):
                    this_edge_in_route_lane_num += 1
            incidence_matrix[i, j] = 1 / this_edge_in_route_lane_num

    return incidence_matrix, lane_id_list, route_id_list


if __name__ == '__main__':
    sumo_net = SumoNetwork('sumo_map/mcity.net.xml')
    _, edge_list_by_route = parse_route('sumo_map/mcity.route.xml')
    calc_incidence_matrix(sumo_net, edge_list_by_route)
