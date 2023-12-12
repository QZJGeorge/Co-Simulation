import sumolib
from shapely.geometry import Point, Polygon, LineString


class SumoNetwork(object):
    def __init__(self, network_file):
        self.network_file = network_file

        self.sumo_net = None

        self.sumo_nodes = {}
        self.sumo_edges = {}
        self.sumo_lanes = {}
        self.sumo_connections = {}

        # geometry and coordinates
        self.node_coord_dict = {}
        self.node_shape_dict = {}
        self.edge_geometry_dict = {}
        self.lane_geometry_dict = {}

        self.load_network()
        self.retrieve_geometry()

    def get_fastest_path(self, from_edge_id, to_edge_id):
        from_edge, to_edge = self.sumo_net.getEdge(from_edge_id), self.sumo_net.getEdge(to_edge_id)
        path = self.sumo_net.getFastestPath(from_edge, to_edge)[0]
        return path

    def load_network(self):
        self.sumo_net = sumolib.net.readNet("sumo_map/mcity.net.xml")
        # fixme: we need to also build the networkx to enable different network functions
        # self.networkx, self._sumo_to_nx_link_dict, self._nx_link_to_sumo_dict =\
        #     build_net_from_sumo_map(self.sumo_net, layer)

        node_list = self.sumo_net.getNodes()
        for node in node_list:
            self.sumo_nodes[node.getID()] = node
        for edge in self.sumo_net.getEdges():
            self.sumo_edges[edge.getID()] = edge

        self.sumo_lanes = {}
        self.sumo_connections = {}
        for edge in self.sumo_edges.values():
            local_lane_list = edge.getLanes()
            for lane in local_lane_list:
                self.sumo_lanes[lane.getID()] = lane

            for cons in edge.getOutgoing().values():
                for con in cons:
                    from_lane = con.getFromLane()
                    to_lane = con.getToLane()
                    self.sumo_connections[f"{from_lane.getID()}>{to_lane.getID()}"] = con

    def retrieve_geometry(self):
        for edge in self.sumo_edges.values():
            edge_id = edge.getID()
            edge_geometry = edge.getShape()
            edge_line_string = LineString(edge_geometry)
            self.edge_geometry_dict[edge_id] = edge_line_string

        for lane in self.sumo_lanes.values():
            lane_id = lane.getID()
            lane_geometry = lane.getShape()
            lane_line_string = LineString(lane_geometry)
            self.lane_geometry_dict[lane_id] = lane_line_string

        for node in self.sumo_nodes.values():
            node_id = node.getID()
            node_coord = node.getCoord()
            node_shape = node.getShape()
            self.node_coord_dict[node_id] = Point(node_coord)
            if len(node_shape) < 4:
                # by xingmin: I use a trick to avoid an issue
                # fixme: this is not a general way to do that, this issue comes from the case when there are
                #  only two points that we want to transfer them to a polygon
                node_shape += node_shape[::-1]
            self.node_shape_dict[node_id] = Polygon(node_shape)

