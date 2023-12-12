import matplotlib.pyplot as plt

from utils.net_const import LinkLayer
from utils.network import SumoNetwork
from utils.plotter import draw_network, draw_path, draw_links


net = SumoNetwork("sumo_map/mcity.net.xml")

from_edge_id, to_edge_id = "EG_29_1_1", "EG_11_2_1"

fig, ax = plt.subplots(figsize=(7, 11), dpi=200)
draw_network(ax, net, with_node_labels=False, link_layer=LinkLayer.LANE)
path = net.get_fastest_path(from_edge_id, to_edge_id)

draw_path(ax, path, net)

draw_links(ax, net, link_layer=LinkLayer.EDGE, link_list=[from_edge_id, to_edge_id], color="r")

fig.show()


