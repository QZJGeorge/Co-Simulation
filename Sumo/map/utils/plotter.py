import networkx as nx
import numpy as np

from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.collections import PatchCollection
from utils.net_const import LinkLayer


# def draw_nx_graph(network, **kwargs):
#     """
#     Use networkx package to draw network
#
#     :param network:
#     :param kwargs: Please refer to the documentation of networkx
#     :return:
#     """
#     nx.draw_networkx(network.networkx,
#                      nx.get_node_attributes(network.networkx, "pos"),
#                      **kwargs)


def _plot_polygon(ax, poly, **kwargs):
    path = Path.make_compound_path(
        Path(np.asarray(poly.exterior.coords)[:, :2]),
        *[Path(np.asarray(ring.coords)[:, :2]) for ring in poly.interiors])

    patch = PathPatch(path, **kwargs)
    collection = PatchCollection([patch], **kwargs)

    ax.add_collection(collection, autolim=True)
    ax.autoscale_view()
    return collection


def draw_network(ax, network, with_node_labels=False, link_layer=LinkLayer.EDGE):
    draw_nodes(ax, network, node_list=None, with_labels=with_node_labels)
    draw_node_geometry(ax, network, node_list=None)
    draw_links(ax, network, link_list=None, link_layer=link_layer)


def draw_nodes(ax, network, node_list=None, with_labels=False, **kwargs):
    kwargs = {"s": 100, "edgecolors": "k", "alpha": 0.5, **kwargs}
    # draw node coord
    scatter_x, scatter_y = [], []
    for node_id, node_coord in network.node_coord_dict.items():
        if isinstance(node_list, list):
            if not (node_id in node_list):
                continue
        scatter_x.append(node_coord.x)
        scatter_y.append(node_coord.y)
        if with_labels:
            ax.text(node_coord.x - 10, node_coord.y, node_id, bbox=dict(facecolor='white', alpha=1))
    ax.scatter(scatter_x, scatter_y, **kwargs)


def draw_node_geometry(ax, network, node_list=None, **kwargs):
    kwargs = {"facecolor": 'k', "edgecolor": "k", "alpha": 0.2, **kwargs}
    for node_id, node_shape in network.node_shape_dict.items():
        if isinstance(node_list, list):
            if not (node_id in node_list):
                continue
        _plot_polygon(ax, node_shape, **kwargs)


def draw_links(ax, network, link_list=None,
               link_layer=LinkLayer.EDGE, **kwargs):
    """

    :param ax:
    :param network:
    :param link_list:
    :param link_layer:
    :param kwargs:
    :return:
    """
    kwargs = {"color": 'k', **kwargs}
    if link_layer == LinkLayer.EDGE:
        link_geometry_dict = network.edge_geometry_dict
    elif link_layer == LinkLayer.LANE:
        link_geometry_dict = network.lane_geometry_dict
    else:
        raise ValueError("Input LinkLayer incorrect")

    # draw edge geometry
    for link_id, link_geometry in link_geometry_dict.items():
        if isinstance(link_list, list):
            if not (link_id in link_list):
                continue
        ax.plot(*link_geometry.xy, **kwargs)
        number_of_points = len(link_geometry.coords)
        if number_of_points < 2:
            continue

        last_point = link_geometry.coords[-1]
        second_last = link_geometry.coords[-2]
        dx, dy = last_point[0] - second_last[0], last_point[1] - second_last[1]
        ax.arrow(second_last[0], second_last[1], dx, dy, shape='full',
                 length_includes_head=False, head_width=2.5, width=0, **kwargs)


def draw_path(ax, path, network, color="b"):
    edge_id_list = [val.getID() for val in path]
    node_id_list = [val.getFromNode().getID() for val in path]
    node_id_list.append(path[-1].getToNode().getID)

    draw_nodes(ax, network, node_id_list, color=color, alpha=1)
    draw_links(ax, network, edge_id_list, color=color)
    draw_node_geometry(ax, network, node_id_list,
                       facecolor=color, edgecolor=color)
