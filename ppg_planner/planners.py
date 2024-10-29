import numpy as np


from tqdm import tqdm

import ppg_planner.geometry as gm

class Data:
    def __init__(self):
        pass


class Node:
    def __init__(self, position: gm.Point2d, unique_id):
        self.position = position
        self.unique_id = unique_id
        self.data = None


class NodeGraph:
    # def __init__(self, input_map):
    #     self.node_list = node_list # n x n array, where n is the number of nodes
    #     self.node_incidency_matrix = {} # calculate

    #     if type(input_map) is list[quad.Quad]:
    #         self.generate_incidency_from_quad_list(input_map)
    #     elif type(input_map) is quad.Quad:
    #         self.generate_incidency_from_quad_tree(input_map)
    #     else:
    #         raise Exception("input_map must be list[quad.Quad] or quad.Quad")

    def __init__(self, node_list, node_incidency_matrix):
        self.node_list = node_list # n x n array, where n is the number of nodes
        self.node_incidency_matrix = node_incidency_matrix # calculate

        for uid, node_center in self.node_list.items():
            node = Node(node_center, uid)
            self.node_list[uid] = node


        def generate_incidency_from_quad_list(self, node_list):
            for i in tqdm(range(len(node_list))):
                node1 = node_list[i]
                self.node_incidency_matrix[i] = {}
                for j in range(len(node_list)):
                    node2 = node_list[j]
                    if node1 == node2:
                        continue
                    dr = ((node1.center_point.x - node2.center_point.x) ** 2 + (node1.center_point.y - node2.center_point.y) ** 2) ** 0.5
                    if dr < node2.side_size * 1.5:
                        self.node_incidency_matrix[i][j] = dr
            
            print(self.node_incidency_matrix)

