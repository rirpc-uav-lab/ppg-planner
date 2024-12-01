# Initial code was taken from https://www.geeksforgeeks.org/quad-tree/
from ppg_planner.geometry import Point2d, SquareRegion, LineLike, GeometryDrawer, Line2d, PolyLike
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon
import ppg_planner.planners as planners
import numpy as np
import math
import cv2 as cv
from getpass import getuser
from copy import copy, deepcopy


class WaveGraph:
    def __init__(self):
        self.node_list = {}
        self.node_incidency_matrix = {}
        self.edge_node_list = {}
    
    def capture_node(self, node: planners.Node, node_incidency_matrix: dict):
        self.node_list[node.unique_id] = node
        self.node_incidency_matrix[node.unique_id] = node_incidency_matrix

        for uid, distance in node_incidency_matrix.items():
            if uid not in self.node_list:
                self.edge_node_list[node.unique_id] = node

    def release_node(self, node: planners.Node):
        del(self.node_list[node.unique_id])
        del(self.node_incidency_matrix[node.unique_id])
        if node.unique_id in self.edge_node_list:
            del(self.edge_node_list[node.unique_id])

    def check_if_node_is_on_edge(self, node: planners.Node):
        node_is_on_edge = True
        for uid, distance in self.node_incidency_matrix[node.unique_id].items():
            if uid not in self.node_list:
                node_is_on_edge = True
                break
            else: 
                node_is_on_edge = False
        return node_is_on_edge

class ZoneDivisionWavefront:
    def __init__(self):
        pass


    def point2d_to_uid(self, point: Point2d):
        return str(point.x) + "," + str(point.y)

    def uid_to_point2d(self, uid):
        return Point2d(float(uid.split(",")[0]), float(uid.split(",")[1]))
    

    def divide(self, zone: planners.NodeGraph, drone_positions: list[Odometry]):
        zones_by_drone = {}
        drone_start_positions = {}
        wave_graphs_by_drone = {}

        free_node_count = len(zone.node_list)

        print(free_node_count)

        # initialize wave_graphs and zones
        for i in range(len(drone_positions)):
            zones_by_drone[i] = Polygon()
            wave_graphs_by_drone[i] = WaveGraph()
        
        # find the closest node to each drone and make it a starting position
        for i in range(len(drone_positions)):
            min_dr = math.inf
            min_dr_node_uid = None

            for uid, node in zone.node_list.items():
                dr = ((drone_positions[i].pose.pose.position.x - node.position.x) ** 2 + (drone_positions[i].pose.pose.position.y - node.position.y) ** 2) ** 0.5
                if dr < min_dr:
                    if not uid in drone_start_positions:
                        min_dr = dr
                        min_dr_node_uid = uid

            drone_start_positions[i] = Point2d(float(min_dr_node_uid.split(",")[0]), float(min_dr_node_uid.split(",")[1]))
            wave_graphs_by_drone[i].capture_node(zone.node_list[min_dr_node_uid], zone.node_incidency_matrix[min_dr_node_uid])
            free_node_count -= 1
        
        # divide the zone into regions
        
        while free_node_count > 0:
            print(free_node_count)
            for i in range(len(drone_positions)):
                current_expansionist_node = None
                for uid, node in wave_graphs_by_drone[i].edge_node_list.items():
                    # choice logic
                    current_expansionist_node = node
                    # break

                    for uid, distance in wave_graphs_by_drone[i].node_incidency_matrix[current_expansionist_node.unique_id].items():
                        capture_this_node = True
                        if uid in wave_graphs_by_drone[i].node_list:
                            capture_this_node = False
                        for j in range(len(drone_positions)):
                            # print(f"uid in wave_graphs_by_drone[j].node_list: {uid in wave_graphs_by_drone[j].node_list}")
                            # print(f"uid in wave_graphs_by_drone[i].node_list: {uid in wave_graphs_by_drone[i].node_list}")
                            if i == j:
                                continue
                            if uid in wave_graphs_by_drone[j].node_list:
                                capture_this_node = False
                                break
                        print(f"capture_this_node: {capture_this_node}")
                        if capture_this_node:
                            # print("Capture node")
                            wave_graphs_by_drone[i].capture_node(zone.node_list[uid], zone.node_incidency_matrix[uid])
                            free_node_count -= 1
                
                if not wave_graphs_by_drone[i].check_if_node_is_on_edge(current_expansionist_node):
                    # wave_graphs_by_drone[i].release_node(current_expansionist_node)
                    del(wave_graphs_by_drone[i].edge_node_list[current_expansionist_node.unique_id])
                    # break

        return wave_graphs_by_drone
