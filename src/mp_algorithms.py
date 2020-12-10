import time
import numpy as np

from environment import *
from utils import *
from node import Node

import random
import math 

#####################################################################
############################# Plain RRT #############################
#####################################################################

def rrt(environment, start_point, goal_point, max_iters, point_extraction_distance, extend_length=1, cutoff_distance=1):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    root = Node(start_point)

    steps = 0
    
    while True:

        # Randomly select a new node to add to the graph
        ran_loc = sample_cube(environment.bounds)

        # Make sure we don't intersect with any obstacles
        if not environment.is_point_obstacle_free(ran_loc):
            continue
        
        all_nodes = [d for d in root.all_descendents]

        # Find a node nearest to the graph randomly sampled point
        nearest_neighbor = find_nearest_neighbor(ran_loc, all_nodes)

        # Get the line going from nearest vertex in G to random point, making our New Point
        new_point = steer(nearest_neighbor.xyz, ran_loc, extend_length)

        # Make sure point is not inside obstacle
        if not environment.is_point_obstacle_free(new_point):
            continue

        # Make sure points along line to new point do not intersect obstacle
        if not environment.is_line_obstacle_free(nearest_neighbor.xyz, new_point, point_extraction_distance):
            continue

        steps += 1
            
        # add this to the graph of visited spaces
        node = Node(new_point, parent=nearest_neighbor)
        nearest_neighbor.children.append(node)

        if max_iters:
            if steps >= max_iters:
                nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
                path_to_goal = [n for n in nearest_neighbor_to_goal.path]
                return path_to_goal, calc_path_cost(path_to_goal), nn_d_to_goal, root, time.time() - start_t, steps
        
        # check if we are with 'cutoff_distance' reach of the end goal
        if close_enough(new_point, goal_point, cutoff_distance):
            nn_d_to_goal = dist_between_points(new_point, goal_point)
            if (nn_d_to_goal <= extend_length) and environment.is_line_obstacle_free(new_point, goal_point, point_extraction_distance):
                    path_to_goal = [n for n in node.path] + [goal_point]
                    nn_d_to_goal = 0
            else:
                path_to_goal = [n for n in node.path]
            return path_to_goal, calc_path_cost(path_to_goal), nn_d_to_goal, root, time.time() - start_t, steps

#####################################################################
############################ Directed RRT ###########################
#####################################################################

def directed_rrt(environment, start_point, goal_point, max_iters, point_extraction_distance, prob_sample_goal=0.05, extend_length=1, cutoff_distance=1):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    root = Node(start_point)

    steps = 0
    
    while True:

        # With probability specified at function call, sample the goal to 'direct' the search
        if np.random.rand() < prob_sample_goal:
            ran_loc = goal_point
        else:
            # Randomly select a new node to add to the graph
            ran_loc = sample_cube(environment.bounds)

        # Make sure we don't intersect with any obstacles
        if not environment.is_point_obstacle_free(ran_loc):
            continue
        
        all_nodes = [d for d in root.all_descendents]

        # Find a node nearest to the graph randomly sampled point
        nearest_neighbor = find_nearest_neighbor(ran_loc, all_nodes)

        # Get the line going from nearest vertex in G to random point, making our New Point
        new_point = steer(nearest_neighbor.xyz, ran_loc, extend_length)

        # Make sure point is not inside obstacle
        if not environment.is_point_obstacle_free(new_point):
            continue

        # Make sure points along line to new point do not intersect obstacle
        if not environment.is_line_obstacle_free(nearest_neighbor.xyz, new_point, point_extraction_distance):
            continue

        steps += 1
            
        # add this to the graph of visited spaces
        node = Node(new_point, parent=nearest_neighbor)
        nearest_neighbor.children.append(node)

        if max_iters:
            if steps >= max_iters:
                nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
                path_to_goal = [n for n in nearest_neighbor_to_goal.path]
                return path_to_goal, calc_path_cost(path_to_goal), nn_d_to_goal, root, time.time() - start_t, steps
        
        # check if we are with 'cutoff_distance' reach of the end goal
        if close_enough(new_point, goal_point, cutoff_distance):
            nn_d_to_goal = dist_between_points(new_point, goal_point)
            if (nn_d_to_goal <= extend_length) and environment.is_line_obstacle_free(new_point, goal_point, point_extraction_distance):
                    path_to_goal = [n for n in node.path] + [goal_point]
                    nn_d_to_goal = 0
            else:
                path_to_goal = [n for n in node.path]
            return path_to_goal, calc_path_cost(path_to_goal), nn_d_to_goal, root, time.time() - start_t, steps

#####################################################################
################################ RRT* ###############################
#####################################################################

def rrt_star_iter_bound(environment, start_point, goal_point, max_iters, nn_rad, point_extraction_distance, prob_check_sol=0.1, extend_length=1):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    root = Node(start_point)

    steps = 0
    
    while True:

        # Randomly select a new node to add to the graph
        ran_loc = sample_cube(environment.bounds)
        
        all_nodes = [d for d in root.all_descendents]

        # Find a node nearest to the graph randomly sampled point
        nearest_neighbor = find_nearest_neighbor(ran_loc, all_nodes)

        # Get the line going from nearest vertex in G to random point, making our New Point
        new_point = steer(nearest_neighbor.xyz, ran_loc, extend_length)

        # Make sure point is not inside obstacle
        if not environment.is_point_obstacle_free(new_point):
            continue

        # Make sure points along line to new point do not intersect obstacle
        if not environment.is_line_obstacle_free(nearest_neighbor.xyz, new_point, point_extraction_distance):
            continue

        steps += 1

        neighbors_within_radius = get_nearby_neighbors(new_point, all_nodes, nn_rad)

        min_cost = np.inf
        min_node = None
        min_idx = None

        # Let's add this node along the minimum-cost-path to reach it from the nodes already in our tree
        for i in range(len(neighbors_within_radius)):
            neighbor = neighbors_within_radius[i]
            cost_from_neighbor_to_new_node = dist_between_points(new_point, neighbor.xyz)
            cost_to_neighbor = calc_path_cost([ancestor_node for ancestor_node in neighbor.path])

            # If it is shorter path distance from new point, to one of the near neighbors...
            if (cost_to_neighbor + cost_from_neighbor_to_new_node) < min_cost:

                if environment.is_line_obstacle_free(neighbor.xyz, new_point, point_extraction_distance):

                    # We can assign a new min cost and min node
                    min_cost = cost_to_neighbor + cost_from_neighbor_to_new_node
                    min_node = neighbor
                    min_idx = i
            
        # Add this new point to the graph along the minimum-cost path
        new_node = Node(new_point, parent=min_node)
        min_node.children.append(new_node)
        

        if min_idx:
            # remove this neighbor from the list
            neighbors_within_radius.pop(min_idx)

        # Now we can rewire the tree
        for neighbor in neighbors_within_radius:

            cost_from_new_node_to_nearby_neighbor = min_cost + dist_between_points(new_node.xyz, neighbor.xyz)

            # If it is shorter path distance from new point, to one of the near neighbors...
            if cost_from_new_node_to_nearby_neighbor < calc_path_cost([ancestor_node for ancestor_node in neighbor.path]):

                if environment.is_line_obstacle_free(new_node.xyz, neighbor.xyz, point_extraction_distance):

                    # remove this neighbor from their original parent's children list
                    neighbor.parent.children.remove(neighbor)

                    # Mark their parent as being the newly formed node
                    neighbor.parent = new_node
                    new_node.children.append(neighbor)

        if max_iters:
            if steps >= max_iters:
                nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
                if (nn_d_to_goal <= extend_length) and environment.is_line_obstacle_free(nearest_neighbor_to_goal.xyz, goal_point, point_extraction_distance):
                        path_to_goal = [n for n in nearest_neighbor_to_goal.path] + [goal_point]
                        nn_d_to_goal = 0
                else:
                    path_to_goal = [n for n in nearest_neighbor_to_goal.path]
                return path_to_goal, calc_path_cost(path_to_goal), nn_d_to_goal, root, time.time() - start_t, steps







