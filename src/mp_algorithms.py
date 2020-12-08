import time
import numpy as np

from environment import *
from utils import *
from node import Node

# DeleteME
import math

#####################################################################
############################# Plain RRT #############################
#####################################################################

def rrt(environment, start_point, goal_point, max_iters, point_extraction_distance, extend_length=1, cutoff_distance=1):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    root = Node(start_point)

    lines_to_display = list()

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

        lines_to_display.append((nearest_neighbor.xyz, new_point))
            
        # add this to the graph of visited spaces
        node = Node(new_point, parent=nearest_neighbor)
        nearest_neighbor.children.append(node)
        
        # check if we are with 'cutoff_distance' reach of the end goal
        if close_enough(new_point, goal_point, cutoff_distance):
            path_to_goal = [n for n in node.path]
            return path_to_goal, len(path_to_goal) * extend_length, dist_between_points(new_point, goal_point), root, time.time() - start_t, steps

        if steps >= max_iters:
            nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
            path_to_goal = [n for n in nearest_neighbor_to_goal.path]
            return path_to_goal, len(path_to_goal) * extend_length, nn_d_to_goal, root, time.time() - start_t, steps

#####################################################################
############################ Directed RRT ###########################
#####################################################################

def directed_rrt(environment, start_point, goal_point, max_iters, point_extraction_distance, prob_sample_goal=0.05, extend_length=1, cutoff_distance=1):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    root = Node(start_point)

    lines_to_display = list()

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

        lines_to_display.append((nearest_neighbor.xyz, new_point))
            
        # add this to the graph of visited spaces
        node = Node(new_point, parent=nearest_neighbor)
        nearest_neighbor.children.append(node)
        
        # check if we are with 'cutoff_distance' reach of the end goal
        if close_enough(new_point, goal_point, cutoff_distance):
            path_to_goal = [n for n in node.path]
            return path_to_goal, len(path_to_goal) * extend_length, dist_between_points(new_point, goal_point), root, time.time() - start_t, steps

        if steps >= max_iters:
            nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
            path_to_goal = [n for n in nearest_neighbor_to_goal.path]
            return path_to_goal, len(path_to_goal) * extend_length, nn_d_to_goal, root, time.time() - start_t, steps

#####################################################################
################################ RRT* ###############################
#####################################################################

def rrt_star_iter_bound(environment, start_point, goal_point, max_iters, nn_rad, point_extraction_distance, prob_sample_goal=0.0, extend_length=1):
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

        # This is a list of tuples of (reference to a node that is with nn_rad distance of the new point, 
        # and the Cost(neighbor node) + Cost(line from this neighbor node to new point))
        neighbors_within_radius = find_nearest_neighbors_within_radius(new_point, all_nodes, nn_rad)

        min_cost = nearest_neighbor.cost + extend_length
        min_node = nearest_neighbor

        # Let's add this node along the minimum-cost-path to reach it from the nodes already in our tree
        for neighbor in neighbors_within_radius:

            cost_from_neighbor_to_new_node = neighbor[1] - neighbor[0].cost

            # If it is shorter path distance from new point, to one of the near neighbors...
            if cost_from_neighbor_to_new_node < min_cost:

                if environment.is_line_obstacle_free(new_point, neighbor[0].xyz, point_extraction_distance):

                    # We can assign a new min cost and min node
                    min_cost = cost_from_neighbor_to_new_node
                    min_node = neighbor[0]
            
        # Add this new point to the graph along the minimum-cost path
        new_node = Node(new_point, parent=min_node, cost=min_cost)
        min_node.children.append(new_node)

        # Now we can rewire the tree
        for neighbor in neighbors_within_radius:

            cost_from_new_node_to_nearby_neighbor = new_node.cost + dist_between_points(new_node.xyz, neighbor[0].xyz)

            # If it is shorter path distance from new point, to one of the near neighbors...
            if cost_from_new_node_to_nearby_neighbor < neighbor[0].cost:

                if environment.is_line_obstacle_free(new_node.xyz, neighbor[0].xyz, point_extraction_distance):

                    # remove this neighbor from their original parent's children list
                    neighbor[0].parent.children.remove(neighbor[0])
                    # Mark their parent as being the newly formed node
                    neighbor[0].parent = new_node
                    new_node.children.append(neighbor[0])
                    # Set the cost of the neighbor to now reflect the shorter distance through our newly formed node
                    neighbor[0].cost = cost_from_new_node_to_nearby_neighbor

        if steps >= max_iters:
            nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
            path_to_goal = [n for n in nearest_neighbor_to_goal.path]
            return path_to_goal, nearest_neighbor_to_goal.cost, nn_d_to_goal, root, time.time() - start_t, steps







