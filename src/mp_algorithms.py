import time
import numpy as np

from environment import *
from utils import *
from node import Node

#####################################################################
############################# Plain RRT #############################
#####################################################################

# def rrt(bounds, environment, start_point, radius, goal_point):
def rrt(bounds, start_point, radius, goal_point, time_out=False):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    extend_length = .75
    root = Node(start_point)

    lines_to_display = list()

    steps = 0
    
    while True:

        steps += 1

        # Randomly select a new node to add to the graph
        ran_loc = sample_cube(bounds)
        
        
        # Find a node nearest to the graph
        nn = find_nearest_neighbor(ran_loc, [d for d in root.all_descendents])#nodes)
        
        # Get the line going from nearest to random point,
        new_point = steer(nn.xyz, ran_loc, 1)

        lines_to_display.append((nn.xyz, new_point))

        ''' Still Need to Check for Collisions '''
        # l = LineString([nn.point, new_point])
        # buff = l.buffer(radius, resolution=3)
        # cols = [buff.intersects(ob) for ob in environment.obstacles]
        # if any(cols):
        #     continue
            
        # add this to the graph, and display
        node = Node(new_point, parent=nn)
        nn.children.append(node)
        # plot_line(ax, l)
        
        # check if we reach end goal
        if close_enough(new_point, goal_point, .9):
            path = [n for n in node.path]
            # plot the path
            # pl = LineString(path)
            return path, lines_to_display, time.time() - start_t

        if steps >= 1000 and time_out:
            return list(), lines_to_display, time.time() - start_t

#####################################################################
############################ Directed RRT ###########################
#####################################################################

# def rrt(bounds, environment, start_point, radius, goal_point):
def directed_rrt(bounds, start_point, radius, goal_point, prob_sample_goal=0.05, time_out=False):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    extend_length = 1
    root = Node(start_point)

    lines_to_display = list()

    steps = 0
    
    while True:

        steps += 1

        if np.random.rand() < prob_sample_goal:
            ran_loc = goal_point
        else:
            # Randomly select a new node to add to the graph
            ran_loc = sample_cube(bounds)
        
        
        # Find a node nearest to the graph
        nn = find_nearest_neighbor(ran_loc, [d for d in root.all_descendents])#nodes)
        
        # Get the line going from nearest to random point,
        new_point = steer(nn.xyz, ran_loc, 1)

        lines_to_display.append((nn.xyz, new_point))

        ''' Still Need to Check for Collisions '''
        # l = LineString([nn.point, new_point])
        # buff = l.buffer(radius, resolution=3)
        # cols = [buff.intersects(ob) for ob in environment.obstacles]
        # if any(cols):
        #     continue
            
        # add this to the graph, and display
        node = Node(new_point, parent=nn)
        nn.children.append(node)
        # plot_line(ax, l)
        
        # check if we reach end goal
        if close_enough(new_point, goal_point, 1.0):
            path = [n for n in node.path]
            # plot the path
            # pl = LineString(path)
            return path, lines_to_display, time.time() - start_t

        if steps >= 1000 and time_out:
            return list(), lines_to_display, time.time() - start_t

#####################################################################
################################ RRT* ###############################
#####################################################################

# def rrt_star_iter_bound(environment, start_point, radius, goal_point, max_iters, nn_rad):
#     ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
#     start_t = time.time()
#     extend_length = 1
#     root = Node(start_point)

#     steps = 0
    
#     while True:

#         # Randomly select a new node to add to the graph
#         ran_loc = sample_cube(environment.bounds)

#         # Make sure we don't intersect with any obstacles
#         if not environment.is_point_obstacle_free(ran_loc):
#             continue

#         steps += 1
        
#         all_nodes = [d for d in root.all_descendents]

#         # Find a node nearest to the graph randomly sampled point
#         nearest_neighbor = find_nearest_neighbor(ran_loc, all_nodes)

#         # Get the line going from nearest vertex in G to random point, making our New Point
#         new_point = steer(nearest_neighbor.xyz, ran_loc, 1)

#         # Make sure we don't intersect with any obstacles
#         if not environment.is_line_obstacle_free(nearest_neighbor.xyz, new_point, 0.25):
#             continue

#         best_neighbor_and_cost = (nearest_neighbor, 1 + nearest_neighbor.cost)

#         neighbors_within_radius = find_nearest_neighbors_within_radius_distance_to_point_included(new_point, all_nodes, nn_rad)

#         if neighbors_within_radius:
#             continue
            
#         for neighbor in neighbors_within_radius:
#             if (neighbor[0].cost + neighbor[1]) < best_neighbor_and_cost[1]:

#                 if environment.is_line_obstacle_free(neighbor[0].xyz, new_point, 0.25):
#                     best_neighbor_and_cost = neighbor

#         # Add this to the graph
#         new_node = Node(new_point, parent=best_neighbor_and_cost[0], cost=best_neighbor_and_cost[1])
#         best_neighbor_and_cost[0].children.append(new_node)

#         for neighbor in neighbors_within_radius[0]:
#             # maybe make best_neighbor_and_cost[1] + neighbor[1] - neighbor[0].cost < neighbor[0].cost
#             # print("Does {} == {}".format((best_neighbor_and_cost[1] + dist_between_points(new_point, neighbor[0].xyz)), (best_neighbor_and_cost[1] + neighbor[1] - neighbor[0].cost)))
#             # If it is shorter path distance from new point, to one of the near neighbors...
#             if (best_neighbor_and_cost[1] + neighbor[1]) < (neighbor[0].cost * 2):

#                 if environment.is_line_obstacle_free(new_point, neighbor[0].xyz, 0.25):

#                     # remove this neighbor from their original parent's children list
#                     neighbor[0].parent.children.remove(neighbor[0])
#                     # Mark their parent as being the newly formed node
#                     neighbor[0].parent = new_node
#                     new_node.children.append(neighbor[0])

#                     # Set the cost of the neighbor to now reflect the shorter distance through our newly formed node
#                     neighbor[0].cost = best_neighbor_and_cost[1] + neighbor[1] - neighbor[0].cost


#         if (steps % 50) == 0:
#             print("RRT* Still running, {} iterations so far".format(steps))

#         if steps >= max_iters:
#             nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
#             path_to_goal = [n for n in nearest_neighbor_to_goal.path]
#             return path_to_goal, nearest_neighbor_to_goal.cost, nn_d_to_goal, root, time.time() - start_t

def rrt_star_iter_bound(environment, start_point, radius, goal_point, max_iters, nn_rad):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    extend_length = 1
    root = Node(start_point)

    steps = 0
    
    while True:

        # Randomly select a new node to add to the graph
        ran_loc = sample_cube(environment.bounds)

        # Make sure we don't intersect with any obstacles
        if not environment.is_point_obstacle_free(ran_loc):
            continue

        steps += 1
        
        all_nodes = [d for d in root.all_descendents]

        # Find a node nearest to the graph randomly sampled point
        nearest_neighbor = find_nearest_neighbor(ran_loc, all_nodes)

        # Get the line going from nearest vertex in G to random point, making our New Point
        new_point = steer(nearest_neighbor.xyz, ran_loc, 1)

        # Make sure we don't intersect with any obstacles
        if not environment.is_line_obstacle_free(nearest_neighbor.xyz, new_point, 0.25):
            continue

        best_neighbor_and_cost = (nearest_neighbor, 1 + nearest_neighbor.cost)

        ''' Still Need to Check for Collisions '''

        neighbors_within_radius = find_nearest_neighbors_within_radius_distance_to_point_included(new_point, all_nodes, nn_rad)

        if neighbors_within_radius:
            # This is a tuple of (parent_node_of_new_point, shortest_distance_from_graph_to_new_point)
            best_neighbor_and_cost = neighbors_within_radius[0]
            
        # Add this to the graph
        new_node = Node(new_point, parent=best_neighbor_and_cost[0], cost=best_neighbor_and_cost[1])
        best_neighbor_and_cost[0].children.append(new_node)

        for neighbor in neighbors_within_radius[1:]:
            # maybe make best_neighbor_and_cost[1] + neighbor[1] - neighbor[0].cost < neighbor[0].cost
            # print("Does {} == {}".format((best_neighbor_and_cost[1] + dist_between_points(new_point, neighbor[0].xyz)), (best_neighbor_and_cost[1] + neighbor[1] - neighbor[0].cost)))
            # If it is shorter path distance from new point, to one of the near neighbors...
            if (best_neighbor_and_cost[1] + neighbor[1]) < (neighbor[0].cost * 2):

                if environment.is_line_obstacle_free(new_point, neighbor[0].xyz, 0.25):

                    # remove this neighbor from their original parent's children list
                    neighbor[0].parent.children.remove(neighbor[0])
                    # Mark their parent as being the newly formed node
                    neighbor[0].parent = new_node
                    new_node.children.append(neighbor[0])

                    # Set the cost of the neighbor to now reflect the shorter distance through our newly formed node
                    neighbor[0].cost = best_neighbor_and_cost[1] + neighbor[1] - neighbor[0].cost

        if steps >= max_iters:
            nearest_neighbor_to_goal, nn_d_to_goal = find_nearest_neighbor_with_distance(goal_point, [d for d in root.all_descendents])
            path_to_goal = [n for n in nearest_neighbor_to_goal.path]
            return path_to_goal, nearest_neighbor_to_goal.cost, nn_d_to_goal, root, time.time() - start_t




# ''' THIS IS PROBABLY BROKEN STILL '''
# def rrt_star_prob_check(bounds, start_point, radius, goal_point, neighbor_radius=3, solution_check_prob=0.1):
#     ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
#     start_t = time.time()
#     extend_length = 1
#     root = Node(start_point)

#     steps = 0
    
#     while True:

#         steps += 1

#         # Randomly select a new node to add to the graph
#         ran_loc = sample_cube(bounds)
        
        
#         all_nodes = [d for d in root.all_descendents]

#         # Find a node nearest to the graph
#         nearest_neighbor = find_nearest_neighbor(ran_loc, all_nodes)

#         # Get the line going from nearest to random point,
#         new_point = steer(nearest_neighbor.xyz, ran_loc, 1)

#         # Set the best cost to get to the new point seen so far
#         best_cost_for_new_point = nearest_neighbor.cost + 1

#         neighbors_within_radius = find_nearest_neighbors_within_radius(new_point, all_nodes, neighbor_radius)

#         # best_neighbor_and_cost = neighbors_within_radius[0]

#         ''' Still Need to Check for Collisions '''
            
#         # add this to the graph, and display
#         new_cost = best_neighbor_and_cost[1] + dist_between_points(best_neighbor_and_cost[0].xyz, new_point)
#         new_node = Node(new_point, parent=best_neighbor_and_cost[0], cost=new_cost)
#         best_neighbor_and_cost[0].children.append(new_node)


#         # for neighbor in neighbors_within_radius[1:]:
#         #     neigh_node = neighbor[0]
#         #     neigh_dist = dist_between_points(new_point, neigh_node.xyz)

#         #     if new_cost + neigh_dist < neigh_node.cost:
#         #         neigh_node.parent.children.remove(neigh_node)
#         #         neigh_node.parent = new_node
#         #         new_node.children.append(neigh_node)
#         #         neigh_node.cost = new_cost + dist_between_points(new_point, neigh_node.xyz)

#         for neighbor in neighbors_within_radius[1:]:
#             if new_cost + dist_between_points(new_point, neighbor[0].xyz) < neighbor[0].cost:
#                 neighbor[0].parent.children.remove(neighbor[0])
#                 neighbor[0].parent = new_node
#                 new_node.children.append(neighbor[0])
#                 neighbor[0].cost = new_cost + dist_between_points(new_point, neighbor[0].xyz)
        
#         # check if eng goal can be added to the graph
#         if np.random.rand() < solution_check_prob:
#             nearest_neighbor_to_goal = find_nearest_neighbor(goal_point, [d for d in root.all_descendents])
#             if dist_between_points(nearest_neighbor_to_goal.xyz, goal_point) <= 1:
#                 path_to_goal = [n for n in nearest_neighbor_to_goal.path]
#                 # return path_to_goal, form_parent_child_tuples(lines_to_display), time.time() - start_t
#                 return path_to_goal, list(), time.time() - start_t, nearest_neighbor_to_goal.cost + dist_between_points(nearest_neighbor_to_goal.xyz, goal_point)


#         if (steps % 1000) == 0:
#             print("RRT* Still running, {} iterations so far".format(steps))










