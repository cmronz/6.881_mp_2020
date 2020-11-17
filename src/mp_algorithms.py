import time
import numpy as np

from utils import *
from node import Node

#####################################################################
############################# Plain RRT #############################
#####################################################################

# def rrt(bounds, environment, start_point, radius, goal_point):
def rrt(bounds, start_point, radius, goal_point, time_out=False):
    """Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region."""
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
        dz = ran_loc[2] - nn.xyz[2]
        dy = ran_loc[1] - nn.xyz[1]
        dx = ran_loc[0] - nn.xyz[0]
        mag = (dz**2 + dy**2 + dx**2)**.5
        new_point = (nn.xyz[0] + dx/mag * extend_length, nn.xyz[1] + dy/mag * extend_length, nn.xyz[2] + dz/mag)

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
    """Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region."""
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
        dz = ran_loc[2] - nn.xyz[2]
        dy = ran_loc[1] - nn.xyz[1]
        dx = ran_loc[0] - nn.xyz[0]
        mag = (dz**2 + dy**2 + dx**2)**.5
        new_point = (nn.xyz[0] + dx/mag * extend_length, nn.xyz[1] + dy/mag * extend_length, nn.xyz[2] + dz/mag)

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

# def rrt(bounds, environment, start_point, radius, goal_point):
def rrt_star(bounds, start_point, radius, goal_point, prob_sample_goal=0.05, time_out=False):
    """Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region."""
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
        dz = ran_loc[2] - nn.xyz[2]
        dy = ran_loc[1] - nn.xyz[1]
        dx = ran_loc[0] - nn.xyz[0]
        mag = (dz**2 + dy**2 + dx**2)**.5
        new_point = (nn.xyz[0] + dx/mag * extend_length, nn.xyz[1] + dy/mag * extend_length, nn.xyz[2] + dz/mag)

        lines_to_display.append((nn.xyz, new_point))

        ''' Still Need to Check for Collisions '''
            
        # add this to the graph, and display
        node = Node(new_point, parent=nn)
        nn.children.append(node)
        
        # check if we reach end goal
        if close_enough(new_point, goal_point, 1.0):
            path = [n for n in node.path]
            return path, lines_to_display, time.time() - start_t

        if steps >= 1000 and time_out:
            return list(), lines_to_display, time.time() - start_t












