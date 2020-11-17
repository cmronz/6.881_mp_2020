import time
import numpy as np

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

def rrt_star(bounds, start_point, radius, goal_point, neighbor_radius=3, solution_check_prob=0.1, max_iters=None):
    ''' Returns a list of tuples describing an obstacle-free path that takes the robot from the start to the target region. '''
    start_t = time.time()
    extend_length = 1
    root = Node(start_point)

    # lines_to_display = dict()

    steps = 0
    
    while True:

        steps += 1

        # Randomly select a new node to add to the graph
        ran_loc = sample_cube(bounds)
        
        
        all_nodes = [d for d in root.all_descendents]

        # Find a node nearest to the graph
        nearest_neighbor = find_nearest_neighbor(ran_loc, all_nodes)

        # Get the line going from nearest to random point,
        new_point = steer(nearest_neighbor.xyz, ran_loc, 1)
        
        # nearest_to_new_cost = dist_between_points(nearest_neighbor, ran_loc)

        neighbors_within_radius = find_nearest_neighbors_within_radius(new_point, all_nodes, neighbor_radius)

        best_neighbor_and_cost = neighbors_within_radius[0]

        # add_to_display_dictionary(lines_to_display, best_neighbor_and_cost[0].xyz, new_point)

        ''' Still Need to Check for Collisions '''
            
        # add this to the graph, and display
        new_cost = best_neighbor_and_cost[1] + dist_between_points(best_neighbor_and_cost[0].xyz, new_point)
        new_node = Node(new_point, parent=best_neighbor_and_cost[0], cost=new_cost)
        best_neighbor_and_cost[0].children.append(new_node)

        for neighbor in neighbors_within_radius[1:]:
            if new_cost + dist_between_points(new_point, neighbor[0].xyz) < neighbor[0].cost:
                # print("\n\n\n\n\n", lines_to_display, "\n\n\n\n\n")
                # remove_from_display_dictionary(lines_to_display, neighbor[0])
                neighbor[0].parent.children.remove(neighbor[0])
                neighbor[0].parent = new_node
                new_node.children.append(neighbor[0])
                neighbor[0].cost = new_cost + dist_between_points(new_point, neighbor[0].xyz)
        
        # check if eng goal can be added to the graph
        if max_iters is None and np.random.rand() < solution_check_prob:
            nearest_neighbor_to_goal = find_nearest_neighbor(goal_point, [d for d in root.all_descendents])
            if dist_between_points(nearest_neighbor_to_goal.xyz, goal_point) <= 1:
                path_to_goal = [n for n in nearest_neighbor_to_goal.path]
                # return path_to_goal, form_parent_child_tuples(lines_to_display), time.time() - start_t
                return path_to_goal, list(), time.time() - start_t, nearest_neighbor_to_goal.cost + dist_between_points(nearest_neighbor_to_goal.xyz, goal_point)


        if max_iters and steps >= max_iters:
            nearest_neighbor_to_goal = find_nearest_neighbor(goal_point, [d for d in root.all_descendents])
            path_to_goal = [n for n in nearest_neighbor_to_goal.path]
            return path_to_goal, list(), time.time() - start_t, nearest_neighbor_to_goal.cost + dist_between_points(nearest_neighbor_to_goal.xyz, goal_point)





environment_bounds = (0, 0, 0, 11, 11, 11)
start_point = (0, 0, 0)  # starting location
goal_point = (10, 10, 10)  # goal location
radius = 3 # radius of robot moving (used in collision detection)
# dir_path, dir_lines, dir_rrt_time = rrt_star(environment_bounds, start_point, radius, goal_point)

# star_e = Environment('rrt*', bounds=environment_bounds)


# star_path, star_lines, star_rrt_time = rrt_star(environment_bounds, start_point, radius, goal_point)
# # for dline in star_lines:
# #     star_e.add_line(dline)

# star_e_title = "RRT* (Prob Solution Check = {}) Path Length = {}, Time to Complete = {} sec".format(0.1, len(star_path), star_rrt_time)
# star_e.add_path(dir_path)
# star_e.add_start(start_point)
# star_e.add_goal(goal_point)
# star_fig = go.Figure(data=dir_e.data)
# star_fig.update_layout(
#     title=star_e_title,
#     font_family="Courier New",
#     font_color="blue",
#     title_font_family="Times New Roman",
#     title_font_color="red",
#     legend_title_font_color="green"
# )
# star_fig.show()









