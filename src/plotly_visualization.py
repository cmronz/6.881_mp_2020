import chart_studio.plotly as py

from scipy import interpolate
from plotly import graph_objs as go
from environment import Environment
from mp_algorithms import *
from path_smoothing import *
import sys 

DEMO = False
KUKA_ENV = False
JUST_ENV = False
RRT = False
DRRT = False
SRRT = False

DEMO = True
# KUKA_ENV = True
# JUST_ENV = True
# RRT = True
# DRRT = True
# SRRT = True

#####################################################################
########################### Visualization ###########################
#####################################################################
if DEMO: 
    np.random.seed(50)

    env_bounds = (0, 0, 0, 11, 11, 11)
    start_point = (0, 0, 0)  # starting location
    goal_point = (10, 10, 10)  # goal location

    PED = .25
    extending_length = 1

    cutoff_d = 1

    max_iters = 500
    sample_goal_prob = 0.1

    scattered_small_boxes = np.array(
        [(2, 2, 2, 4, 4, 4), (2, 2, 6, 4, 4, 8), (2, 6, 2, 4, 8, 4), (6, 6, 2, 8, 8, 4),
         (6, 2, 2, 8, 4, 4), (6, 2, 6, 8, 4, 8), (2, 6, 6, 4, 8, 8), (6, 6, 6, 8, 8, 8)])

    one_big_box = np.array([(2.75, 2.75, 2.75, 7.75, 7.75, 7.75)])

    wall = np.array([(1.5, 5, 1, 2.5, 11, 11), (1.5, 1, 1, 2.5, 4, 11)])

    maze = np.array([
        (1, 0, -1, 2, 6, 11), (1, 7,  -1, 2, 11, 11),
        (4, 0, -1, 5, 9, 11), (4, 10, -1, 5, 11, 11),
        (8, 2, -1, 9, 11, 11)
        ])

    obs = scattered_small_boxes

    # only_environment = Environment(filename='just env', bounds=env_bounds, obstacles=obs)
    
    # env_title = "Environment for RRT"

    # only_environment.add_start(start_point)
    # only_environment.add_goal(goal_point)
    # env_fig = go.Figure(data=only_environment.data)
    # env_fig.update_layout(
    #     title=env_title,
    #     font_family="Courier New",
    #     font_color="blue",
    #     title_font_family="Times New Roman",
    #     title_font_color="red",
    #     legend_title_font_color="green"
    # )
    # env_fig.show()

#####################################################################
############################# Plain RRT #############################
#####################################################################

    rrt_e = Environment(filename='plain rrt', bounds=env_bounds, obstacles=obs)

    rrt_path, path_cost, distance_from_goal, root_node, rrt_time, iters = rrt(rrt_e, start_point, goal_point, max_iters, PED, extend_length=extending_length, cutoff_distance=cutoff_d)

    title = "RRT (Iter={}, Cost={}, Dist_to_G={}) in {} sec".format(iters, path_cost, round(distance_from_goal, 2), round(rrt_time, 4))

    leaves = find_leaf_nodes(root_node)

    paths_to_leaves = [[la for la in leaf.path] for leaf in leaves]
    for lp in paths_to_leaves:
        rrt_e.add_line(lp)

    rrt_e.add_path(rrt_path)

    rrt_e.add_start(start_point)
    rrt_e.add_goal(goal_point)
    fig = go.Figure(data=rrt_e.data)
    fig.update_layout(
        title=title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    fig.show()

#####################################################################
############################ Directed RRT ###########################
#####################################################################

    dir_rrt_e = Environment(filename='directed rrt', bounds=env_bounds, obstacles=obs)

    dir_rrt_path, dir_path_cost, dir_distance_from_goal, dir_root_node, dir_rrt_time, dir_itters = directed_rrt(dir_rrt_e, start_point, goal_point, max_iters, PED, prob_sample_goal=sample_goal_prob, extend_length=extending_length, cutoff_distance=cutoff_d)

    dir_title = "Dir-RRT (Iters={}, p_g_samp={}, Cost={}, Dist_to_G={}) in {} sec".format(dir_itters, sample_goal_prob, len(dir_rrt_path), round(dir_distance_from_goal, 2), round(dir_rrt_time, 4))

    dir_leaves = find_leaf_nodes(dir_root_node)

    dir_paths_to_leaves = [[la for la in leaf.path] for leaf in dir_leaves]
    for lp in dir_paths_to_leaves:
        dir_rrt_e.add_line(lp)

    dir_rrt_e.add_path(dir_rrt_path)

    dir_rrt_e.add_start(start_point)
    dir_rrt_e.add_goal(goal_point)
    dir_fig = go.Figure(data=dir_rrt_e.data)
    dir_fig.update_layout(
        title=dir_title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    dir_fig.show()

#####################################################################
################################ RRT* ###############################
#####################################################################

    star_rrt_e = Environment(filename='rrt*', bounds=env_bounds, obstacles=obs)

    check_neighbor_radius = 3

    star_rrt_path, star_path_cost, star_distance_from_goal, star_root_node, star_rrt_time, star_itters  = rrt_star_iter_bound(star_rrt_e, start_point, goal_point, max_iters, check_neighbor_radius, PED, extend_length=extending_length)

    star_title = "RRT* (Iter={}, NN_Rad={}, Cost={}, Dist_to_G={}) in {} sec".format(star_itters, check_neighbor_radius, round(star_path_cost, 4), round(star_distance_from_goal, 4), round(star_rrt_time, 4))

    star_leaves = find_leaf_nodes(star_root_node)

    star_paths_to_leaves = [[la for la in leaf.path] for leaf in star_leaves]
    for lp in star_paths_to_leaves:
        star_rrt_e.add_line(lp)

    star_rrt_e.add_path(star_rrt_path)

    star_rrt_e.add_start(start_point)
    star_rrt_e.add_goal(goal_point)
    star_fig = go.Figure(data=star_rrt_e.data)
    star_fig.update_layout(
        title=star_title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    star_fig.show()

#####################################################################
################################ TEST ###############################
#####################################################################

test_bounds = (0, -0.5, 0, 1.0, 0.5, 1.3)
np.random.seed(50)

if KUKA_ENV:

    kuka = (-.1, -.1, 0, 0.1, 0.1, 0.35)
    left_door = (0.4, 0.28, 0, 0.75, 0.29, 0.82)
    right_door = (0.4, -0.29, 0, 0.75, -0.28, 0.82)
    bottom_shelf = (0.75, -0.29, 0.0, 1.05, 0.29, 0.01)
    second_shelf = (0.75, -0.29, 0.27, 1.05, 0.29, 0.28)
    third_shelf = (0.75, -0.29, 0.53, 1.05, 0.29, 0.54)
    top_shelf = (0.75, -0.29, 0.80, 1.05, 0.29, 0.81)
    left_cabinet = (0.75, 0.28, 0, 1.05, 0.29, 0.82)
    right_cabinet = (0.75, -0.29, 0, 1.05, -0.28, 0.82)
    left_pole = (-0.27, 0.31, 0, -0.2, 0.35, 1.2)
    right_pole = (-0.27, -0.35, 0, -0.2, -0.31, 1.2)

    robot_vis_obs = np.array([
        kuka,
        left_door,
        right_door,
        bottom_shelf,
        second_shelf,
        third_shelf,
        top_shelf,
        left_cabinet,
        right_cabinet,
        left_pole,
        right_pole,
    ]) 

    obs_colors = [
        'orange',    # kuka,
        'red',       # left_door,
        'red',       # right_door,
        'white',     # bottom_shelf,
        'white',     # second_shelf,
        'white',     # third_shelf,
        'white',     # top_shelf,
        'white',     # left_cabinet,
        'white',     # right_cabinet,
        'gray',      # left_pole,
        'gray',      # right_pole,
    ] 

else:
    wall = (0.4, 0.28, 0, 0.75, 0.29, 0.82)
    robot_vis_obs = wall
    obs_colors = None

max_iters = 2000
# max_iters = 500
check_neighbor_radius = 0.5

start_point_right_of_cabinet = (0.5, -0.4, 0.6)
start_point_top_of_cabinet = (0.9, 0, 1.1)

start_point = start_point_top_of_cabinet
goal_point = (0.9, 0, 0.1)

point_extraction_distance = 0.025
PED = point_extraction_distance
extending_length = 0.05
prob_sample_goal = 0.05
star_goal_sample_prob = prob_sample_goal
d_star_goal_sample_prob = prob_sample_goal
cutoff_d = 0.05


obs_boundary_value = 0.05


# max_iters = 1000
# check_neighbor_radius = 0.05 

# start_point = (0.7, 0.5, 0.5)
# goal_point = (0.5, 0, 0.65) 

# # point_extraction_distance
# PED = 0.025
# extending_length = 0.025

# cutoff_d = 0.05

# dir_goal_sample_prob = 0.05
# star_goal_sample_prob = 0.0
# d_star_goal_sample_prob = 0.5


'''
Title follows the form:
'RRT* MI ( {Max Iterations}, {Neighbor Checking Radius}, 
           {True Path Cost}, {Distance from Last Node in Path to Goal} ) 
        in {Total Time for Algorithm to Return Path} sec'
'''

if JUST_ENV:

    only_environment = Environment(filename='just env', bounds=test_bounds, obstacles=robot_vis_obs, obstacle_colors=obs_colors, obstacle_boundaries=False, obstacle_boundary_value=obs_boundary_value)
    
    env_title = "Environment for RRT"

    only_environment.add_start(start_point)
    only_environment.add_goal(goal_point)
    env_fig = go.Figure(data=only_environment.data)
    env_fig.update_layout(
        title=env_title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    env_fig.show()


##### PLAIN RRT #####

if RRT:

    rrt_e = Environment(filename='plain rrt', bounds=test_bounds, obstacles=robot_vis_obs, obstacle_colors=obs_colors, obstacle_boundaries=False, obstacle_boundary_value=obs_boundary_value)

    rrt_path, path_cost, distance_from_goal, root_node, rrt_time, iters = rrt(rrt_e, start_point, goal_point, max_iters, PED, extend_length=extending_length, cutoff_distance=cutoff_d)

    title = "RRT (Iter={}, Cost={}, Distance={}) in {} sec".format(iters, path_cost, distance_from_goal, rrt_time)

    leaves = find_leaf_nodes(root_node)

    paths_to_leaves = [[la for la in leaf.path] for leaf in leaves]
    for lp in paths_to_leaves:
        rrt_e.add_line(lp)
        # for node_on_path in lp:
            # rrt_e.add_search_space_node(node_on_path)

    rrt_e.add_path(rrt_path)

    rrt_e.add_start(start_point)
    rrt_e.add_goal(goal_point)
    fig = go.Figure(data=rrt_e.data)
    fig.update_layout(
        title=title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    fig.show()

##### DIRECTED RRT #####

if DRRT:

    dir_rrt_e = Environment(filename='dir rrt', bounds=test_bounds, obstacles=robot_vis_obs, obstacle_colors=obs_colors, obstacle_boundaries=False, obstacle_boundary_value=obs_boundary_value)

    dir_rrt_path, dir_path_cost, dir_distance_from_goal, dir_root_node, dir_rrt_time, dir_itters = directed_rrt(dir_rrt_e, start_point, goal_point, max_iters, PED, prob_sample_goal=dir_goal_sample_prob, extend_length=extending_length, cutoff_distance=cutoff_d)

    dir_title = "Dir-RRT(Iter={}, p_g_samp={}, Cost={}, Distance={}) in {} sec".format(dir_itters, dir_goal_sample_prob, dir_path_cost, dir_distance_from_goal, dir_rrt_time)

    dir_leaves = find_leaf_nodes(dir_root_node)

    dir_paths_to_leaves = [[la for la in leaf.path] for leaf in dir_leaves]
    for lp in dir_paths_to_leaves:
        dir_rrt_e.add_line(lp)
        # for node_on_path in lp:
            # dir_rrt_e.add_search_space_node(node_on_path)

    dir_rrt_e.add_path(dir_rrt_path)

    dir_rrt_e.add_start(start_point)
    dir_rrt_e.add_goal(goal_point)
    dir_fig = go.Figure(data=dir_rrt_e.data)
    dir_fig.update_layout(
        title=dir_title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    dir_fig.show()

##### RRT STAR #####

if SRRT:

    star_rrt_e = Environment(filename='rrt*', bounds=test_bounds, obstacles=robot_vis_obs, obstacle_colors=obs_colors, obstacle_boundaries=True, obstacle_boundary_value=obs_boundary_value, draw_boundaries=False)

    star_rrt_path, star_path_cost, star_distance_from_goal, star_root_node, star_rrt_time, star_itters  = new_rrt_star_iter_bound(star_rrt_e, start_point, goal_point, max_iters, check_neighbor_radius, PED, extend_length=extending_length)

    star_title = "RRT* (Iter={}, NN_Rad={}, Cost={}, Distance={}) in {} sec".format(star_itters, check_neighbor_radius, star_path_cost, star_distance_from_goal, star_rrt_time)

    # for n in star_rrt_path:
    #     star_rrt_e.add_search_space_node(n)

    star_leaves = find_leaf_nodes(star_root_node)

    star_paths_to_leaves = [[la for la in leaf.path] for leaf in star_leaves]
    for lp in star_paths_to_leaves:
        star_rrt_e.add_line(lp)

    star_rrt_e.add_path(star_rrt_path)
    
    # smoothed_path = RRTPathProcessor.interpolate_path(star_rrt_path, num_points=100)

    # star_rrt_e.add_path(smoothed_path, 'rgba(152, 2, 253, 1.0)')

    star_rrt_e.add_start(start_point)
    star_rrt_e.add_goal(goal_point)
    star_fig = go.Figure(data=star_rrt_e.data)
    star_fig.update_layout(
        title=star_title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    star_fig.show()
