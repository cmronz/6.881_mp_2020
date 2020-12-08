import chart_studio.plotly as py

from plotly import graph_objs as go
from environment import Environment
from mp_algorithms import *

#####################################################################
########################### Visualization ###########################
#####################################################################

environment_bounds = (0, 0, 0, 11, 11, 11)
start_point = (0, 0, 0)  # starting location
goal_point = (10, 10, 10)  # goal location

# TODO (Make this a function of environment for buffer objects)
radius = 3 # radius of robot moving (used in collision detection)

scattered_small_boxes = np.array(
    [(2, 2, 2, 4, 4, 4), (2, 2, 6, 4, 4, 8), (2, 6, 2, 4, 8, 4), (6, 6, 2, 8, 8, 4),
     (6, 2, 2, 8, 4, 4), (6, 2, 6, 8, 4, 8), (2, 6, 6, 4, 8, 8), (6, 6, 6, 8, 8, 8)])

one_big_box = np.array([(2.75, 2.75, 2.75, 7.75, 7.75, 7.75)])

wall_with_gap = np.array([(1.5, 5, 1, 2.5, 10, 10), (1.5, 1, 1, 2.5, 4, 10)])

obs = scattered_small_boxes


#####################################################################
############################# Plain RRT #############################
#####################################################################

# # path, lines = rrt(environment_bounds, start_point, radius, goal_point, True)
# path, lines, rrt_time = rrt(environment_bounds, start_point, radius, goal_point)

# e = Environment('rrt', bounds=environment_bounds)

# for line in lines:
#     e.add_line(line)

# e_title = "Plain RRT Path Length = {}, Time to Complete = {} sec".format(len(path), rrt_time)
# e.add_path(path)
# e.add_start(start_point)
# e.add_goal(goal_point)

# fig = go.Figure(data=e.data)

# fig.update_layout(
#     title=e_title,
#     font_family="Courier New",
#     font_color="blue",
#     title_font_family="Times New Roman",
#     title_font_color="red",
#     legend_title_font_color="green"
# )

# fig.show()

#####################################################################
############################ Directed RRT ###########################
#####################################################################

# dir_e = Environment('directed rrt', bounds=environment_bounds)


# dir_path, dir_lines, dir_rrt_time = directed_rrt(environment_bounds, start_point, radius, goal_point)
# for dline in dir_lines:
#     dir_e.add_line(dline)

# dir_e_title = "Directed RRT (Prob Goal Sample = {}) Path Length = {}, Time to Complete = {} sec".format(0.05, len(dir_path), dir_rrt_time)
# dir_e.add_path(dir_path)
# dir_e.add_start(start_point)
# dir_e.add_goal(goal_point)
# dir_fig = go.Figure(data=dir_e.data)
# dir_fig.update_layout(
#     title=dir_e_title,
#     font_family="Courier New",
#     font_color="blue",
#     title_font_family="Times New Roman",
#     title_font_color="red",
#     legend_title_font_color="green"
# )
# dir_fig.show()

#####################################################################
################################ RRT* ###############################
#####################################################################

# star_e = Environment(filename='rrt*', bounds=environment_bounds, obstacles=obs)

# np.random.seed(50)

# prob_check_sol = 0.01
# max_it = 250
# check_neighbor_radius = 3

# star_path, path_cost, distance_from_goal, root_node, star_rrt_time  = rrt_star_iter_bound(star_e, start_point, radius, goal_point, max_it, check_neighbor_radius)

# '''
# Title follows the form:
# 'RRT* MI ( {Max Iterations}, {Neighbor Checking Radius}, 
#            {True Path Cost}, {Distance from Last Node in Path to Goal} ) 
#         in {Total Time for Algorithm to Return Path} sec'
# '''
# star_e_title = "RRT* MI ({}, {}, {}, {}) in {} sec".format(max_it, check_neighbor_radius, path_cost, distance_from_goal, star_rrt_time)

# # print("Last Node: {}".format(star_path[-1]))

# leaves = find_leaf_nodes(root_node)

# paths_to_leaves = [[la for la in leaf.path] for leaf in leaves]
# for lp in paths_to_leaves:
#     star_e.add_line(lp)

# star_e.add_path(star_path)
# star_e.add_start(start_point)
# star_e.add_goal(goal_point)
# star_fig = go.Figure(data=star_e.data)
# star_fig.update_layout(
#     title=star_e_title,
#     font_family="Courier New",
#     font_color="blue",
#     title_font_family="Times New Roman",
#     title_font_color="red",
#     legend_title_font_color="green"
# )
# # py.plot(star_fig, filename=star_e_title, auto_open=True)
# star_fig.show()

#####################################################################
################################ TEST ###############################
#####################################################################

RRT = True
DRRT = True
SRRT = True
DSRRT = True

np.random.seed(50)

cabinet = np.array([(.5, .25, 0, 1.05, .45, .75)])
kuka = np.array([(-.1, -.1, 0, .1, .1, .75)])

robot_vis_obs = cabinet

test_bounds = (-.3, -0.1, -0.1, 1.1, 0.75, 0.8)
rrt_e = Environment(filename='plain rrt', bounds=test_bounds, obstacles=robot_vis_obs)
dir_rrt_e = Environment(filename='dir rrt', bounds=test_bounds, obstacles=robot_vis_obs)
star_rrt_e = Environment(filename='rrt*', bounds=test_bounds, obstacles=robot_vis_obs)
d_star_rrt_e = Environment(filename='dir rrt*', bounds=test_bounds, obstacles=robot_vis_obs)


max_iters = 1500
check_neighbor_radius = 0.05 

start_point = (0.7, 0.5, 0.5)
goal_point = (0.5, 0, 0.65) 

# point_extraction_distance
PED = 0.025
extending_length = 0.025

cutoff_d = 0.05

dir_goal_sample_prob = 0.05
star_goal_sample_prob = 0.0
d_star_goal_sample_prob = 0.5


'''
Title follows the form:
'RRT* MI ( {Max Iterations}, {Neighbor Checking Radius}, 
           {True Path Cost}, {Distance from Last Node in Path to Goal} ) 
        in {Total Time for Algorithm to Return Path} sec'
'''

##### PLAIN RRT #####

if RRT:

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

    star_rrt_path, star_path_cost, star_distance_from_goal, star_root_node, star_rrt_time, star_itters  = rrt_star_iter_bound(star_rrt_e, start_point, goal_point, max_iters, check_neighbor_radius, PED, prob_sample_goal=star_goal_sample_prob, extend_length=extending_length)

    star_title = "RRT* (Iter={}, NN_Rad={}, p_g_samp={}, Cost={}, Distance={}) in {} sec".format(star_itters, check_neighbor_radius, star_goal_sample_prob, star_path_cost, star_distance_from_goal, star_rrt_time)

    # for n in star_rrt_path:
    #     star_rrt_e.add_search_space_node(n)

    star_leaves = find_leaf_nodes(star_root_node)

    star_paths_to_leaves = [[la for la in leaf.path] for leaf in star_leaves]
    for lp in star_paths_to_leaves:
        star_rrt_e.add_line(lp)
        # for node_on_path in lp:
            # star_rrt_e.add_search_space_node(node_on_path)

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

##### DRRT STAR #####

if DSRRT:

    d_star_rrt_path, d_star_path_cost, d_star_distance_from_goal, d_star_root_node, d_star_rrt_time, d_star_itters  = rrt_star_iter_bound(d_star_rrt_e, start_point, goal_point, max_iters, check_neighbor_radius, PED, prob_sample_goal=d_star_goal_sample_prob, extend_length=extending_length)

    d_star_title = "Dir-RRT* (Iter={}, NN_Rad={}, p_g_samp={}, Cost={}, Distance={}) in {} sec".format(d_star_itters, check_neighbor_radius, d_star_goal_sample_prob, d_star_path_cost, d_star_distance_from_goal, d_star_rrt_time)

    # for n in d_star_rrt_path:
    #     d_star_rrt_e.add_search_space_node(n)

    d_star_leaves = find_leaf_nodes(d_star_root_node)

    d_star_paths_to_leaves = [[la for la in leaf.path] for leaf in d_star_leaves]
    for lp in d_star_paths_to_leaves:
        d_star_rrt_e.add_line(lp)
        # for node_on_path in lp:
            # d_star_rrt_e.add_search_space_node(node_on_path)

    d_star_rrt_e.add_path(star_rrt_path)

    d_star_rrt_e.add_start(start_point)
    d_star_rrt_e.add_goal(goal_point)
    d_star_fig = go.Figure(data=d_star_rrt_e.data)
    d_star_fig.update_layout(
        title=d_star_title,
        font_family="Courier New",
        font_color="blue",
        title_font_family="Times New Roman",
        title_font_color="red",
        legend_title_font_color="green"
    )
    d_star_fig.show()
