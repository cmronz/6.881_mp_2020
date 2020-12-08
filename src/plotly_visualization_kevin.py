import chart_studio.plotly as py

from plotly import graph_objs as go
from environment import Environment
from mp_algorithms import *

JUST_ENV = False
RRT = False
DRRT = False
SRRT = False
DSRRT = False

JUST_ENV = True
# RRT = True
# DRRT = True
SRRT = True
# DSRRT = True

np.random.seed(50)

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

pad_amt = 0.05
obstacles = np.array([
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

test_bounds = (0, -0.5, 0, 1.0, 0.5, 1.3)

random_seed = 50

max_iters = 2000
check_neighbor_radius = 0.05

# start_point = (0.5, 0.4, 0.6)
start_point = (0.9, 0, 1.1)
goal_point = (0.9, 0, 0.1)

point_extraction_distance = 0.025
extend_length = 0.025
prob_sample_goal = 0.05
cutoff_d = 0.05

# specific to local env
obs_boundary_value = 0.025
PED = point_extraction_distance
extending_length = extend_length
star_goal_sample_prob = prob_sample_goal

only_environment = Environment(filename='plain rrt', bounds=test_bounds, obstacles=obstacles, obstacle_colors=['green'], obstacle_boundaries=True, obstacle_boundary_value=obs_boundary_value)
rrt_e = Environment(filename='plain rrt', bounds=test_bounds, obstacles=obstacles, obstacle_boundaries=True, obstacle_boundary_value=obs_boundary_value)
dir_rrt_e = Environment(filename='dir rrt', bounds=test_bounds, obstacles=obstacles, obstacle_boundaries=True, obstacle_boundary_value=obs_boundary_value)
star_rrt_e = Environment(filename='rrt*', bounds=test_bounds, obstacles=obstacles, obstacle_boundaries=True, obstacle_boundary_value=obs_boundary_value)
d_star_rrt_e = Environment(filename='dir rrt*', bounds=test_bounds, obstacles=obstacles, obstacle_boundaries=True, obstacle_boundary_value=obs_boundary_value)


'''
Title follows the form:
'RRT* MI ( {Max Iterations}, {Neighbor Checking Radius}, 
           {True Path Cost}, {Distance from Last Node in Path to Goal} ) 
        in {Total Time for Algorithm to Return Path} sec'
'''

if JUST_ENV:
    
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

    star_title = "RRT* (Iter={}, NN_Rad={}, Cost={}, Distance={}) in {} sec".format(star_itters, check_neighbor_radius, star_path_cost, star_distance_from_goal, star_rrt_time)

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
