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
radius = 3 # radius of robot moving (used in collision detection)

scattered_small_boxes = np.array(
    [(2, 2, 2, 4, 4, 4), (2, 2, 6, 4, 4, 8), (2, 6, 2, 4, 8, 4), (6, 6, 2, 8, 8, 4),
     (6, 2, 2, 8, 4, 4), (6, 2, 6, 8, 4, 8), (2, 6, 6, 4, 8, 8), (6, 6, 6, 8, 8, 8)])

one_big_box = np.array([(2.75, 2.75, 2.75, 7.75, 7.75, 7.75)])

wall_with_gap = np.array([(1.5, 5, 1, 2.5, 10, 10), (1.5, 1, 1, 2.5, 4, 10)])

cabinet = np.array([(.5, .25, 0, 1.05, .45, .75)])

obs = cabinet


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

# test_bounds = (-1, -1, -1, 1, 1, 1)
# test_bounds = (0, -0.1, -0.1, 1, 0.75, 0.6)  No Cabinet
test_bounds = (-.3, -0.1, -0.1, 1.1, 0.75, 0.8)
t_e = Environment(filename='rrt*', bounds=test_bounds, obstacles=obs)

np.random.seed(50)

max_it = 1000
check_neighbor_radius = 0.15 

# start_point = (-0.5, 0, 0.1) # didnt work
# start_point = (0.7, 0.5, 0.1) # start for tuning params
start_point = (0.7, 0.5, 0.5)

goal_point = (0.5, 0, 0.65) 

# point_extraction_distance
PED = 0.1
extend_length = 0.05


'''
Title follows the form:
'RRT* MI ( {Max Iterations}, {Neighbor Checking Radius}, 
           {True Path Cost}, {Distance from Last Node in Path to Goal} ) 
        in {Total Time for Algorithm to Return Path} sec'
'''

test_path, path_cost, distance_from_goal, root_node, star_rrt_time  = rrt_star_iter_bound(t_e, start_point, radius, goal_point, max_it, check_neighbor_radius, PED, extend_length)

test_title = "RRT* MIn ({}, {}, {}, {}) in {} sec".format(max_it, check_neighbor_radius, path_cost, distance_from_goal, star_rrt_time)

leaves = find_leaf_nodes(root_node)

paths_to_leaves = [[la for la in leaf.path] for leaf in leaves]
for lp in paths_to_leaves:
    t_e.add_line(lp)
    # for node_on_path in lp:
    	# t_e.add_search_space_node(node_on_path)

t_e.add_path(test_path)

t_e.add_start(start_point)
t_e.add_goal(goal_point)
t_fig = go.Figure(data=t_e.data)
t_fig.update_layout(
    title=test_title,
    font_family="Courier New",
    font_color="blue",
    title_font_family="Times New Roman",
    title_font_color="red",
    legend_title_font_color="green"
)
# py.plot(star_fig, filename=star_e_title, auto_open=True)
# print('Path found of length: {}\n\n Here is the path ==> \n{}'.format(len(test_path), test_path))
t_fig.show()

#####################################################################
####################### Perftecly-Directed RRT ######################
#####################################################################

# dir_e = Environment('directed rrt', bounds=environment_bounds)


# dir_path, dir_lines, dir_rrt_time = directed_rrt(environment_bounds, start_point, radius, goal_point, prob_sample_goal=1.0)
# for dline in dir_lines:
#     dir_e.add_line(dline)

# dir_e_title = "Perfectly-Directed RRT (Prob Goal Sample = {}) Path Length = {}, Time to Complete = {} sec".format(1.0, len(dir_path), dir_rrt_time)
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
