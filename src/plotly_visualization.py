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

star_e = Environment('rrt*', bounds=environment_bounds)

np.random.seed(50)

prob_check_sol = 0.01
max_it = 500
check_neighbor_radius = 3

star_path, path_cost, distance_from_goal, root_node, star_rrt_time  = rrt_star_iter_bound(environment_bounds, start_point, radius, goal_point, max_it, check_neighbor_radius)

star_e_title = "RRT* (Max Iterartions = {}, Neighbor Radius = {}), Path Length = {}, Distance to Goal From Last Node = {}, Time to Complete = {} sec".format(max_it, check_neighbor_radius, path_cost, distance_from_goal, star_rrt_time)

full_tree = form_full_tree_with_piecewise_pairings(root_node)
for segment in full_tree:
    star_e.add_line(segment)

star_e.add_path(star_path)
star_e.add_start(start_point)
star_e.add_goal(goal_point)
star_fig = go.Figure(data=star_e.data)
star_fig.update_layout(
    title=star_e_title,
    font_family="Courier New",
    font_color="blue",
    title_font_family="Times New Roman",
    title_font_color="red",
    legend_title_font_color="green"
)
star_fig.show()

# star_cost = np.inf
# star_path = None
# star_rrt_time = None
# for i in range(50):
#     best_path, star_lines, best_time, min_cost = rrt_star(environment_bounds, start_point, radius, goal_point, solution_check_prob=prob_check_sol)
#     if min_cost < star_cost:
#         star_cost = min_cost
#         star_path = best_path
#         star_rrt_time = best_time
# for dline in star_lines:
#     star_e.add_line(dline)

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
