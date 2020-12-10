import plotly as py

from utils import *
from collision import *
from rtree import index 
from plotly import graph_objs as go

#####################################################################
############################ Environment ############################
#####################################################################

def create_obstacle_boundary(obstacle, obstacle_boundary_value):
    ''' Creates dimensions for an obstacle '''
    
    dimensions = int(len(obstacle) / 2)

    boundary_min = obstacle[:dimensions] - obstacle_boundary_value
    boundary_max = obstacle[dimensions:] + obstacle_boundary_value

    boundary = np.hstack((boundary_min, boundary_max))

    return tuple(boundary)

class Environment:
    def __init__(self, filename='rrt3d', bounds=None, obstacles=None, obstacle_colors=None, obstacle_boundaries=False, obstacle_boundary_value=0.0, draw_boundaries=False):
        ''' 
        @param bounds = 6-element tuple (minx, miny, minz, maxx, maxy, maxz)
        '''
        self.filename = 'visuals/' + filename + '.html'
        self.bounds = bounds

        # holds obstacles/paths to be plotted 
        self.data = []

        self.layout = {'title': 'Plot',
                       'showlegend': False
                       }

        self.fig = {'data': self.data,
                    'layout': self.layout}

        properties = index.Property()
        properties.dimension = 3
        if obstacles is None:
            self.obstacles = index.Index(interleaved=True, properties=properties)
        else:
            self.add_obstacles(obstacles, obstacle_colors)

            # Check if should check for collisions with actual objects or with a buffer around the objects
            if obstacle_boundaries:
                boundaries = list()
                for obs in obstacles:
                    boundaries.append(create_obstacle_boundary(obs, obstacle_boundary_value))
                boundaries = np.array(boundaries)
                if draw_boundaries:
                    self.add_boundary_obstacles(boundaries)
                self.obstacles = index.Index(ob_gen(boundaries), interleaved=True, properties=properties)
            else:
                self.obstacles = index.Index(ob_gen(obstacles), interleaved=True, properties=properties)

    def is_point_obstacle_free(self, point):
        return self.obstacles.count(point) == 0


    def is_line_obstacle_free(self, x_near, x_new, max_d):
        points_along_line = extract_points_from_line(x_near, x_new, max_d)
        for point in points_along_line:
            if not self.is_point_obstacle_free(point):
                return False
        return True

    def add_obstacles(self, obstacles, colors=None):
        ''' Assumes obstacle is a np array of obstacles'''
        if colors:
            for i in range(len(colors)):
                o_color = colors[i]
                obstacle = obstacles[i]
                obs = go.Mesh3d(
                    x=[obstacle[0], obstacle[0], obstacle[3], obstacle[3], obstacle[0], obstacle[0], obstacle[3], obstacle[3]],
                    y=[obstacle[1], obstacle[4], obstacle[4], obstacle[1], obstacle[1], obstacle[4], obstacle[4], obstacle[1]],
                    z=[obstacle[2], obstacle[2], obstacle[2], obstacle[2], obstacle[5], obstacle[5], obstacle[5], obstacle[5]],
                    i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                    j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                    k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                    color=o_color,
                    opacity=0.70
                )
                self.data.append(obs)
        else: 
            for obstacle in obstacles:
                obs = go.Mesh3d(
                    x=[obstacle[0], obstacle[0], obstacle[3], obstacle[3], obstacle[0], obstacle[0], obstacle[3], obstacle[3]],
                    y=[obstacle[1], obstacle[4], obstacle[4], obstacle[1], obstacle[1], obstacle[4], obstacle[4], obstacle[1]],
                    z=[obstacle[2], obstacle[2], obstacle[2], obstacle[2], obstacle[5], obstacle[5], obstacle[5], obstacle[5]],
                    i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                    j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                    k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                    color='rgba(178, 34, 34, 1.0)',
                    opacity=0.70
                )
                self.data.append(obs)

    def add_boundary_obstacles(self, obstacles):
        ''' Assumes obstacle is a np array of obstacles'''

        for obstacle in obstacles:
            obs = go.Mesh3d(
                x=[obstacle[0], obstacle[0], obstacle[3], obstacle[3], obstacle[0], obstacle[0], obstacle[3], obstacle[3]],
                y=[obstacle[1], obstacle[4], obstacle[4], obstacle[1], obstacle[1], obstacle[4], obstacle[4], obstacle[1]],
                z=[obstacle[2], obstacle[2], obstacle[2], obstacle[2], obstacle[5], obstacle[5], obstacle[5], obstacle[5]],
                i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                color='rgba(195, 253, 253, 0.5)',
                opacity=0.70
            )
            self.data.append(obs)

    def add_path(self, path, path_color='red'):
        ''' path is a list of (x, y, z) points '''
        
        x, y, z = [], [], []
        for point in path:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        trace = go.Scatter3d(x=x, y=y, z=z, line=dict(color=path_color, width=4), mode="lines")
        self.data.append(trace)

    def add_line(self, endpoints):
        ''' adds a line to the environment '''
        
        x, y, z = [], [], []
        for point in endpoints:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        trace = go.Scatter3d(x=x, y=y, z=z, line=dict(color='rgba(0, 0, 139, 0.15)', width=4), mode="lines")
        self.data.append(trace)       

    def add_start(self, start_point):
        trace = go.Scatter3d(
            x=[start_point[0]],
            y=[start_point[1]],
            z=[start_point[2]],
            line=dict(
                color="orange",
                width=10
            ),
            mode="markers"
        )
        self.data.append(trace)

    def add_goal(self, goal_point):
        trace = go.Scatter3d(
            x=[goal_point[0]],
            y=[goal_point[1]],
            z=[goal_point[2]],
            line=dict(
                color="green",
                width=10
            ),
            mode="markers"
        )
        self.data.append(trace)

    def add_search_space_node(self, ss_node, mark_color="black"):
        trace = go.Scatter3d(
            x=[ss_node[0]],
            y=[ss_node[1]],
            z=[ss_node[2]],
            line=dict(
                color=mark_color,
                width=1
            ),
            mode="markers"
        )
        self.data.append(trace)


    # Still Need to modify RRT/environemnt to use RTree for collision detection and tree management
    def plot_tree_3d(self, trees):
        """
        Plot 3D trees
        :param trees: trees to plot
        """
        for i, tree in enumerate(trees):
            for start, end in tree.E.items():
                if end is not None:
                    trace = go.Scatter3d(
                        x=[start[0], end[0]],
                        y=[start[1], end[1]],
                        z=[start[2], end[2]],
                        line=dict(
                            color=colors[i]
                        ),
                        mode="lines"
                    )
                    self.data.append(trace)

        self.data.append(trace)

    def draw(self, auto_open=True):
        """
        Render the plot to a file
        """
        py.offline.plot(self.fig, filename=self.filename, auto_open=auto_open)
