import time
import plotly as py
import numpy as np

from shapely.geometry import Point
from plotly import graph_objs as go
from itertools import tee

import numpy as np


def pairwise(iterable):
    """
    Pairwise iteration over iterable
    :param iterable: iterable
    :return: s -> (s0,s1), (s1,s2), (s2, s3), ...
    """
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

def dist_between_points(a, b):
    """
    Return the Euclidean distance between two points
    :param a: first point
    :param b: second point
    :return: Euclidean distance between a and b
    """
    distance = np.linalg.norm(np.array(b) - np.array(a))
    return distance

# %load_ext line_profiler
class Node(object):
    def __init__(self, point, children=None, parent=None):
        self.xyz = np.array(point)
        self.parent = parent
        self.children = children if children else []
    
    @property
    def all_descendents(self):
        """Return an iterable of all the descendants from this node (i.e. children + childrens' children + ...)"""
        yield self
        for c in self.children:
            for n in c.all_descendents:
                yield n
    @property
    def point(self):
        return Point(self.xyz)
    
    @property
    def all_ancestors(self):
        """Return an iterable of all the ancestors this node (i.e. parent + parent's parent + ...),
        starting from this node."""
        yield self
        if self.parent is not None:
            for a in self.parent.all_ancestors:
                yield a
        
    @property
    def path(self):
        """Returns a path from the root to this node, expressed as an iterable of tuples of x-y coordinates"""
        path = list(self.all_ancestors)
        for n in reversed(path):
            yield tuple(n.xyz)
    
    def __repr__(self):
        return "<Node xyz: %s>" % (self.xyz)

def close_enough(p1, p2, epsilon):
    ''' checks if p2 is withing epsilon on p1 '''
    return dist_between_points(p1, p2) <= epsilon


def find_nearest_neighbor(xyz, nodes):
    """Returns the node in nodes (iterable) closest to location xy"""
    def dist(point, node):
        dif = node.xyz - point
        return dif.dot(dif)
    point = np.array(xyz)
    mind = np.inf
    nn = None
    for n in nodes:
        d = dist(point, n)
        if d < mind:
            mind = d
            nn = n
    return nn

def sample_cube(bounds):
    """Returns a random point (x, y, z) s.t. minx < x < maxx, miny < y < maxy, minz < z < maxz, given bounds=[minx, miny, minz, maxx, maxy, maxz]"""
    minx, miny, minz, maxx, maxy, maxz = bounds
    r = np.random.uniform(low=0.0, high=1.0, size=3)
    return (minx + (maxx-minx)*r[0], miny + (maxy-miny)*r[1], minz + (maxz-minz)*r[2])



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
            return path, lines_to_display

        if steps >= 1000 and time_out:
            return list(), lines_to_display

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
            return path, lines_to_display

        if steps >= 1000 and time_out:
            return list(), lines_to_display

class Environment:
    def __init__(self, filename='rrt3d', bounds=None):
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

    def add_obstacles(self, obstacles):
        ''' Assumes obstacle is a '''

        for obstacle in obstacles:
            obs = go.Mesh3d(
                x=[obstacle[0], obstacle[0], obstacle[3], obstacle[3], obstacle[0], obstacle[0], obstacle[3], obstacle[3]],
                y=[obstacle[1], obstacle[4], obstacle[4], obstacle[1], obstacle[1], obstacle[4], obstacle[4], obstacle[1]],
                z=[obstacle[2], obstacle[2], obstacle[2], obstacle[2], obstacle[5], obstacle[5], obstacle[5], obstacle[5]],
                i=[7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                j=[3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                k=[0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
                color='purple',
                opacity=0.70
            )
            self.data.append(obs)

    def add_path(self, path):
        ''' path is a list of (x, y, z) points '''
        
        x, y, z = [], [], []
        for point in path:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        trace = go.Scatter3d(x=x, y=y, z=z, line=dict(color="red", width=4), mode="lines")
        self.data.append(trace)

    def add_line(self, endpoints):
        ''' path is a tuple of 2 (x, y, z) points '''
        
        x, y, z = [], [], []
        for point in endpoints:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        trace = go.Scatter3d(x=x, y=y, z=z, line=dict(color="darkblue", width=4), mode="lines")
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

environment_bounds = (0, 0, 0, 15, 15, 15)
start_point = (0, 0, 0)  # starting location
goal_point = (10, 10, 10)  # goal location
radius = 3 # radius of robot moving (used in collision detection)


path, lines = rrt(environment_bounds, start_point, radius, goal_point, True)
# path, lines = rrt(environment_bounds, start_point, radius, goal_point)

e = Environment('rrt', bounds=environment_bounds)

# for line in lines:
#     e.add_line(line)

# e.add_path(path)
# e.add_start(start_point)
# e.add_goal(goal_point)

# fig = go.Figure(data=e.data)

# fig.show()


dir_e = Environment('directed rrt', bounds=environment_bounds)

dir_path, dir_lines = directed_rrt(environment_bounds, start_point, radius, goal_point)
for dline in dir_lines:
    dir_e.add_line(dline)
dir_e.add_path(dir_path)
dir_e.add_start(start_point)
dir_e.add_goal(goal_point)
dir_fig = go.Figure(data=dir_e.data)
dir_fig.show()


# # plot
# plot = Plot("rrt_3d_with_random_obstacles")
# plot.plot_tree(X, rrt.trees)
# if path is not None:
#     plot.plot_path(X, path)
# plot.plot_obstacles(X, Obstacles)
# plot.plot_start(X, x_init)
# plot.plot_goal(X, x_goal)
# plot.draw(auto_open=True)
