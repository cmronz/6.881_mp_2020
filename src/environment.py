import plotly as py

from plotly import graph_objs as go

#####################################################################
############################ Environment ############################
#####################################################################

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
        ''' adds a line to the environment '''
        
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

    # def add_goal(self, goal_point):
    #     trace = go.Scatter3d(
    #         x=[goal_point[0]],
    #         y=[goal_point[1]],
    #         z=[goal_point[2]],
    #         line=dict(
    #             color="green",
    #             width=10
    #         ),
    #         mode="markers"
    #     )
    #     self.data.append(trace)


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
