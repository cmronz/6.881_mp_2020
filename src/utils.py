import numpy as np

def dist_between_points(p1, p2):
    ''' Calculates Euclidean distance between two points, p2 & p1 '''
    return np.linalg.norm(np.array(p2) - np.array(p1))

def close_enough(p1, p2, epsilon):
    ''' checks if p2 is withing epsilon on p1 '''
    return dist_between_points(p1, p2) <= epsilon


def find_nearest_neighbor(xyz, nodes):
    ''' Returns the node in nodes (iterable) closest to location xy '''
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


def find_nearest_neighbors_within_radius(xyz, nodes, radius):
    ''' Returns the nodes (iterable) withing radius of xyz to location xy '''
    def dist(point, node):
        dif = node.xyz - point
        return dif.dot(dif)
    point = np.array(xyz)

    nn = list()
    for n in nodes:
        d = dist(point, n)
        if d < radius:
            cost = path_cost(n) + d
            nn.append((n, cost))
    nn.sort(key = lambda k: k[1])
    return nn

def path_cost(node):
    ''' Returns the path incurred by traversing from the root node to the node passed in '''
    if node.parent is None:
        return 0
    else:
        return node.parent.cost + dist_between_points(node.parent.xyz, node.xyz)


def sample_cube(bounds):
    ''' Returns a random point (x, y, z) s.t. minx < x < maxx, miny < y < maxy, minz < z < maxz, given bounds=[minx, miny, minz, maxx, maxy, maxz] '''
    minx, miny, minz, maxx, maxy, maxz = bounds
    r = np.random.uniform(low=0.0, high=1.0, size=3)
    return (minx + (maxx-minx)*r[0], miny + (maxy-miny)*r[1], minz + (maxz-minz)*r[2])

def steer(start, goal, d):
    ''' Return a point in the direction of the goal, that is d (distance) away from start '''
    start, end = np.array(start), np.array(goal)
    v = end - start
    u = v / (np.sqrt(np.sum(v ** 2)))
    steered_point = start + u * d
    return tuple(steered_point)

def add_to_display_dictionary(d, parent_xyz, child_loc):
    parent_loc = tuple(parent_xyz)
    if parent_loc not in d:
        d[parent_loc] = set()
    d[parent_loc].add(child_loc)
    return

def remove_from_display_dictionary(d, node):
    parent_loc = tuple(node.parent.xyz)
    node_loc = tuple(node.xyz)
    d[parent_loc].remove(node_loc)
    return

def form_parent_child_tuples(node_dict):
    display_lines = list()
    for key in node_dict:
        for child in node_dict[key]:
            display_lines.append((key.xyz, child.xyz))
    return display_lines



