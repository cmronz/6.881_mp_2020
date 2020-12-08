import numpy as np

def dist_between_points(p1, p2):
    ''' Calculates Euclidean distance between two points, p2 & p1 '''
    return np.linalg.norm(np.array(p2) - np.array(p1))

def dist_between_np_array_points(p1, p2):
    ''' Calculates Euclidean distance between two points, p2 & p1 '''
    return np.linalg.norm(p2 - p1)

def extract_points_from_line(start, end, max_distance):
    ''' extract points increments of step_size away from eachother along the line from start to end '''
    points = list()
    distance = dist_between_points(start, end)
    num_points = int(np.ceil(distance / max_distance))
    if (num_points - 1) > 0:
        step_size = distance / (num_points - 1)
        for i in range(num_points):
            points.append(steer(start, end, step_size * i))
    return points


def close_enough(p1, p2, epsilon):
    ''' checks if p2 is withing epsilon on p1 '''
    return dist_between_points(p1, p2) <= epsilon


def find_nearest_neighbor(xyz, nodes):
    ''' Returns the node in nodes (iterable) closest to location xy '''
    point = np.array(xyz)
    mind = np.inf
    nn = None
    for n in nodes:
        d = dist_between_np_array_points(n.xyz, point)
        if d < mind:
            mind = d
            nn = n
    return nn

def find_nearest_neighbor_with_distance(xyz, nodes):
    ''' Returns the node in nodes (iterable) closest to location xy '''
    point = np.array(xyz)
    mind = np.inf
    nn = None
    for n in nodes:
        d = dist_between_np_array_points(n.xyz, point)
        if d < mind:
            mind = d
            nn = n
    return nn, mind

def nn_insertion_sort(nns, new_neighbor):
    cost = new_neighbor[1]
    ind = len(nns)
    for i in range(len(nns)):
        if cost < nns[i][1]:
            ind = i
            break
    nns.insert(ind, new_neighbor)

def find_nearest_neighbors_within_radius(xyz, nodes, radius):
    ''' Returns the nodes (iterable) withing radius of xyz to location xy '''
    point = np.array(xyz)
    nn = list()
    for n in nodes:
        d = dist_between_np_array_points(n.xyz, xyz)
        if d < radius:
            cost = n.cost #path_cost(n) + d
            nn.append((n, cost))
    nn.sort(key = lambda k: k[1])
    return nn

def find_nearest_neighbors_within_radius_distance_to_point_included(xyz, nodes, radius):
    ''' Returns the nodes (iterable) withing radius of xyz to location xy '''
    point = np.array(xyz)
    nn = list()
    for n in nodes:
        d = dist_between_np_array_points(n.xyz, xyz)
        if d < radius:
            cost = n.cost + d
            nn_insertion_sort(nn, (n, cost))
    return nn

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

def form_full_tree_with_piecewise_pairings(node):
    all_lines = list()
    for child in node.children:
        all_lines.append((node.xyz, child.xyz))
        all_lines.extend(form_full_tree_with_piecewise_pairings(child))
    return all_lines

def find_leaf_nodes(node):
    leaves = list()
    for child in node.children:
        if not child.children:
            leaves.append(child)
        else:
            leaves.extend(find_leaf_nodes(child))
    return leaves
