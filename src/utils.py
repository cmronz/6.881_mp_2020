import numpy as np

def dist_between_points(a, b):
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
