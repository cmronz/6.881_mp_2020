import numpy as np

from shapely.geometry import Point

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