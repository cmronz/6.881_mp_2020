# 
# from rtree import index 
# import numpy as np
import uuid 

# # O = np.array(
# #     [(2, 2, 2, 4, 4, 4), (2, 2, 6, 4, 4, 8), (2, 6, 2, 4, 8, 4), (6, 6, 2, 8, 8, 4),
# #      (6, 2, 2, 8, 4, 4), (6, 2, 6, 8, 4, 8), (2, 6, 6, 4, 8, 8), (6, 6, 6, 8, 8, 8)])

# O = np.array([(2.75, 2.75, 2.75, 7.75, 7.75, 7.75)])

def ob_gen(obstacles):
	for o in obstacles:
		yield (uuid.uuid4(), o, o)


# p = index.Property()
# p.dimension = 3
# obs = index.Index(ob_gen(O), interleaved=True, properties=p)

# point = (0, 0, 0)

# print(obs.count(point))
# print(obs.count((5,5,5)))