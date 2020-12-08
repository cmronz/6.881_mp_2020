import uuid 

def ob_gen(obstacles):
	for o in obstacles:
		yield (uuid.uuid4(), o, o)