from scipy import interpolate
import numpy as np
import warnings

class RRTPathProcessor:
    @classmethod
    def convert_to_joint_space(cls, path_3d, append_non_optimal=False):
        path_q = []
        q_prev = None
        warnings.filterwarnings('ignore')
        for pos in path_3d:
            ik_solver = IKSolver()
            q, optimal = ik_solver.solve(RigidTransform(p=np.array(pos), R=R_WG), q_guess=q_prev)
            if (not optimal and append_non_optimal) or optimal:
                q_prev = q
                path_q.append(q)
        return path_q
    
    @classmethod
    def interpolate_path(cls, path, num_points=200):
        Q = np.array(path)
        # _, idx = np.unique(Q, return_index=True)
        # Q_i = Q[np.sort(idx)]
        # # Q = np.unique(, axis=0)
        tck, u = interpolate.splprep(Q.T, s=2)
        u_fine = np.linspace(0, 1, num_points)
        O = np.array(interpolate.splev(u_fine, tck)).T
        return O