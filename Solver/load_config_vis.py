import os
import numpy as np
import scipy.io
import plotly.graph_objects as go


def unwrap(matobj):
    """
    Recursively unwrap MATLAB arrays of size 1 to native Python types or arrays.
    """
    if isinstance(matobj, np.ndarray) and matobj.size == 1:
        return unwrap(matobj.flat[0])
    return matobj


class ConfigurationLoader:
    """
    Loads and processes tensegrity drone configuration data from .mat files.
    """
    def __init__(self, mat_file_path=None):
        if mat_file_path is None:
            # Default to the .mat file in the same directory as this script
            mat_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Sixbar_Drone_12vertices_8configs_NEW_V4.mat')
        self.mat_file_path = mat_file_path
        self.configs_data = None
        self.robots = []
        self.num_configs = 0
        self.num_nodes = 0
        self.spatial_dim = 3  # 3D coordinates

    def load_configurations(self):
        if not os.path.exists(self.mat_file_path):
            raise FileNotFoundError(f"Configuration file not found: {self.mat_file_path}")

        data = scipy.io.loadmat(self.mat_file_path)
        if 'configs' not in data:
            raise ValueError("No 'configs' field found in .mat file")

        raw = unwrap(data['configs'])
        # Determine count
        if raw.ndim == 2:
            self.num_configs = raw.shape[1]
        else:
            self.num_configs = 1
            raw = raw.reshape((1,))

        self.configs_data = []
        self.robots = []

        for idx in range(self.num_configs):
            cfg = raw[0, idx] if self.num_configs > 1 else raw[0]
            # Extract positions and connectivity
            nodes_pos = unwrap(cfg['nodes_position'])  # 3xN
            C = unwrap(cfg['Cables'])                  # NxN
            R = unwrap(cfg['Rods'])                    # NxN
            active = unwrap(cfg['active_nodes']).flatten().tolist()

            if idx == 0:
                self.num_nodes = nodes_pos.shape[1]

            # store raw positions and robot dict
            self.configs_data.append(nodes_pos.T)  # Nx3
            self.robots.append({
                'active_nodes': active,
                'Cables': C,
                'Rods': R
            })

        return self._process_for_qubo()

    def _process_for_qubo(self):
        """
        Process configs into QUBO-ready structures, preserving raw positions & robot info.
        """
        # flatten configuration vectors
        r_conf = [pos.flatten() for pos in self.configs_data]
        # build extraction matrices
        P_node = []
        for ni in range(self.num_nodes):
            P = np.zeros((self.spatial_dim, self.num_nodes * self.spatial_dim))
            idx = ni * self.spatial_dim
            P[:, idx:idx + self.spatial_dim] = np.eye(self.spatial_dim)
            P_node.append(P)

        return {
            'r_conf': r_conf,
            'P_node': P_node,
            'num_configs': self.num_configs,
            'num_nodes': self.num_nodes,
            'spatial_dim': self.spatial_dim,
            'raw_positions': self.configs_data,  # list of Nx3
            'robots': self.robots               # list of robot dicts
        }

    def get_configuration_summary(self):
        if not self.configs_data:
            raise ValueError("No configurations loaded yet.")

        all_pos = np.vstack(self.configs_data)
        summary = {
            'num_configs': self.num_configs,
            'num_nodes': self.num_nodes,
            'spatial_dim': self.spatial_dim,
            'range': {
                'x': [float(all_pos[:,0].min()), float(all_pos[:,0].max())],
                'y': [float(all_pos[:,1].min()), float(all_pos[:,1].max())],
                'z': [float(all_pos[:,2].min()), float(all_pos[:,2].max())]
            },
            'mean': all_pos.mean(axis=0).tolist(),
            'std': all_pos.std(axis=0).tolist(),
        }
        return summary


def vis_draw(robot, r,
             node_radius=0.014,
             cable_radius=0.001,
             rod_radius=0.004,
             node_color="magenta",
             passive_node_color="yellow",
             cable_color="green",
             rod_color="blue",
             opacity=1.0):
    """
    Inline 3D visualization of a tensegrity config in Plotly.

    robot: dict with 'active_nodes', 'Cables', 'Rods'
    r: 3xN positions
    """
    fig = go.Figure()
    N = r.shape[1]

    # spheres for nodes
    for i in range(N):
        active = (i+1) in robot['active_nodes']
        color = node_color if active else passive_node_color
        u = np.linspace(0, 2*np.pi, 24)
        v = np.linspace(0, np.pi, 12)
        x = node_radius * np.outer(np.cos(u), np.sin(v)) + r[0,i]
        y = node_radius * np.outer(np.sin(u), np.sin(v)) + r[1,i]
        z = node_radius * np.outer(np.ones_like(u), np.cos(v)) + r[2,i]
        fig.add_trace(go.Surface(x=x, y=y, z=z, showscale=False,
                                 opacity=opacity,
                                 surfacecolor=np.zeros_like(x),
                                 colorscale=[[0, color],[1,color]]))

    # cylinder helper
    def draw_cyl(p1, p2, radius, color):
        v = p2 - p1; L = np.linalg.norm(v)
        z = v/L
        not_z = np.array([1,0,0]) if abs(z[0])<0.9 else np.array([0,1,0])
        x_dir = np.cross(not_z, z); x_dir /= np.linalg.norm(x_dir)
        y_dir = np.cross(z, x_dir)
        theta = np.linspace(0,2*np.pi,24)
        circ = np.outer(np.cos(theta), x_dir)+np.outer(np.sin(theta), y_dir)
        X = np.zeros((len(theta),2)); Y = X.copy(); Z = X.copy()
        for k, c in enumerate(circ):
            X[k,0],Y[k,0],Z[k,0] = p1 + radius*c
            X[k,1],Y[k,1],Z[k,1] = p1 + radius*c + v
        fig.add_trace(go.Surface(x=X,y=Y,z=Z,showscale=False,
                                 opacity=opacity,
                                 surfacecolor=np.zeros_like(X),
                                 colorscale=[[0,color],[1,color]]))

    # draw cables and rods
    C = robot['Cables']; R = robot['Rods']
    for i in range(C.shape[0]):
        for j in range(i+1, C.shape[1]):
            if C[i,j] == 1:
                draw_cyl(r[:,i], r[:,j], cable_radius, cable_color)
    for i in range(R.shape[0]):
        for j in range(i+1, R.shape[1]):
            if R[i,j] == 1:
                draw_cyl(r[:,i], r[:,j], rod_radius, rod_color)

    fig.update_layout(scene=dict(aspectmode='data'))
    fig.show()


if __name__ == "__main__":
    loader = ConfigurationLoader()
    data = loader.load_configurations()
    print("Summary:", loader.get_configuration_summary())

    # visualize each configuration with connectivity
    for idx, (robot, pos) in enumerate(zip(data['robots'], data['raw_positions']), start=1):
        print(f"Displaying config {idx}")
        vis_draw(robot, pos.T)
