import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.colors as pc
from Inputs import get_config_with_data
from Solver.load_config_vis import ConfigurationLoader


class PathEnvironmentVisualizer:
    """
    Visualizes the tensegrity drone path planning environment including:
    - Safe zones (convex H-polytope regions)
    - Start and finish positions
    - Drone placed at the start position
    """
    
    def __init__(self):
        self.params = get_config_with_data()
        self.config_loader = ConfigurationLoader()
        self.config_data = self.config_loader.load_configurations()
        
        # Colors for different regions
        self.region_colors = ['lightblue', 'lightgreen', 'lightcoral', 'lightyellow', 'lightpink']
        self.region_names = ['Room1', 'Corridor', 'Room2']  # Based on Inputs.py structure
        
    def create_box_from_constraints(self, A, b):
        """
        Create a 3D box mesh from H-polytope constraints A*x <= b.
        Assumes the constraints define a rectangular box.
        """
        # Extract bounds from box constraints
        
        # Extract bounds from the constraint matrix
        D = A.shape[1]  # Number of dimensions
        bounds = np.zeros((D, 2))  # [min, max] for each dimension
        
        for i in range(D):
            # Find constraints that involve only dimension i
            pos_constraint = None
            neg_constraint = None
            
            for j in range(A.shape[0]):
                constraint = A[j, :]
                if np.sum(np.abs(constraint)) == 1:  # Only one non-zero element
                    if constraint[i] == 1:  # x_i <= b_j
                        bounds[i, 1] = min(bounds[i, 1], b[j]) if pos_constraint is not None else b[j]
                        pos_constraint = j
                    elif constraint[i] == -1:  # -x_i <= b_j => x_i >= -b_j
                        bounds[i, 0] = max(bounds[i, 0], -b[j]) if neg_constraint is not None else -b[j]
                        neg_constraint = j
        
        return bounds
    
    def draw_box(self, bounds, color='lightblue', opacity=0.3, name='Safe Zone'):
        """
        Draw a 3D box given bounds [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
        """
        x_min, x_max = bounds[0]
        y_min, y_max = bounds[1]
        z_min, z_max = bounds[2]
        
        # Box vertices
        vertices = np.array([
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_max, y_max, z_min],
            [x_min, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_max, y_max, z_max],
            [x_min, y_max, z_max]
        ])
        
        # Box faces (triangles)
        faces = [
            # Bottom face (z = z_min)
            [0, 1, 2], [0, 2, 3],
            # Top face (z = z_max)
            [4, 6, 5], [4, 7, 6],
            # Front face (y = y_min)
            [0, 4, 5], [0, 5, 1],
            # Back face (y = y_max)
            [2, 6, 7], [2, 7, 3],
            # Left face (x = x_min)
            [0, 3, 7], [0, 7, 4],
            # Right face (x = x_max)
            [1, 5, 6], [1, 6, 2]
        ]
        
        # Create mesh3d object
        return go.Mesh3d(
            x=vertices[:, 0],
            y=vertices[:, 1],
            z=vertices[:, 2],
            i=[f[0] for f in faces],
            j=[f[1] for f in faces],
            k=[f[2] for f in faces],
            color=color,
            opacity=opacity,
            name=name,
            showscale=False
        )
    
    def draw_sphere(self, center, radius, color, name):
        """
        Draw a sphere at given center with specified radius and color
        """
        u = np.linspace(0, 2*np.pi, 20)
        v = np.linspace(0, np.pi, 10)
        x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
        y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
        z = radius * np.outer(np.ones_like(u), np.cos(v)) + center[2]
        
        return go.Surface(
            x=x, y=y, z=z,
            showscale=False,
            opacity=0.8,
            surfacecolor=np.zeros_like(x),
            colorscale=[[0, color], [1, color]],
            name=name
        )
    
    def draw_drone_at_position(self, position, config_idx=0):
        """
        Draw the tensegrity drone at a specified position using a given configuration
        """
        robot = self.config_data['robots'][config_idx]
        # Get the raw positions and translate to the desired position
        drone_pos = self.config_data['raw_positions'][config_idx].copy()  # Nx3
        
        # Center the drone configuration around origin first
        drone_center = drone_pos.mean(axis=0)
        drone_pos_centered = drone_pos - drone_center
        
        # Scale down the drone to a reasonable size (assuming original is in meters)
        scale_factor = 0.5  # Make drone smaller for better visualization
        drone_pos_scaled = drone_pos_centered * scale_factor
        
        # Translate to the desired position
        drone_pos_final = drone_pos_scaled + position
        
        return self.create_drone_traces(robot, drone_pos_final.T)  # Transpose to 3xN
    
    def create_drone_traces(self, robot, r, 
                           node_radius=0.05,
                           cable_radius=0.005,
                           rod_radius=0.015,
                           node_color="red",
                           passive_node_color="orange",
                           cable_color="blue",
                           rod_color="darkblue"):
        """
        Create Plotly traces for drone visualization (adapted from vis_draw)
        """
        traces = []
        N = r.shape[1]
        
        # Add nodes as spheres
        for i in range(N):
            active = (i+1) in robot['active_nodes']
            color = node_color if active else passive_node_color
            
            node_trace = self.draw_sphere(r[:, i], node_radius, color, 
                                        f"{'Active' if active else 'Passive'} Node {i+1}")
            traces.append(node_trace)
        
        # Add cables and rods as cylinders
        C = robot['Cables']
        R = robot['Rods']
        
        # Cables
        for i in range(C.shape[0]):
            for j in range(i+1, C.shape[1]):
                if C[i,j] == 1:
                    cylinder_trace = self.create_cylinder(r[:,i], r[:,j], cable_radius, cable_color, f"Cable {i+1}-{j+1}")
                    traces.append(cylinder_trace)
        
        # Rods
        for i in range(R.shape[0]):
            for j in range(i+1, R.shape[1]):
                if R[i,j] == 1:
                    cylinder_trace = self.create_cylinder(r[:,i], r[:,j], rod_radius, rod_color, f"Rod {i+1}-{j+1}")
                    traces.append(cylinder_trace)
        
        return traces
    
    def create_cylinder(self, p1, p2, radius, color, name):
        """
        Create a cylinder between two points
        """
        v = p2 - p1
        L = np.linalg.norm(v)
        z = v / L
        
        # Create orthogonal vectors
        not_z = np.array([1,0,0]) if abs(z[0]) < 0.9 else np.array([0,1,0])
        x_dir = np.cross(not_z, z)
        x_dir /= np.linalg.norm(x_dir)
        y_dir = np.cross(z, x_dir)
        
        # Create cylinder
        theta = np.linspace(0, 2*np.pi, 12)
        circ = np.outer(np.cos(theta), x_dir) + np.outer(np.sin(theta), y_dir)
        
        X = np.zeros((len(theta), 2))
        Y = X.copy()
        Z = X.copy()
        
        for k, c in enumerate(circ):
            X[k,0], Y[k,0], Z[k,0] = p1 + radius * c
            X[k,1], Y[k,1], Z[k,1] = p1 + radius * c + v
        
        return go.Surface(
            x=X, y=Y, z=Z,
            showscale=False,
            opacity=0.8,
            surfacecolor=np.zeros_like(X),
            colorscale=[[0, color], [1, color]],
            name=name
        )
    
    def visualize_environment(self, show_drone=True, drone_config_idx=0):
        """
        Create the complete environment visualization
        """
        fig = go.Figure()
        
        # 1. Add safe zones
        print("Adding safe zones...")
        for i, (A, b) in enumerate(zip(self.params['A'], self.params['b'])):
            bounds = self.create_box_from_constraints(A, b)
            region_name = self.region_names[i] if i < len(self.region_names) else f"Region {i+1}"
            color = self.region_colors[i % len(self.region_colors)]
            
            box_trace = self.draw_box(bounds, color=color, opacity=0.2, name=region_name)
            fig.add_trace(box_trace)
            
            # Add region label at center
            center = bounds.mean(axis=1)
            fig.add_trace(go.Scatter3d(
                x=[center[0]], y=[center[1]], z=[center[2]],
                mode='text',
                text=[region_name],
                textfont=dict(size=14, color='black'),
                showlegend=False,
                name=f"{region_name} Label"
            ))
        
        # 2. Add start position
        print("Adding start position...")
        start_pos = self.params['x_start']
        start_sphere = self.draw_sphere(start_pos, 0.1, 'green', 'Start Position')
        fig.add_trace(start_sphere)
        
        # Add start label
        fig.add_trace(go.Scatter3d(
            x=[start_pos[0]], y=[start_pos[1]], z=[start_pos[2] + 0.2],
            mode='text',
            text=['START'],
            textfont=dict(size=16, color='green'),
            showlegend=False,
            name='Start Label'
        ))
        
        # 3. Add goal position
        print("Adding goal position...")
        goal_pos = self.params['x_goal']
        goal_sphere = self.draw_sphere(goal_pos, 0.1, 'red', 'Goal Position')
        fig.add_trace(goal_sphere)
        
        # Add goal label
        fig.add_trace(go.Scatter3d(
            x=[goal_pos[0]], y=[goal_pos[1]], z=[goal_pos[2] + 0.2],
            mode='text',
            text=['GOAL'],
            textfont=dict(size=16, color='red'),
            showlegend=False,
            name='Goal Label'
        ))
        
        # 4. Add drone at start position
        if show_drone:
            print(f"Adding drone (configuration {drone_config_idx + 1}) at start position...")
            drone_traces = self.draw_drone_at_position(start_pos, drone_config_idx)
            for trace in drone_traces:
                fig.add_trace(trace)
        
        # Configure layout
        fig.update_layout(
            title={
                'text': 'Tensegrity Drone Path Planning Environment',
                'x': 0.5,
                'font': {'size': 18}
            },
            scene=dict(
                xaxis_title='X (m)',
                yaxis_title='Y (m)', 
                zaxis_title='Z (m)',
                aspectmode='data',
                camera=dict(
                    eye=dict(x=1.5, y=1.5, z=1.5)
                )
            ),
            width=1200,
            height=800,
            showlegend=True
        )
        
        return fig
    
    def show_environment_summary(self):
        """
        Print a summary of the environment setup
        """
        print("=" * 60)
        print("TENSEGRITY DRONE ENVIRONMENT SUMMARY")
        print("=" * 60)
        
        print(f"🚁 Drone Configurations Available: {self.params['Q']}")
        print(f"📍 Start Position: [{self.params['x_start'][0]:.2f}, {self.params['x_start'][1]:.2f}, {self.params['x_start'][2]:.2f}] m")
        print(f"🎯 Goal Position:  [{self.params['x_goal'][0]:.2f}, {self.params['x_goal'][1]:.2f}, {self.params['x_goal'][2]:.2f}] m")
        
        print(f"\n🏠 Safe Zones ({len(self.params['A'])}):")
        for i, (A, b) in enumerate(zip(self.params['A'], self.params['b'])):
            bounds = self.create_box_from_constraints(A, b)
            region_name = self.region_names[i] if i < len(self.region_names) else f"Region {i+1}"
            volume = np.prod(bounds[:, 1] - bounds[:, 0])
            center = bounds.mean(axis=1)
            
            print(f"  {region_name}:")
            print(f"    Center: [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}] m")
            print(f"    Size: [{bounds[0,1]-bounds[0,0]:.2f} × {bounds[1,1]-bounds[1,0]:.2f} × {bounds[2,1]-bounds[2,0]:.2f}] m")
            print(f"    Volume: {volume:.1f} m³")
        
        print("\n" + "=" * 60)


def main():
    """
    Main function to create and display the environment visualization
    """
    print("🚁 Creating Tensegrity Drone Environment Visualization...")
    
    # Create visualizer
    visualizer = PathEnvironmentVisualizer()
    
    # Show environment summary
    visualizer.show_environment_summary()
    
    # Create and show visualization
    print("\n📊 Generating 3D visualization...")
    fig = visualizer.visualize_environment(show_drone=True, drone_config_idx=0)
    
    print("✅ Visualization ready! Opening in browser...")
    fig.show()
    
    print("\n💡 Visualization includes:")
    print("  - Safe zones (translucent colored boxes)")
    print("  - Start position (green sphere)")
    print("  - Goal position (red sphere)")
    print("  - Tensegrity drone at start position")
    
    return visualizer, fig


if __name__ == "__main__":
    main() 