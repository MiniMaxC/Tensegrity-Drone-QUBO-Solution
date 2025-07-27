"""
Inputs for Compact QUBO Construction - Tensegrity Drone Path Planning

This file contains all the problem parameters needed to construct the QUBO matrix.
Modify these values to adjust the problem size and characteristics.
"""

import numpy as np
from Solver.config_loader import load_real_configurations

def load_real_data(params):
    """
    Loads real configuration data from the .mat file and generates
    appropriate supporting data for the optimization problem.
    """
    print("Loading real configuration data...")
    K, D, n, T, Q, U = params['K'], params['D'], params['n'], params['T'], params['Q'], params['U']

    # Load real configuration data from .mat file
    try:
        config_data = load_real_configurations()
        
        # Extract the real configuration vectors and node extraction matrices
        params['r_conf'] = config_data['r_conf']
        params['P_node'] = config_data['P_node']
        
        # Verify dimensions match
        actual_configs = config_data['num_configs']
        actual_nodes = config_data['num_nodes']
        actual_dim = config_data['spatial_dim']
        
        if actual_configs != Q:
            print(f"⚠️  Warning: Expected {Q} configurations, found {actual_configs}. Updating Q parameter.")
            params['Q'] = actual_configs
        
        if actual_nodes != n:
            print(f"⚠️  Warning: Expected {n} nodes, found {actual_nodes}. Updating n parameter.")
            params['n'] = actual_nodes
            
        if actual_dim != D:
            print(f"⚠️  Warning: Expected {D} dimensions, found {actual_dim}. Updating D parameter.")
            params['D'] = actual_dim
        
        print(f"✅ Loaded {len(params['r_conf'])} real configurations with {actual_nodes} nodes each")
        
    except Exception as e:
        print(f"❌ Error loading real configuration data: {e}")
        print("Falling back to mock data generation...")
        return generate_mock_data_fallback(params)

    # Generate reasonable start and goal positions based on actual configuration bounds
    all_positions = np.concatenate(params['r_conf'])
    pos_min, pos_max = all_positions.min(), all_positions.max()
    pos_range = pos_max - pos_min
    pos_center = (pos_min + pos_max) / 2
    
    # Generate safe convex regions: 2 rooms connected by a corridor
    params['A'] = []
    params['b'] = []
    
    # Calculate spatial layout parameters for drone-scale regions
    pos_center = (pos_min + pos_max) / 2
    
    # Make rooms large enough for the drone (52cm span) with safety margins
    # Target: rooms ≥ 30m³, corridor long enough to connect them
    room_side = 3.2      # 3.2m × 3.2m × 3.2m = 32.8m³ > 30m³
    corridor_width = 2.0  # 2m wide corridor (4x drone span)
    corridor_length = 8.0 # 8m long corridor to connect rooms
    separation = 5.6      # 5.6m separation = corridor_length/2 + room_side/2 (ensures connection)
    
    # Define the 3 regions: Room1 - Corridor - Room2
    regions = [
        {
            'name': 'Room1',
            'center': pos_center + np.array([-separation, 0, 0]),
            'size': np.array([room_side, room_side, room_side])
        },
        {
            'name': 'Corridor', 
            'center': pos_center,
            'size': np.array([corridor_length, corridor_width, room_side])
        },
        {
            'name': 'Room2',
            'center': pos_center + np.array([separation, 0, 0]), 
            'size': np.array([room_side, room_side, room_side])
        }
    ]
    
    print("Creating H-Polytope safe regions:")
    
    for t, region in enumerate(regions):
        center = region['center']
        box_size = region['size']
        
        # Box constraints: -I*x <= -(center - size/2) and I*x <= center + size/2
        # This creates: center - size/2 <= x <= center + size/2
        A_t = np.vstack([-np.eye(params['D']), np.eye(params['D'])])  # (2*D, D) matrix
        b_t = np.concatenate([-(center - box_size/2), center + box_size/2])  # (2*D,) vector
        
        # Adjust constraints if U != 2*D
        if U > 2 * params['D']:
            # Pad with additional constraints (keep the region valid)
            extra_constraints = U - 2 * params['D']
            A_extra = np.random.randn(extra_constraints, params['D']) * 0.01  # Very small random constraints
            b_extra = np.random.rand(extra_constraints) * pos_range * 0.1
            A_t = np.vstack([A_t, A_extra])
            b_t = np.concatenate([b_t, b_extra])
        elif U < 2 * params['D']:
            # Truncate to U constraints
            A_t = A_t[:U, :]
            b_t = b_t[:U]
        
        params['A'].append(A_t)
        params['b'].append(b_t)
        
        volume = np.prod(box_size)
        print(f"  ✅ {region['name']}: center={center.round(3)}, size={box_size.round(3)}, volume={volume:.1f}m³")
    
    # Place start and goal positions within safe regions
    # Start in Room1, Goal in Room2 with some offset from center for realism
    room1_center = regions[0]['center']  # Room1
    room2_center = regions[2]['center']  # Room2
    room_size = regions[0]['size']       # Both rooms same size
    
    # Add small random offsets within the rooms (±25% of room size from center)
    start_offset = (np.random.rand(params['D']) - 0.5) * 0.5 * room_size  # ±25% offset
    goal_offset = (np.random.rand(params['D']) - 0.5) * 0.5 * room_size   # ±25% offset
    
    params['x_start'] = room1_center + start_offset
    params['x_goal'] = room2_center + goal_offset
    
    print(f"📍 Start/Goal Positions:")
    print(f"  ✅ Start (Room1): [{params['x_start'][0]:.2f}, {params['x_start'][1]:.2f}, {params['x_start'][2]:.2f}]m")
    print(f"  ✅ Goal (Room2):  [{params['x_goal'][0]:.2f}, {params['x_goal'][1]:.2f}, {params['x_goal'][2]:.2f}]m")
    
    # Verify positions are within safe regions
    def check_position_in_region(pos, A, b, region_name):
        """Check if position satisfies A*x <= b constraints"""
        constraints_satisfied = np.all(A @ pos <= b + 1e-6)  # Small tolerance for numerical precision
        return constraints_satisfied
    
    start_safe = check_position_in_region(params['x_start'], params['A'][0], params['b'][0], 'Room1')
    goal_safe = check_position_in_region(params['x_goal'], params['A'][2], params['b'][2], 'Room2')
    
    print(f"  ✅ Start in Room1: {'✅ YES' if start_safe else '❌ NO'}")
    print(f"  ✅ Goal in Room2:  {'✅ YES' if goal_safe else '❌ NO'}")
    
    return params

def generate_mock_data_fallback(params):
    """
    Fallback function that generates mock data if real data loading fails.
    """
    print("Generating fallback mock data...")
    K, D, n, T, Q, U = params['K'], params['D'], params['n'], params['T'], params['Q'], params['U']

    params['x_start'] = np.random.rand(D)
    params['x_goal'] = np.random.rand(D) + 5
    
    # A_t will be a list of T matrices, each of size U x D
    params['A'] = [np.random.randn(U, D) for _ in range(T)]
    
    # b_t will be a list of T vectors, each of size U
    params['b'] = [np.random.rand(U) * 10 for _ in range(T)]

    # r_conf will be a list of Q vectors, each representing a drone shape
    # A shape is n nodes * D dimensions = 3n vector
    params['r_conf'] = [np.random.rand(n * D) * 0.5 for _ in range(Q)]
    
    # P_node are indexing matrices to get node i from r_conf vector
    # P_i is a 3 x 3n matrix for D=3
    params['P_node'] = []
    for i in range(n):
        Pi = np.zeros((D, n * D))
        Pi[:, i*D:(i+1)*D] = np.eye(D)
        params['P_node'].append(Pi)
    
    return params

# ==============================================================================
# Configuration Dictionary
# ==============================================================================
# Define all problem parameters here.
# Note: These dimensions are based on the real tensegrity drone data.
# The .mat file contains 8 configurations with 12 nodes each.

config = {
    # Problem Dimensions (updated to match real data)
    'K': 3,       # Number of steps in the path (reduced for computational efficiency)
    'D': 3,       # Number of spatial dimensions (3D coordinates)
    'n': 12,      # Number of nodes in the drone's structure (from .mat file)
    'T': 3,       # Number of safe convex regions (2 rooms + 1 corridor)
    'Q': 8,       # Number of available drone configurations (from .mat file)
    'U': 6,       # Number of constraints per convex region (e.g., a box)

    # Discretization Parameters
    'P': 3,       # Bits of precision for position variables
    'P_s': 2,     # Bits of precision for slack variables
    'S': 100.0,   # Scaling factor for position
    'x_shift': 5.0, # Position offset

    # Penalty Weights
    'w': 1.0,
    'lambda_1': 10.0,
    'lambda_2': 10.0,
    'lambda_3': 10.0,
    'M': 100.0      # Big-M constant
}

# ==============================================================================
# Computed Parameters (Do Not Modify)
# ==============================================================================
# Note: These calculations show the problem size with real data
num_h = config['K'] * config['D'] * config['P']          # Path variables
num_c = config['T'] * config['K']                        # Region selection variables  
num_s = config['Q'] * config['K']                        # Configuration selection variables
num_constraints = config['n'] * config['K'] * config['T'] * config['Q'] * config['U']  # Total constraint count
num_z = num_constraints * config['P_s']                  # Slack variables
total_vars = num_h + num_c + num_s + num_z              # Total binary variables in QUBO

def get_config_with_data():
    """
    Returns the configuration dictionary with real configuration data loaded.
    """
    return load_real_data(config.copy())

def print_problem_summary():
    """
    Prints a summary of the problem dimensions and variables.
    """
    print("="*50)
    print("TENSEGRITY DRONE OPTIMIZATION - PROBLEM SUMMARY")
    print("="*50)
    print("📁 Data Source: Real configurations from .mat file")
    print(f"🚁 Drone Structure: {config['n']} nodes in {config['D']}D space")
    print(f"🔧 Available Configs: {config['Q']} different tensegrity shapes")
    print(f"🛣️  Path Planning: {config['K']} steps through {config['T']} safe regions (Room1→Corridor→Room2)")
    print(f"📍 Start/Goal: Random positions within Room1 and Room2 respectively")
    print()
    print("📊 Optimization Parameters:")
    print(f"  - Position Precision (P): {config['P']} bits")
    print(f"  - Slack Precision (P_s): {config['P_s']} bits") 
    print(f"  - Constraints per region (U): {config['U']}")
    print()
    print("🔢 QUBO Matrix Dimensions:")
    print(f"  - Position variables (h): {num_h:,}")
    print(f"  - Region selection (c): {num_c:,}")
    print(f"  - Configuration selection (s): {num_s:,}")
    print(f"  - Slack variables (z): {num_z:,}")
    print(f"  {'─' * 35}")
    print(f"  - Total Binary Variables: {total_vars:,}")
    print(f"  - QUBO Matrix Size: {total_vars:,} × {total_vars:,}")
    print(f"  - Matrix Elements: {total_vars**2:,}")
    print()
    print("⚠️  Note: Large matrix sizes may require significant memory")
    print("="*50)

# Print summary when module is imported
if __name__ == "__main__":
    print_problem_summary()

