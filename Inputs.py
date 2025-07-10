"""
Inputs for Compact QUBO Construction - Tensegrity Drone Path Planning

This file contains all the problem parameters needed to construct the QUBO matrix.
Modify these values to adjust the problem size and characteristics.
"""

import numpy as np

def generate_mock_data(params):
    """
    Generates plausible mock data based on the dimensions in params.
    This makes the script runnable without real experimental data.
    """
    print("Generating mock data...")
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
# Note: Even small numbers will result in a very large QUBO matrix.
# The number of slack variables (z) is the main driver of size.

config = {
    # Problem Dimensions
    'K': 2,       # Number of steps in the path
    'D': 3,       # Number of spatial dimensions
    'n': 4,       # Number of nodes in the drone's structure
    'T': 2,       # Number of safe convex regions
    'Q': 2,       # Number of available drone configurations
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
num_h = config['K'] * config['D'] * config['P']
num_c = config['T'] * config['K']
num_s = config['Q'] * config['K']
num_constraints = config['n'] * config['K'] * config['T'] * config['Q'] * config['U']
num_z = num_constraints * config['P_s']
total_vars = num_h + num_c + num_s + num_z

def get_config_with_data():
    """
    Returns the configuration dictionary with mock data generated.
    """
    return generate_mock_data(config.copy())

def print_problem_summary():
    """
    Prints a summary of the problem dimensions and variables.
    """
    print("="*40)
    print("Problem Inputs Initialized")
    print("="*40)
    print(f"Path Steps (K): {config['K']}, Dimensions (D): {config['D']}")
    print(f"Position Precision (P): {config['P']}, Slack Precision (Ps): {config['P_s']}")
    print(f"Regions (T): {config['T']}, Configurations (Q): {config['Q']}")
    print(f"Drone Nodes: {config['n']}")
    print("-"*40)
    print(f"Variable Counts:")
    print(f"  Position (h): {num_h}")
    print(f"  Region Select (c): {num_c}")
    print(f"  Config Select (s): {num_s}")
    print(f"  Slack (z): {num_z}")
    print(f"  ------------------")
    print(f"  Total Binary Variables: {total_vars}")
    print(f"  QUBO Matrix Size: {total_vars}x{total_vars}")
    print("="*40)

# Print summary when module is imported
if __name__ == "__main__":
    print_problem_summary()

