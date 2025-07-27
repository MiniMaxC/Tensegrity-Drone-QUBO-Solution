import numpy as np

# Import from the project modules
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from .quboconstruction import VariableMapper
except ImportError:
    from quboconstruction import VariableMapper
from Inputs import get_config_with_data

def decode_solution(solution_vector, mapper, params):
    """
    Decodes a binary solution vector from the QUBO solver into a structured, 
    human-readable format.

    Args:
        solution_vector (np.ndarray): The binary vector (0s and 1s) returned by the solver.
        mapper (VariableMapper): An initialized VariableMapper instance used for QUBO construction.
        params (dict): The configuration dictionary containing all problem parameters.

    Returns:
        dict: A dictionary containing the decoded path, selected regions, and configurations.
              Returns None if the input is invalid.
    """
    if not isinstance(solution_vector, np.ndarray) or solution_vector.ndim != 1:
        print("Error: solution_vector must be a 1D numpy array.")
        return None
    
    if solution_vector.size != mapper.N:
        print(f"Error: solution_vector size ({solution_vector.size}) does not match "
              f"mapper's expected size ({mapper.N}).")
        return None

    # Extract variable segments from the master solution vector
    h_vars = solution_vector[mapper.h_start_idx : mapper.c_start_idx]
    c_vars = solution_vector[mapper.c_start_idx : mapper.s_start_idx]
    s_vars = solution_vector[mapper.s_start_idx : mapper.z_start_idx]

    # Unpack parameters needed for decoding
    K, D, P = params['K'], params['D'], params['P']
    T, Q = params['T'], params['Q']
    S, x_shift = params['S'], params['x_shift']

    # Decode Position Variables (h)
    decoded_path = np.zeros((K, D))
    for k in range(1, K + 1):
        for d in range(1, D + 1):
            int_val = 0
            for p in range(P):
                # Get the flat index for this h-variable
                h_idx_flat = (k - 1) * D * P + (d - 1) * P + p
                binary_val = h_vars[h_idx_flat]
                int_val += (2**p) * binary_val
            
            # Apply reverse scaling and shifting
            decoded_path[k - 1, d - 1] = (int_val / S) - x_shift

    # Decode Region Selection Variables (c)
    selected_regions = []
    c_matrix = c_vars.reshape((K, T))  # Reshape to K steps x T regions
    for k in range(K):
        # Find selected region
        region_idx = np.argmax(c_matrix[k, :])
        selected_regions.append(region_idx + 1)  # Return 1-based index

    # Decode Configuration Selection Variables (s)
    selected_configs = []
    s_matrix = s_vars.reshape((K, Q))  # Reshape to K steps x Q configs
    for k in range(K):
        # Find selected configuration
        config_idx = np.argmax(s_matrix[k, :])
        selected_configs.append(config_idx + 1)  # Return 1-based index

    return {
        'path': decoded_path,
        'regions': selected_regions,
        'configs': selected_configs
    }

def main():
    """
    Example usage of the decode_solution function with mock data.
    This demonstrates how to use the decoder with sample solver output.
    """
    print("--- Testing Solution Decoder ---")
    
    # 1. Get problem configuration with real tensegrity data
    params = get_config_with_data()
    K, T, Q = params['K'], params['T'], params['Q']
    
    # 2. Initialize the VariableMapper to understand the solution vector structure
    mapper = VariableMapper(params)
    
    # 3. Generate a mock binary solution vector for demonstration
    # This simulates a realistic output from a QUBO solver
    print("\nGenerating a mock solution vector for a plausible scenario...")
    mock_solution = np.zeros(mapper.N, dtype=int)
    
    # Mock positions (h): choose some random binary values for demonstration
    num_h_vars = mapper.num_h
    mock_solution[mapper.h_start_idx : mapper.c_start_idx] = np.random.randint(0, 2, num_h_vars)
    
    # Mock region choices (c): for each step k, select one region t
    # Example path: Step 1 -> Region 1, Step 2 -> Region 2, etc.
    for k in range(1, min(K + 1, T + 1)):  # Ensure we don't exceed available regions
        c_idx = mapper.get_c_idx(t=k, k=k)
        mock_solution[c_idx] = 1

    # Mock config choices (s): for each step k, select one configuration q
    # Example configurations: Step 1 -> Config 2, Step 2 -> Config 1, etc.
    config_choices = [2, 1, 3]  # Example configuration sequence
    for k in range(1, K + 1):
        config_choice = config_choices[(k-1) % len(config_choices)]
        s_idx = mapper.get_s_idx(q=config_choice, k=k)
        mock_solution[s_idx] = 1
    
    # Note: Slack variables (z) are left as 0 for this example

    print(f"Mock solution vector of size {mapper.N} created.")

    # 4. Decode the mock solution
    decoded_results = decode_solution(mock_solution, mapper, params)
    
    # 5. Display the results in a readable format
    if decoded_results:
        print("\n--- Decoding Successful ---")
        print("Decoded Path Coordinates:")
        for k, pos in enumerate(decoded_results['path']):
            print(f"  Step {k+1}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
        print("\nSelected Safe Regions:")
        for k, region in enumerate(decoded_results['regions']):
            print(f"  Step {k+1}: Region {region}")
            
        print("\nSelected Drone Configurations:")
        for k, config in enumerate(decoded_results['configs']):
            print(f"  Step {k+1}: Configuration {config}")
        print("---------------------------\n")
    else:
        print("❌ Decoding failed - check solution vector format")

if __name__ == '__main__':
    # Run the decoder test: python decoder.py
    main()