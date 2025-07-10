import numpy as np
from qdeepsdk import QDeepHybridSolver
import requests
import os

# Initialize the solver
solver = QDeepHybridSolver()

# Set the authentication token
solver.token = "your-auth-token"

# Configure parameters (if different from defaults)
solver.m_budget = 50000     # Measurement budget
solver.num_reads = 10000    # Number of reads

# Load QUBO matrix from text file (look in parent directory)
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
matrix_file = os.path.join(parent_dir, "Q_matrix.txt")
try:
    Q = np.loadtxt(matrix_file)
    print(f"QUBO matrix loaded from {matrix_file}")
    print(f"Matrix shape: {Q.shape}")
except FileNotFoundError:
    print(f"Error: Matrix file '{matrix_file}' not found.")
    print("Please generate a QUBO matrix first using the QUBO builder.")
    exit(1)
except Exception as e:
    print(f"Error loading matrix: {e}")
    exit(1)

# Solve the QUBO problem
try:
    response = solver.solve(Q)
    results = response['QdeepHybridSolver']
    print("Hybrid Solver Results:", results)
    
    # Write results to file (save in parent directory)
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output_file = os.path.join(parent_dir, "Solved_Problem")
    with open(output_file, 'w') as f:
        f.write("QUBO Problem Solution Results\n")
        f.write("=" * 40 + "\n\n")
        f.write(f"Matrix file used: {matrix_file}\n")
        f.write(f"Matrix shape: {Q.shape}\n")
        f.write(f"Measurement budget: {solver.m_budget}\n")
        f.write(f"Number of reads: {solver.num_reads}\n\n")
        f.write("Solver Results:\n")
        f.write(str(results))
    
    print(f"Results saved to {output_file}")
    
except ValueError as e:
    print(f"Error: {e}")
except requests.RequestException as e:
    print(f"API Error: {e}")