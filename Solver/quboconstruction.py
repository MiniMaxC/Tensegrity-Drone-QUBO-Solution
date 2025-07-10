import numpy as np
import itertools
import sys
import os

# Add parent directory to path to import Inputs
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Inputs import get_config_with_data

class VariableMapper:
    """
    Maps multi-indexed variables (h, c, s, z) to a single flat index.
    This is crucial for building the N x N QUBO matrix.
    The order of variables in the flat vector 'v' will be: H, C, S, Z.
    """
    def __init__(self, params):
        self.p = params
        # Unpack dimensions for easier access
        self.K, self.D, self.P = self.p['K'], self.p['D'], self.p['P']
        self.T, self.Q = self.p['T'], self.p['Q']
        self.n, self.U, self.P_s = self.p['n'], self.p['U'], self.p['P_s']

        # Calculate the number of variables of each type
        self.num_h = self.K * self.D * self.P
        self.num_c = self.T * self.K
        self.num_s = self.Q * self.K
        self.num_z = self.n * self.K * self.T * self.Q * self.U * self.P_s

        # Calculate the total number of variables
        self.N = self.num_h + self.num_c + self.num_s + self.num_z

        # Calculate the starting index (offset) for each variable type
        self.h_start_idx = 0
        self.c_start_idx = self.num_h
        self.s_start_idx = self.c_start_idx + self.num_c
        self.z_start_idx = self.s_start_idx + self.num_s

        print("--- Variable Mapping Initialized ---")
        print(f"Total number of variables (N): {self.N}")
        print(f"  - Position vars (h): {self.num_h}")
        print(f"  - Region choice vars (c): {self.num_c}")
        print(f"  - Config choice vars (s): {self.num_s}")
        print(f"  - Slack vars (z): {self.num_z}")
        print("------------------------------------")


    def get_h_idx(self, k, d, p):
        # k: step (1 to K), d: dimension (1 to D), p: bit (0 to P-1)
        # Note: We convert 1-based k and d to 0-based indices
        return self.h_start_idx + (k - 1) * self.D * self.P + (d - 1) * self.P + p

    def get_c_idx(self, t, k):
        # t: region (1 to T), k: step (1 to K)
        return self.c_start_idx + (k - 1) * self.T + (t - 1)

    def get_s_idx(self, q, k):
        # q: configuration (1 to Q), k: step (1 to K)
        return self.s_start_idx + (k - 1) * self.Q + (q - 1)

    def get_z_idx(self, i, k, t, q, u, m):
        # i: node (1 to n), k: step (1 to K), t: region (1 to T), q: config (1 to Q)
        # u: constraint row (1 to U), m: bit (0 to Ps-1)
        offset = (i - 1) * (self.K * self.T * self.Q * self.U * self.P_s) + \
                 (k - 1) * (self.T * self.Q * self.U * self.P_s) + \
                 (t - 1) * (self.Q * self.U * self.P_s) + \
                 (q - 1) * (self.U * self.P_s) + \
                 (u - 1) * (self.P_s) + \
                 m
        return self.z_start_idx + offset

def add_to_q(Q, idx1, idx2, value):
    """
    Adds a value to the QUBO matrix Q, ensuring it remains upper-triangular.
    - For linear terms (idx1 == idx2), it adds to the diagonal.
    - For quadratic terms (idx1 != idx2), it adds to the off-diagonal
      at Q[min(idx1, idx2), max(idx1, idx2)].
    """
    if idx1 == idx2:
        Q[idx1, idx1] += value
    else:
        # Enforce upper-triangular form
        i, j = min(idx1, idx2), max(idx1, idx2)
        Q[i, j] += value

def populate_path_objective(Q, params, mapper):
    """
    Populates Q with coefficients from the binarised path objective.
    (Eqs. 14, 15, 16 from the document)
    """
    print("Populating Path Objective...")
    # Unpack parameters
    K, D, P = params['K'], params['D'], params['P']
    S, x_shift = params['S'], params['x_shift']
    x_start, x_goal = params['x_start'], params['x_goal']
    w = params['w']

    # Term 1: Start point term ||x_1 - x_start||^2 (Eq. 14)
    # This expands to x_1,d^2 - 2*x_start,d*x_1,d + const
    for d in range(1, D + 1):
        x_start_d = x_start[d - 1]
        # x_1,d = (1/S * sum(2^p * h_1,d,p)) - x_shift
        # Expand the square of this expression
        for p in range(P):
            for p_prime in range(P):
                h_idx1 = mapper.get_h_idx(1, d, p)
                h_idx2 = mapper.get_h_idx(1, d, p_prime)
                # From ( (1/S)*sum(...) - x_shift )^2
                # This gives (1/S^2)*2^(p+p')*h*h'
                add_to_q(Q, h_idx1, h_idx2, (1/S**2) * (2**(p + p_prime)))

        for p in range(P):
            h_idx = mapper.get_h_idx(1, d, p)
            # Linear terms from expansion:
            # from x_1,d^2: we get (1/S)^2 * (2^p)^2 * h^2 -> becomes linear, but handled above by h*h
            # from -2*x_shift*(1/S)*sum(...): we get -2*x_shift*(1/S)*2^p * h
            # from -2*x_start,d*x_1,d: we get -2*x_start,d*(1/S)*2^p * h
            term1 = -2 * x_shift * (1 / S) * (2**p)
            term2 = -2 * x_start_d * (1 / S) * (2**p)
            add_to_q(Q, h_idx, h_idx, term1 + term2)


    # Term 2: Goal point term ||x_K - x_goal||^2 (Eq. 15)
    # Same structure as the start point term
    for d in range(1, D + 1):
        x_goal_d = x_goal[d - 1]
        for p in range(P):
            for p_prime in range(P):
                h_idx1 = mapper.get_h_idx(K, d, p)
                h_idx2 = mapper.get_h_idx(K, d, p_prime)
                add_to_q(Q, h_idx1, h_idx2, (1 / S**2) * (2**(p + p_prime)))
        for p in range(P):
            h_idx = mapper.get_h_idx(K, d, p)
            term1 = -2 * x_shift * (1 / S) * (2**p)
            term2 = -2 * x_goal_d * (1 / S) * (2**p)
            add_to_q(Q, h_idx, h_idx, term1 + term2)


    # Term 3: Path smoothness term w * ||x_k+1 - x_k||^2 (Eq. 16)
    # x_k+1,d - x_k,d = (1/S) * (sum(h_k+1) - sum(h_k))
    for k in range(1, K):
        for d in range(1, D + 1):
            # Expand ( (sum_p h_k+1) - (sum_m h_k) )^2
            # (sum_p h_k+1)^2 + (sum_m h_k)^2 - 2*(sum_p h_k+1)*(sum_m h_k)
            for p in range(P):
                for p_prime in range(P):
                    # (sum_p h_k+1)^2 term
                    h1_idx = mapper.get_h_idx(k + 1, d, p)
                    h2_idx = mapper.get_h_idx(k + 1, d, p_prime)
                    add_to_q(Q, h1_idx, h2_idx, (w / S**2) * (2**(p + p_prime)))

                    # (sum_m h_k)^2 term
                    h1_idx = mapper.get_h_idx(k, d, p)
                    h2_idx = mapper.get_h_idx(k, d, p_prime)
                    add_to_q(Q, h1_idx, h2_idx, (w / S**2) * (2**(p + p_prime)))
            
            # Cross term -2 * (sum_p h_k+1) * (sum_m h_k)
            for p in range(P):
                for m in range(P):
                    h_kplus1_idx = mapper.get_h_idx(k + 1, d, p)
                    h_k_idx = mapper.get_h_idx(k, d, m)
                    add_to_q(Q, h_kplus1_idx, h_k_idx, -2 * (w / S**2) * (2**(p + m)))


def populate_inequality_penalty(Q, params, mapper):
    """
    Populates Q with coefficients from the P1 inequality penalty.
    (Eqs. 17-19 and their expansions on pages 6-7)
    """
    print("Populating Inequality Penalty (P1)...")
    # Unpack parameters
    n, K, T, Q_conf, U, P = params['n'], params['K'], params['T'], params['Q'], params['U'], params['P']
    Ps = params['P_s']
    D = params['D']
    M, S, x_shift = params['M'], params['S'], params['x_shift']
    lambda1 = params['lambda_1']
    
    A, b, r_conf, P_node = params['A'], params['b'], params['r_conf'], params['P_node']

    # Loop over every single scalar constraint that makes up P1
    # i:node, k:step, t:region, q:config, u:constraint_row
    for i, k, t, q, u in itertools.product(range(1, n + 1), range(1, K + 1), range(1, T + 1), range(1, Q_conf + 1), range(1, U + 1)):
        # For each constraint, we are expanding (L_i,k,t,q,u)^2
        # where L = H + M*c + M*s + Z + G (simplified notation from doc)

        # 1. Calculate the constant G for this constraint (Eq. 19)
        # G = (A_t * P_i * r^q)_u - (b_t)_u - 2M - sum_d(A_t,u,d * x_shift)
        At = A[t - 1]
        bt = b[t - 1]
        rq = r_conf[q - 1]
        Pi = P_node[i - 1]
        
        A_t_u = At[u - 1, :]
        
        term1 = (A_t_u @ Pi @ rq)
        term2 = -bt[u - 1]
        term3 = -2 * M
        term4 = -np.sum(A_t_u * x_shift)
        G = term1 + term2 + term3 + term4
        
        # 2. Get indices for c, s variables for this constraint
        c_idx = mapper.get_c_idx(t, k)
        s_idx = mapper.get_s_idx(q, k)
        
        # 3. Collect lists of indices for h and z variables for this constraint
        h_indices = [mapper.get_h_idx(k, d, p) for d in range(1, D + 1) for p in range(P)]
        z_indices = [mapper.get_z_idx(i, k, t, q, u, m) for m in range(Ps)]
        
        # 4. Define coefficients for H and Z parts of L (Eq. 19)
        # H = sum_{d,p} (A_t,u,d / S) * 2^p * h_k,d,p
        # Z = sum_{m} 2^m * z_i,k,t,q,u,m
        h_coeffs = [(A_t_u[d-1] / S) * (2**p) for d in range(1, D+1) for p in range(P)]
        z_coeffs = [(2**m) for m in range(Ps)]

        # 5. Add coefficients to Q by expanding (H + C + S + Z + G)^2
        # This is V^T * Coeff_Matrix * V where V = [h..., c, s, z..., 1]
        # We need to add lambda1 * (all expanded terms)
        
        all_indices = h_indices + [c_idx, s_idx] + z_indices
        all_coeffs = h_coeffs + [M, M] + z_coeffs
        
        # Add squared terms and cross-product terms
        for idx_a, (var_idx_1, var_coeff_1) in enumerate(zip(all_indices, all_coeffs)):
            # Squared terms: (coeff*var)^2 -> coeff^2 * var (since var is binary)
            add_to_q(Q, var_idx_1, var_idx_1, lambda1 * var_coeff_1**2)
            
            # Cross-product terms with other variables: 2 * (coeff1*var1) * (coeff2*var2)
            for var_idx_2, var_coeff_2 in zip(all_indices[idx_a+1:], all_coeffs[idx_a+1:]):
                 add_to_q(Q, var_idx_1, var_idx_2, lambda1 * 2 * var_coeff_1 * var_coeff_2)
            
            # Linear terms from constant G: 2 * G * (coeff*var)
            add_to_q(Q, var_idx_1, var_idx_1, lambda1 * 2 * G * var_coeff_1)
            
def populate_equality_penalties(Q, params, mapper):
    """
    Populates Q with coefficients from P2 and P3 equality penalties.
    (Eqs. on page 8)
    """
    print("Populating Equality Penalties (P2, P3)...")
    K, T, Q_conf = params['K'], params['T'], params['Q']
    lambda2, lambda3 = params['lambda_2'], params['lambda_3']

    for k in range(1, K + 1):
        # P2 Penalty: lambda2 * (sum(c_t,k) - 1)^2
        # Expansion: sum(c_t,k*c_j,k for t!=j) - sum(c_t,k) + const
        c_indices_k = [mapper.get_c_idx(t, k) for t in range(1, T + 1)]
        
        # Add linear term -lambda2 to each c_t,k
        for idx in c_indices_k:
            add_to_q(Q, idx, idx, -lambda2)
        
        # Add quadratic term lambda2 for each pair (c_t,k, c_j,k)
        for t1_idx, t2_idx in itertools.combinations(c_indices_k, 2):
            add_to_q(Q, t1_idx, t2_idx, lambda2) # From c_t*c_j + c_j*c_t = 2*c_t*c_j in expansion. The paper simplifies this slightly.
                                                # (sum c_i - 1)^2 = (sum c_i)^2 - 2 sum c_i + 1
                                                # = sum c_i^2 + 2 sum_{i<j} c_i c_j - 2 sum c_i + 1
                                                # = -sum c_i + 2 sum_{i<j} c_i c_j + 1
                                                # The paper uses sum_{t!=j} which is the same as 2 sum_{i<j}
            add_to_q(Q, t1_idx, t2_idx, lambda2) # Adding it again to match the sum_{t!=j} formula
            
        # P3 Penalty: lambda3 * (sum(s_q,k) - 1)^2
        # Same logic as P2
        s_indices_k = [mapper.get_s_idx(q, k) for q in range(1, Q_conf + 1)]

        for idx in s_indices_k:
            add_to_q(Q, idx, idx, -lambda3)
            
        for q1_idx, q2_idx in itertools.combinations(s_indices_k, 2):
            add_to_q(Q, q1_idx, q2_idx, 2*lambda3) # Using the 2*sum{i<j} form



def build_q_matrix(params):
    """
    Main function to build the QUBO matrix Q.
    """
    mapper = VariableMapper(params)
    Q_matrix = np.zeros((mapper.N, mapper.N))

    # Populate Q matrix from each part of the objective function
    populate_path_objective(Q_matrix, params, mapper)
    populate_inequality_penalty(Q_matrix, params, mapper)
    populate_equality_penalties(Q_matrix, params, mapper)

    print("--- Q Matrix Construction Complete ---")
    return Q_matrix


def main():
    """
    Main function to generate the QUBO matrix.
    This function can be called from other scripts.
    """
    # Load configuration with mock data from Inputs.py
    config_with_data = get_config_with_data()
    
    # Build the Q matrix
    Q_final = build_q_matrix(config_with_data)

    # Save the Q matrix to a text file (save in parent directory)
    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output_filename = os.path.join(parent_dir, 'Q_matrix.txt')
    np.savetxt(output_filename, Q_final, fmt='%.6f', delimiter='\t')
    
    print(f"\nFinal Q matrix shape: {Q_final.shape}")
    print(f"Matrix saved to '{output_filename}'")
    
    # Verification: check if it's upper triangular
    if np.allclose(Q_final, np.triu(Q_final)):
        print("Verification successful: The Q matrix is upper-triangular.")
    else:
        print("Verification FAILED: The Q matrix is NOT upper-triangular.")
    
    return Q_final

if __name__ == '__main__':
    main()
