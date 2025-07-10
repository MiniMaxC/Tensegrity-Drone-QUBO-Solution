# Tensegrity Drone Path Planning with QUBO Optimization

A path planning system for tensegrity drones using Quadratic Unconstrained Binary Optimization (QUBO) and quantum-inspired hybrid solvers.

## 🚁 Project Overview

This project provides the complete mathematical framework and a corresponding Python implementation for converting a complex Mixed-Integer Quadratic Program (MIQP) into a Quadratic Unconstrained Binary Optimization (QUBO) model. The source problem is based on the paper "Mixed-Integer-Based Path and Morphing Planning for a Tensegrity Drone" (Appl. Sci. 2022, 12, 5588), and the final QUBO formulation is designed for use with quantum annealers and other specialized QUBO solvers.

The core task is to translate a high-level optimization problem with continuous variables and multiple constraints into the strict QUBO format $\min z^T Q z$, where every variable is binary.

This project implements an optimal path planning algorithm for tensegrity drones navigating through complex environments with multiple convex regions and configuration constraints. The system formulates the path planning problem as a QUBO optimization problem and solves it using the Q Deep hybrid quantum-classical solver.

**🎯 User-Friendly Design**: The core algorithmic components are organized in a `Solver/` folder that acts as a "black box" - users can simply run `python Main.py` and configure parameters in `Inputs.py` without needing to understand the complex mathematical implementation details.

## 🔬 Methodology

The conversion follows a rigorous, multi-stage mathematical process:

### 1. Constrained to Unconstrained Conversion
The initial step is to transform the original MIQP into an unconstrained problem. This is achieved by moving all operational constraints (both equalities and inequalities) into the main objective function as quadratic penalty terms. A large penalty coefficient $\lambda$ is used to ensure that any violation of a constraint results in a high cost, making such solutions undesirable.

### 2. Handling Constraints

**Equality Constraints**: Constraints of the form $f(x) = 0$ are converted using the standard $(f(x))^2$ penalty method.

**Inequality Constraints**: To ensure mathematical robustness, each scalar inequality constraint (e.g., ensuring a single drone node is within a single plane of a safe region) is handled independently. A unique slack variable is introduced for each inequality to convert it into an equality, which is then penalized. The total penalty is the "sum of squares" of all these individual constraint equations.

### 3. Binarization of Variables
The QUBO format exclusively uses binary variables. Therefore, all non-binary variables from the problem must be converted:

Continuous variables (like the drone's position coordinates) and integer variables (like the slack variables) are represented using a binary expansion. For example, a variable $v$ is approximated as $v = \sum_p 2^p h_p$, where $h_p$ are new binary variables.

### 4. Expansion to a Binary Polynomial
After all variables are represented in binary form, the entire unconstrained objective function is algebraically expanded. This results in a single, large quadratic polynomial where every term is either a constant, a single binary variable (linear term), or a product of two binary variables (quadratic term).

### 5. Mapping to the QUBO Matrix ($Q$)
The final step is to map the coefficients from the binary polynomial to the QUBO matrix $Q$. The coefficient of each linear term is placed on the matrix diagonal, and the coefficient of each quadratic term is placed on the off-diagonal.

## ✨ Features

- **Multi-Objective Optimization**: The model simultaneously optimizes for path distance (start to goal), path smoothness, and volumetric obstacle avoidance.

- **Complex Environment Navigation**: Handles navigation through environments composed of multiple, distinct convex regions (H-polytopes).

- **Drone Configuration Management**: Supports a dataset of different drone shapes (configurations) and selects the optimal one for each step of the path using one-hot constraints.

- **QUBO Formulation**: Provides a complete mathematical and programmatic workflow to convert the entire MIQP into a standard QUBO format.

- **Solver-Agnostic Output**: Generates a final QUBO matrix ($Q$) that can be used with any quantum annealer, quantum-inspired optimizer, or classical QUBO solver. Here Q Deep's Qonquester platform is used.

- **Automated Matrix Construction**: Includes a Python script that automates the end-to-end process of building the QUBO matrix from defined problem parameters.

- **Modular Design**: Clean separation of concerns with centralized configuration (`Inputs.py`), workflow orchestration (`Main.py`), and core solver components organized in a dedicated `Solver/` folder for matrix construction and optimization.

- **Mock Data Generation**: Automatic generation of realistic problem data for testing and development, with easy replacement for real-world scenarios.

## 📁 Project Structure

```
TensegrityDroneOptimisation/
├── Main.py                                                     # Main workflow orchestrator - runs complete pipeline
├── Inputs.py                                                  # Configuration, parameters, and mock data generation
├── Q_matrix.txt                                               # Generated QUBO matrix (output)
├── Solved_Problem                                             # Optimization results (output)
├── Articles/                                                  # Research papers and documentation
│   ├── Drone Path Planning From MIP to QUBO.pdf              # Technical documentation and methodology
│   └── Source Problem - MIBP + Morphing Planning for a Tensegrity Drone.pdf  # Original research paper
├── Solver/                                                    # Core solver components ("black box")
│   ├── quboconstruction.py                                    # QUBO matrix construction with callable main() function
│   └── qdeepsdksolver.py                                      # Q Deep hybrid solver integration
└── README.md                                                  # This file
```

### File Descriptions

- **`Main.py`**: Complete workflow orchestrator that imports and runs both QUBO construction and solving in sequence
- **`Inputs.py`**: Centralized configuration containing the `config` dictionary, mock data generation function, and problem parameters
- **`Solver/quboconstruction.py`**: Core QUBO matrix construction logic with a callable `main()` function for integration with other scripts
- **`Solver/qdeepsdksolver.py`**: Q Deep quantum-inspired hybrid solver interface and results processing

The `Solver/` folder contains the core algorithmic components that can be treated as a "black box" - users typically only need to interact with `Main.py` and `Inputs.py` for configuration and execution.

## 🛠️ Requirements

### Dependencies
```bash
pip install numpy
pip install qdeepsdk
pip install requests
```

### System Requirements
- Python 3.7+
- Internet connection (for Q Deep API access)
- Q Deep account and API token

## ⚙️ Setup

1. **Clone the repository**:
   ```bash
   git clone <https://github.com/MiniMaxC/Tensegrity-Drone-QUBO-Solution.git>
   cd Tensegrity-Drone-QUBO-Solution
   ```

2. **Install dependencies**:
   ```bash
   pip install numpy qdeepsdk requests
   ```

3. **Configure API Token**:
   - Update the API token in `Solver/qdeepsdksolver.py`:
   ```python
   solver.token = "your_qdeep_api_token_here"
   ```

4. **Adjust Problem Parameters** (optional):
   - Modify parameters in `Inputs.py` to customize your problem

## 🚀 Usage

### Quick Start - Complete Workflow
Run the complete end-to-end workflow with default parameters:

```bash
python Main.py
```

This orchestrates the entire pipeline:
1. **Load Configuration**: Import settings from `Inputs.py`
2. **Generate Mock Data**: Create random problem matrices and vectors
3. **Build QUBO Matrix**: Construct the 410×410 QUBO matrix with all constraints and objectives
4. **Save Matrix**: Export to `Q_matrix.txt` (~1.5MB)
5. **Solve Problem**: Use Q Deep hybrid solver for optimization
6. **Save Results**: Export solution to `Solved_Problem`

Expected output: Complete execution in ~10-15 seconds with solution energy and variable assignments.

### Individual Components

**Generate QUBO Matrix Only**:
```bash
python Solver/quboconstruction.py
# or programmatically:
python -c "from Solver.quboconstruction import main; main()"
```

**Solve Existing QUBO Matrix** (requires `Q_matrix.txt`):
```bash
python Solver/qdeepsdksolver.py
```

**View Configuration Summary**:
```bash
python Inputs.py
```

### Customization

**Modify Problem Parameters**:
Edit the `config` dictionary in `Inputs.py`:
```python
config = {
    'K': 3,        # Increase time steps
    'D': 2,        # Change to 2D problem
    'P': 4,        # Higher precision (larger matrix!)
    # ... other parameters
}
```

**Use Real Data Instead of Mock**:
Replace `generate_mock_data()` calls with your actual constraint matrices, drone configurations, and start/goal positions.

## 📊 Problem Formulation

The system optimizes the following objective function:

$$\text{minimize: } \Phi_{\text{start}} + \Phi_{\text{goal}} + w \cdot \Phi_{\text{smooth}} + \lambda_1 \cdot P_1 + \lambda_2 \cdot P_2 + \lambda_3 \cdot P_3$$

Where:
- **$\Phi_{\text{start}}$**: Distance from start position
- **$\Phi_{\text{goal}}$**: Distance to goal position  
- **$\Phi_{\text{smooth}}$**: Path smoothness penalty
- **$P_1$**: Inequality constraint violations
- **$P_2$**: Region selection one-hot constraint
- **$P_3$**: Configuration selection one-hot constraint

## 🔧 Configuration

### Key Parameters in `Inputs.py`:

The main configuration is stored in the `config` dictionary, with automatic mock data generation for matrices and vectors.

| Parameter | Description | Default |
|-----------|-------------|---------|
| `K` | Number of time steps | 2 |
| `D` | Spatial dimensions | 3 |
| `n` | Number of drone nodes | 4 |
| `P` | Binary precision bits for position | 3 |
| `P_s` | Binary precision bits for slack | 2 |
| `T` | Number of convex regions | 2 |
| `Q` | Number of drone configurations | 2 |
| `U` | Constraints per convex region | 6 |
| `S` | Position scaling factor | 100.0 |
| `x_shift` | Position offset | 5.0 |
| `w` | Smoothness weight | 1.0 |
| `lambda_1` | Inequality constraint penalty | 10.0 |
| `lambda_2` | Region selection penalty | 10.0 |
| `lambda_3` | Configuration selection penalty | 10.0 |
| `M` | Big-M constant | 100.0 |

### Generated Data (via `generate_mock_data()`):
- **`x_start`, `x_goal`**: Random start/goal positions
- **`A`, `b`**: Convex region constraint matrices and vectors
- **`r_conf`**: Drone configuration shape vectors
- **`P_node`**: Node indexing matrices for constraint mapping

### Solver Parameters in `Solver/Q Deepsdksolver.py`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `m_budget` | Measurement budget | 50000 |
| `num_reads` | Number of solver reads | 10000 |

## 📈 Output Files

### `Q_matrix.txt`
- Generated QUBO matrix in text format
- Square matrix of size `total_vars` $\times$ `total_vars`
- Contains all objective and penalty terms

### `Solved_Problem`
- Optimization results from Q Deep solver
- Includes problem metadata and solution statistics
- Contains binary variable assignments

## 🔬 Technical Details

### Variable Encoding
- **Position variables** ($h$): Binary-encoded spatial coordinates
- **Region variables** ($c$): One-hot selection of convex regions
- **Configuration variables** ($s$): One-hot selection of drone configurations
- **Slack variables** ($z$): Constraint violation handling

### Constraint Handling
- **Convex regions**: Linear inequality constraints $A_t \cdot x \leq b_t$
- **One-hot constraints**: Exactly one region/configuration per time step
- **Penalty method**: Soft constraints with configurable weights

## 🐛 Troubleshooting

### Common Issues

1. **API Token Error**:
   ```
   Error: Invalid or missing API token
   ```
   **Solution**: Update your Q Deep API token in `Solver/qdeepsdksolver.py`

2. **Matrix File Not Found**:
   ```
   Error: Matrix file 'Q_matrix.txt' not found
   ```
   **Solution**: Run QUBO construction first (`python Solver/quboconstruction.py`) or check file permissions

3. **Import Error from Inputs.py**:
   ```
   ModuleNotFoundError: No module named 'Inputs'
   ```
   **Solution**: Ensure you're running from the project directory and `Inputs.py` exists

4. **Missing main() Function**:
   ```
   AttributeError: module 'quboconstruction' has no attribute 'main'
   ```
   **Solution**: This is now fixed - `Solver/quboconstruction.py` has a callable `main()` function

5. **Memory Issues with Large Problems**:
   - Reduce problem size in `Inputs.py` config dictionary
   - Lower precision bits `P` and `P_s`
   - Reduce time steps `K` or drone nodes `n`
   - Matrix size scales as O(K×D×P + K×T + K×Q + n×K×T×Q×U×P_s)²

### Performance Tips
- Start with small problem sizes (`K=3, P=3`)
- Increase `m_budget` for better solution quality
- Adjust penalty weights for better constraint satisfaction

## 📝 Example

A 3D path planning example with default parameters:
- **Dimensions**: 3D space (x, y, z)
- **Time steps**: 2 waypoints
- **Nodes per drone**: 4 structural nodes
- **Regions**: 2 convex safe regions (6 constraints each)
- **Configurations**: 2 drone morphing settings
- **Variables**: 410 total binary variables (18 position + 8 selection + 384 slack)

Expected output:
```
--- Variable Mapping Initialized ---
Total number of variables (N): 410
  - Position vars (h): 18
  - Region choice vars (c): 4  
  - Config choice vars (s): 4
  - Slack vars (z): 384
------------------------------------
Final Q matrix shape: (410, 410)
Matrix saved to 'Q_matrix.txt'
Verification successful: The Q matrix is upper-triangular.
```

The solver will find optimal binary assignments that minimize the path cost while respecting all region and configuration constraints.

## 📄 License

[Add license information here]

## 🙏 Acknowledgments

- Q Deep
- Innopolis University
- Hadi Salloum, Amer Albadr

## 📞 Contact

- Email: m.mifsudbonici@innopolis.university
- LinkedIn: www.linkedin.com/in/maximilian-mifsud-bonici-36100b371
---

**Note**: This implementation is for research and educational purposes. For production use, consider additional validation and error handling. 
