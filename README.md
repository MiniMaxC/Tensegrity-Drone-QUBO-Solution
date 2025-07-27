# Tensegrity Drone Path Planning with QUBO Optimization

A path planning system for tensegrity drones using Quadratic Unconstrained Binary Optimization (QUBO) and quantum-inspired hybrid solvers.

## 🚁 Project Overview

This project provides the complete mathematical framework and a corresponding Python implementation for converting a complex Mixed-Integer Quadratic Program (MIQP) into a Quadratic Unconstrained Binary Optimization (QUBO) model. The source problem is based on the paper "Mixed-Integer-Based Path and Morphing Planning for a Tensegrity Drone" (Appl. Sci. 2022, 12, 5588), and the final QUBO formulation is designed for use with quantum annealers and other specialized QUBO solvers.

The core task is to translate a high-level optimization problem with continuous variables and multiple constraints into the strict QUBO format $\min z^T Q z$, where every variable is binary.

This project implements an optimal path planning algorithm for tensegrity drones navigating through complex environments with multiple convex regions and configuration constraints. The system formulates the path planning problem as a QUBO optimization problem and solves it using the Q Deep hybrid quantum-classical solver.

**🎯 User-Friendly Design**: The core algorithmic components are organized in a `Solver/` folder that acts as a "black box" - users can simply run `python Main.py` and configure parameters in `Inputs.py` without needing to understand the complex mathematical implementation details.

**🎨 Advanced Visualization**: Includes comprehensive 3D visualization capabilities for exploring the path planning environment, tensegrity drone configurations, safe zones, start/finish positions, and solution paths using interactive Plotly visualizations.

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

- **Interactive 3D Visualization**: Comprehensive visualization system featuring:
  - **Environment Visualization**: 3D rendering of safe zones, start/finish positions, and drone placement
  - **Configuration Exploration**: Interactive visualization of all 8 tensegrity drone configurations with nodes, cables, and rods
  - **Multi-Selection Capabilities**: Compare multiple configurations side-by-side or view them sequentially
  - **Path Planning Display**: Visualize solution paths through the environment
  - **Interactive Menu System**: User-friendly interface for accessing different visualization modes

## 📁 Project Structure

```
TensegrityDroneOptimisation/
├── Main.py                                                     # Main workflow orchestrator with visualization options
├── Inputs.py                                                  # Configuration, parameters, and data generation
├── interactive_visualizer.py                                 # Interactive menu system for visualizations
├── Q_matrix.txt                                               # Generated QUBO matrix (output)
├── Solved_Problem                                             # Optimization results (output)
├── Articles/                                                  # Research papers and documentation
│   ├── Drone Path Planning From MIP to QUBO.pdf              # Technical documentation and methodology
│   └── Source Problem - MIBP + Morphing Planning for a Tensegrity Drone.pdf  # Original research paper
├── Solver/                                                    # Core solver components ("black box")
│   ├── quboconstruction.py                                    # QUBO matrix construction with callable main() function
│   ├── qdeepsdksolver.py                                      # Q Deep hybrid solver integration
│   ├── solutiondecoder.py                                     # Binary solution decoding utilities
│   ├── config_loader.py                                       # Configuration loading from .mat files
│   ├── load_config_vis.py                                     # Tensegrity configuration loader and visualizer
│   ├── path_environment_visualizer.py                        # 3D environment and drone visualization
│   └── Sixbar_Drone_12vertices_8configs_NEW_V4.mat           # Real tensegrity drone configuration data
└── README.md                                                  # This file
```

### File Descriptions

**Core Workflow:**
- **`Main.py`**: Complete workflow orchestrator with enhanced command-line options for optimization and visualization
- **`Inputs.py`**: Centralized configuration containing the `config` dictionary, real data loading, and problem parameters

**Visualization System:**
- **`interactive_visualizer.py`**: Interactive menu system providing multiple visualization modes and configuration selection options

**Solver Components:**
- **`Solver/quboconstruction.py`**: Core QUBO matrix construction logic with a callable `main()` function
- **`Solver/qdeepsdksolver.py`**: Q Deep quantum-inspired hybrid solver interface and results processing
- **`Solver/solutiondecoder.py`**: Utilities for decoding binary solutions back to interpretable path and configuration data
- **`Solver/config_loader.py`**: Configuration loading utilities for .mat file processing
- **`Solver/load_config_vis.py`**: Tensegrity drone configuration loader with 3D visualization capabilities
- **`Solver/path_environment_visualizer.py`**: 3D visualization of the path planning environment, safe zones, start/finish positions, and drone placement
- **`Solver/Sixbar_Drone_12vertices_8configs_NEW_V4.mat`**: Real tensegrity drone configuration data (8 configurations, 12 nodes each)

The `Solver/` folder contains the core algorithmic components that can be treated as a "black box" - users typically only need to interact with `Main.py` and the visualization files for configuration and execution.

## 🛠️ Requirements

### Dependencies
```bash
pip install numpy
pip install qdeepsdk
pip install requests
pip install plotly
pip install scipy
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
   pip install numpy qdeepsdk requests plotly scipy
   ```

3. **Configure API Token**:
   - Update the API token in `Solver/qdeepsdksolver.py`:
   ```python
   solver.token = "your_q_deep_api_token_here"
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
1. **Load Configuration**: Import settings from `Inputs.py` with real tensegrity data
2. **Build QUBO Matrix**: Construct the QUBO matrix with all constraints and objectives
3. **Solve Problem**: Use Q Deep hybrid solver for optimization
4. **Interactive Visualization**: Enhanced 3D visualization menu with multiple options

Expected output: Complete execution with solution energy, variable assignments, and interactive visualizations.

### Enhanced Visualization Options

```bash
# Full workflow with visualization menu
python Main.py

# Skip optimization, go directly to visualization menu
python Main.py --visualize-only

# Interactive visualization with full menu system
python Main.py --interactive

# Quick preset visualization examples
python Main.py --quick-examples

# Show configuration details only
python Main.py --config-details
```

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

### Solver Parameters in `Solver/qdeepsdksolver.py`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `m_budget` | Measurement budget | 50000 |
| `num_reads` | Number of solver reads | 10000 |

## 🎨 Visualization Capabilities

The project includes a comprehensive 3D visualization system built with Plotly for exploring and understanding the tensegrity drone path planning problem:

### Environment Visualization
- **Safe Zones**: 3D rendering of convex H-polytope regions (Room1, Corridor, Room2)
- **Start & Goal Positions**: Clearly marked start (green) and goal (red) locations
- **Drone Placement**: Tensegrity drone positioned at the start location
- **Interactive 3D**: Rotate, zoom, and explore the environment

### Configuration Exploration
- **8 Real Configurations**: Visualize all tensegrity drone configurations from the dataset
- **Structural Details**: Nodes (active/passive), cables (24 per config), rods (6 per config)
- **Multi-Selection**: Choose specific configurations ("1 3 5") or view all
- **Side-by-Side Comparison**: Compare up to 6 configurations simultaneously
- **Sequential Viewing**: Browse through configurations one by one

### Visualization Modes
1. **Interactive Menu**: Full-featured menu with all visualization options
2. **Quick Examples**: Preset demonstrations (environment + configurations + comparisons)
3. **Configuration Details**: Summary of all available drone configurations
4. **Specific Selection**: Choose and visualize particular configurations
5. **Multi-Configuration Comparison**: Side-by-side visualization of multiple configurations
6. **Sequential Browser**: View all configurations in sequence

### Technical Features
- **Plotly Integration**: High-quality, interactive 3D graphics
- **Real-Time Interaction**: Rotate, zoom, pan through visualizations
- **Browser-Based**: Visualizations open automatically in your default browser
- **Modular Design**: Separate visualization system that can be used independently

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

A 3D path planning example with real tensegrity data:
- **Dimensions**: 3D space (x, y, z)
- **Time steps**: 3 waypoints (configurable)
- **Nodes per drone**: 12 structural nodes (real tensegrity data)
- **Regions**: 3 convex safe regions (Room1 → Corridor → Room2)
- **Configurations**: 8 real drone morphing configurations
- **Real Data**: Loaded from `Sixbar_Drone_12vertices_8configs_NEW_V4.mat`

Expected output:
```
Loading real configuration data...
✅ Loaded 8 real configurations with 12 nodes each
Creating H-Polytope safe regions:
  ✅ Room1: center=[-5.6 -0. -0.], size=[3.2 3.2 3.2], volume=32.8m³
  ✅ Corridor: center=[0. 0. 0.], size=[8. 2. 3.2], volume=51.2m³
  ✅ Room2: center=[5.6 0. 0.], size=[3.2 3.2 3.2], volume=32.8m³
📍 Start (Room1): [-5.80, 0.35, 0.15]m
🎯 Goal (Room2): [5.97, -0.04, -0.66]m
```

The solver finds optimal binary assignments that minimize the path cost while respecting all region and configuration constraints, followed by interactive 3D visualizations.

## 📄 License

[Add license information here]

## 🙏 Acknowledgments

- Q Deep
- Innopolis University
  
## 📞 Contact

- Email: m.mifsudbonici@innopolis.university
- LinkedIn: www.linkedin.com/in/maximilian-mifsud-bonici-36100b371
---

**Note**: This implementation is for research and educational purposes. The visualization system provides comprehensive tools for exploring tensegrity drone configurations and path planning environments. For production use, consider additional validation and error handling. 