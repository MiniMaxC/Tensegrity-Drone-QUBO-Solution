#!/usr/bin/env python3
"""
Complete Workflow Script for Tensegrity Drone QUBO

This script runs the complete workflow:
1. Generate QUBO matrix using CompactQUBOConstruction
2. Wait briefly to ensure file is written
3. Solve the QUBO problem using FinalQUBOSolver
4. Enhanced 3D visualization with multiple options

Usage:
    python Main.py                    # Full workflow with visualization menu
    python Main.py --visualize-only   # Visualization menu only (skip optimization)
    python Main.py --interactive      # Interactive visualization with full menu
    python Main.py --quick-examples   # Preset visualization examples
    python Main.py --config-details   # Show configuration details only
"""

import time
import sys
import os
import subprocess
import importlib.util
import numpy as np
from Solver.solutiondecoder import decode_solution
from Solver.quboconstruction import VariableMapper
from Inputs import get_config_with_data
from Solver.load_config_vis import ConfigurationLoader, vis_draw
from interactive_visualizer import (
    show_available_configurations,
    visualize_specific_config,
    compare_configurations,
    interactive_menu,
    quick_visualization_examples
)

def run_qubo_construction():
    """Run the QUBO matrix construction."""
    print("=" * 60)
    print("STEP 1: GENERATING QUBO MATRIX")
    print("=" * 60)
    
    try:
        # Import and run QUBO construction
        spec = importlib.util.spec_from_file_location("quboconstruction", "Solver/quboconstruction.py")
        qubo_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(qubo_module)
        
        # Run the main function
        qubo_module.main()
        return True
        
    except Exception as e:
        print(f"❌ Error during QUBO construction: {e}")
        return False

def run_qubo_solver():
    """Run the QUBO solver."""
    print("\n" + "=" * 60)
    print("STEP 2: SOLVING QUBO PROBLEM")
    print("=" * 60)
    
    try:
        # Check if Q_matrix.txt exists
        if not os.path.exists("Q_matrix.txt"):
            print("❌ Error: Q_matrix.txt not found!")
            return False
        
        # Import and run QUBO solver
        spec = importlib.util.spec_from_file_location("qdeepsdksolver", "Solver/qdeepsdksolver.py")
        solver_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(solver_module)
        
        return True
        
    except Exception as e:
        print(f"❌ Error during QUBO solving: {e}")
        return False

def visualize_configurations():
    """Enhanced visualization using interactive visualizer capabilities."""
    try:
        print("🎨 ENHANCED VISUALIZATION OPTIONS")
        print("=" * 50)
        print("Choose your visualization mode:")
        print("1. Interactive Menu (Full-featured)")
        print("2. Quick Examples (Environment + Configurations)")
        print("3. Configuration Details Only")
        print("4. Select Specific Configurations")
        print("5. Compare Multiple Configurations")
        print("6. View All Configurations Sequentially")
        print("0. Skip Visualization")
        
        choice = input("\nEnter your choice (0-6): ").strip()
        
        if choice == "0":
            print("⏭️  Skipping visualization")
            return
            
        elif choice == "1":
            print("\n🎮 Starting Interactive Menu...")
            interactive_menu()
            
        elif choice == "2":
            print("\n🚀 Running Quick Examples...")
            quick_visualization_examples()
            
        elif choice == "3":
            print("\n📊 Configuration Details:")
            num_configs = show_available_configurations()
            
        elif choice == "4":
            print("\n🎯 Select Specific Configurations:")
            num_configs = show_available_configurations()
            
            try:
                config_input = input(f"\nEnter configuration numbers (1-{num_configs}), separated by spaces: ")
                config_nums = [int(x) - 1 for x in config_input.split() if x.isdigit()]
                
                # Filter valid indices
                valid_configs = [i for i in config_nums if 0 <= i < num_configs]
                
                if not valid_configs:
                    print("⚠️  No valid configurations selected, showing first configuration")
                    valid_configs = [0]
                
                print(f"\n🚁 Visualizing {len(valid_configs)} configuration(s)...")
                for config_idx in valid_configs:
                    print(f"\nShowing Configuration {config_idx + 1}")
                    visualize_specific_config(config_idx, show_drone=True)
                    if len(valid_configs) > 1 and config_idx != valid_configs[-1]:
                        input("Press Enter for next configuration...")
                        
            except ValueError:
                print("❌ Invalid input. Showing first configuration.")
                visualize_specific_config(0, show_drone=True)
                
        elif choice == "5":
            print("\n🔍 Compare Multiple Configurations:")
            num_configs = show_available_configurations()
            
            try:
                config_input = input(f"\nEnter 2-6 configuration numbers (1-{num_configs}) to compare: ")
                config_nums = [int(x) - 1 for x in config_input.split() if x.isdigit()]
                
                # Filter valid indices
                valid_configs = [i for i in config_nums if 0 <= i < num_configs]
                
                if len(valid_configs) < 2:
                    print("⚠️  Need at least 2 configurations for comparison. Showing first 3.")
                    valid_configs = [0, 1, 2] if num_configs >= 3 else list(range(min(num_configs, 2)))
                elif len(valid_configs) > 6:
                    print("⚠️  Too many configurations. Showing first 6 selected.")
                    valid_configs = valid_configs[:6]
                
                print(f"\n🔍 Comparing {len(valid_configs)} configurations...")
                compare_configurations(valid_configs)
                
            except ValueError:
                print("❌ Invalid input. Comparing first 3 configurations.")
                compare_configurations([0, 1, 2] if num_configs >= 3 else [0, 1])
                
        elif choice == "6":
            print("\n🎬 Viewing All Configurations Sequentially...")
            num_configs = show_available_configurations()
            
            for i in range(num_configs):
                print(f"\nConfiguration {i + 1}/{num_configs}")
                visualize_specific_config(i, show_drone=True)
                if i < num_configs - 1:
                    input("Press Enter for next configuration...")
                    
        else:
            print("❌ Invalid choice. Running quick examples...")
            quick_visualization_examples()
        
        print("\n✅ Visualization complete!")
        
    except Exception as e:
        print(f"❌ Error during visualization: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Main workflow function."""
    # Check for various command-line options
    if len(sys.argv) > 1:
        option = sys.argv[1]
        
        if option == "--visualize-only":
            print("🚁 TENSEGRITY DRONE VISUALIZATION ONLY")
            print("=" * 50)
            visualize_configurations()
            return
            
        elif option == "--interactive":
            print("🎮 INTERACTIVE VISUALIZATION MODE")
            print("=" * 40)
            interactive_menu()
            return
            
        elif option == "--quick-examples":
            print("🚀 QUICK VISUALIZATION EXAMPLES")
            print("=" * 35)
            quick_visualization_examples()
            return
            
        elif option == "--config-details":
            print("📊 CONFIGURATION DETAILS ONLY")
            print("=" * 35)
            show_available_configurations()
            return
    
    print("🚁 TENSEGRITY DRONE PATH PLANNING - COMPLETE WORKFLOW")
    print("=" * 60)
    print("This script will:")
    print("  1. Generate the QUBO matrix")
    print("  2. Solve the optimization problem")
    print("  3. Enhanced visualization with multiple options")
    print("\n💡 Available options:")
    print("  --visualize-only     : Skip optimization, show visualization menu")
    print("  --interactive        : Go directly to interactive visualization")
    print("  --quick-examples     : Show preset visualization examples")
    print("  --config-details     : Show configuration details only")
    print("=" * 60)
    
    start_time = time.time()
    
    # Generate QUBO matrix
    construction_success = run_qubo_construction()
    
    if not construction_success:
        print("\n❌ WORKFLOW FAILED: Could not generate QUBO matrix")
        sys.exit(1)
    
    # Brief pause to ensure file is written properly
    print("\n⏳ Waiting for file system sync...")
    time.sleep(1)
    
    # Solve QUBO problem
    solver_success = run_qubo_solver()
    
    # Final status
    total_time = time.time() - start_time
    
    print("\n" + "=" * 60)
    print("WORKFLOW SUMMARY")
    print("=" * 60)
    
    if construction_success and solver_success:
        print("✅ SUCCESS: Complete workflow executed successfully!")
        print(f"⏱️  Total execution time: {total_time:.2f} seconds")
        print("\n📊 Results:")
        print("  - QUBO matrix generated and saved to Q_matrix.txt")
        print("  - Optimization problem solved using QDeep hybrid solver")
        print("  - Enhanced 3D visualization with interactive features")
        print("\n🎯 Available visualizations included:")
        print("  - Environment with safe zones, start/finish positions")
        print("  - Individual tensegrity configurations")
        print("  - Side-by-side configuration comparisons")
        print("  - Interactive 3D exploration capabilities")
    else:
        print("❌ WORKFLOW INCOMPLETE:")
        if not construction_success:
            print("  - QUBO matrix generation failed")
        if not solver_success:
            print("  - QUBO solver execution failed")
        sys.exit(1)
    
    
    # Configuration Visualization
    if construction_success and solver_success:
        print("\n" + "=" * 60)
        print("STEP 3: CONFIGURATION VISUALIZATION")
        print("=" * 60)
        
        visualize_configurations()
    
    print("=" * 60)
    print("🎯 Workflow complete! Check results above.")
if __name__ == "__main__":
    main() 