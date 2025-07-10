#!/usr/bin/env python3
"""
Complete Workflow Script for Tensegrity Drone QUBO

This script runs the complete workflow:
1. Generate QUBO matrix using CompactQUBOConstruction
2. Wait briefly to ensure file is written
3. Solve the QUBO problem using FinalQUBOSolver

Usage:
    python run_complete_workflow.py
"""

import time
import sys
import os
import subprocess
import importlib.util

def run_qubo_construction():
    """Run the QUBO matrix construction."""
    print("=" * 60)
    print("STEP 1: GENERATING QUBO MATRIX")
    print("=" * 60)
    
    try:
        # Import and run CompactQUBOConstruction from Solver folder
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
        
        # Import and run FinalQUBOSolver from Solver folder
        spec = importlib.util.spec_from_file_location("qdeepsdksolver", "Solver/qdeepsdksolver.py")
        solver_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(solver_module)
        
        return True
        
    except Exception as e:
        print(f"❌ Error during QUBO solving: {e}")
        return False

def main():
    """Main workflow function."""
    print("🚁 TENSEGRITY DRONE PATH PLANNING - COMPLETE WORKFLOW")
    print("=" * 60)
    print("This script will:")
    print("  1. Generate the QUBO matrix")
    print("  2. Solve the optimization problem")
    print("  3. Display results")
    print("=" * 60)
    
    start_time = time.time()
    
    # Step 1: Generate QUBO matrix
    construction_success = run_qubo_construction()
    
    if not construction_success:
        print("\n❌ WORKFLOW FAILED: Could not generate QUBO matrix")
        sys.exit(1)
    
    # Brief pause to ensure file is written properly
    print("\n⏳ Waiting for file system sync...")
    time.sleep(1)
    
    # Step 2: Solve QUBO problem
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
        print("  - Check the output above for solution details")
    else:
        print("❌ WORKFLOW INCOMPLETE:")
        if not construction_success:
            print("  - QUBO matrix generation failed")
        if not solver_success:
            print("  - QUBO solver execution failed")
        sys.exit(1)
    
    print("=" * 60)
    print("🎯 Workflow complete! Check results above.")

if __name__ == "__main__":
    main() 