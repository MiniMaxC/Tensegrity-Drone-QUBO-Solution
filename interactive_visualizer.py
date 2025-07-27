#!/usr/bin/env python3
"""
Interactive Tensegrity Drone Environment Visualizer

This script provides an easy interface to visualize the path planning environment
with different drone configurations and view options.
"""

from Solver.path_environment_visualizer import PathEnvironmentVisualizer
import numpy as np


def show_available_configurations():
    """Display information about available drone configurations"""
    visualizer = PathEnvironmentVisualizer()
    print("=" * 50)
    print("AVAILABLE DRONE CONFIGURATIONS")
    print("=" * 50)
    
    config_data = visualizer.config_data
    print(f"Total configurations available: {len(config_data['robots'])}")
    
    for i, robot in enumerate(config_data['robots']):
        print(f"\nConfiguration {i + 1}:")
        print(f"  Active nodes: {robot['active_nodes']}")
        print(f"  Total cables: {np.sum(robot['Cables']) // 2}")  # Divided by 2 because matrix is symmetric
        print(f"  Total rods: {np.sum(robot['Rods']) // 2}")
    
    print("=" * 50)
    return len(config_data['robots'])


def visualize_specific_config(config_idx, show_drone=True):
    """Visualize environment with a specific drone configuration"""
    print(f"\n🚁 Visualizing with drone configuration {config_idx + 1}...")
    
    visualizer = PathEnvironmentVisualizer()
    fig = visualizer.visualize_environment(show_drone=show_drone, drone_config_idx=config_idx)
    
    if show_drone:
        fig.update_layout(title=f"Environment with Drone Configuration {config_idx + 1}")
    else:
        fig.update_layout(title="Environment (No Drone)")
    
    fig.show()
    return fig


def compare_configurations(config_indices):
    """Create side-by-side comparison of different drone configurations"""
    from plotly.subplots import make_subplots
    import plotly.graph_objects as go
    
    print(f"\n🔍 Comparing drone configurations: {[i+1 for i in config_indices]}")
    
    visualizer = PathEnvironmentVisualizer()
    
    cols = min(len(config_indices), 3)  # Max 3 columns
    rows = (len(config_indices) + cols - 1) // cols
    
    fig = make_subplots(
        rows=rows, cols=cols,
        specs=[[{'type': 'scene'} for _ in range(cols)] for _ in range(rows)],
        subplot_titles=[f"Configuration {i+1}" for i in config_indices]
    )
    
    for idx, config_idx in enumerate(config_indices):
        row = idx // cols + 1
        col = idx % cols + 1
        
        # Create individual visualization
        temp_fig = visualizer.visualize_environment(show_drone=True, drone_config_idx=config_idx)
        
        # Add traces to subplot
        for trace in temp_fig.data:
            trace.showlegend = idx == 0  # Only show legend for first subplot
            fig.add_trace(trace, row=row, col=col)
    
    fig.update_layout(
        title="Drone Configuration Comparison",
        height=400 * rows,
        width=1200
    )
    
    fig.show()
    return fig


def interactive_menu():
    """Main interactive menu for the visualizer"""
    print("🚁" + "=" * 58 + "🚁")
    print("  INTERACTIVE TENSEGRITY DRONE ENVIRONMENT VISUALIZER")
    print("🚁" + "=" * 58 + "🚁")
    
    num_configs = show_available_configurations()
    
    while True:
        print("\n📋 VISUALIZATION OPTIONS:")
        print("1. View environment with drone (choose configuration)")
        print("2. View environment without drone")
        print("3. Compare multiple drone configurations")
        print("4. View all configurations sequentially") 
        print("5. Show configuration details")
        print("0. Exit")
        
        choice = input("\nEnter your choice (0-5): ").strip()
        
        if choice == "0":
            print("👋 Goodbye!")
            break
            
        elif choice == "1":
            try:
                config_num = int(input(f"Enter configuration number (1-{num_configs}): ")) - 1
                if 0 <= config_num < num_configs:
                    visualize_specific_config(config_num, show_drone=True)
                else:
                    print(f"❌ Invalid configuration. Please choose 1-{num_configs}")
            except ValueError:
                print("❌ Please enter a valid number")
                
        elif choice == "2":
            visualize_specific_config(0, show_drone=False)
            
        elif choice == "3":
            try:
                config_input = input(f"Enter configuration numbers separated by spaces (1-{num_configs}): ")
                config_nums = [int(x) - 1 for x in config_input.split()]
                
                if all(0 <= c < num_configs for c in config_nums):
                    if len(config_nums) <= 6:  # Limit to 6 for readability
                        compare_configurations(config_nums)
                    else:
                        print("❌ Too many configurations. Please choose 6 or fewer.")
                else:
                    print(f"❌ Invalid configuration numbers. Please choose from 1-{num_configs}")
            except ValueError:
                print("❌ Please enter valid numbers separated by spaces")
                
        elif choice == "4":
            print(f"\n🎬 Showing all {num_configs} configurations...")
            for i in range(num_configs):
                print(f"Configuration {i + 1}/{num_configs}")
                visualize_specific_config(i, show_drone=True)
                if i < num_configs - 1:
                    input("Press Enter for next configuration...")
                    
        elif choice == "5":
            show_available_configurations()
            
        else:
            print("❌ Invalid choice. Please try again.")


def quick_visualization_examples():
    """Show some predefined interesting visualizations"""
    print("\n🎯 QUICK EXAMPLES:")
    print("=" * 30)
    
    # Environment overview
    print("1. Environment overview (no drone)")
    visualize_specific_config(0, show_drone=False)
    
    # First configuration example
    print("\n2. Environment with first drone configuration")
    visualize_specific_config(0, show_drone=True)
    
    # Compare configurations
    visualizer = PathEnvironmentVisualizer()
    num_configs = len(visualizer.config_data['robots'])
    if num_configs >= 3:
        print("\n3. Comparison of first 3 configurations")
        compare_configurations([0, 1, 2])


if __name__ == "__main__":
    # Check if user wants interactive mode or quick examples
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--quick":
        print("🚀 Running quick visualization examples...")
        quick_visualization_examples()
    else:
        print("🎮 Starting interactive mode...")
        print("   (Use --quick flag for preset examples)")
        interactive_menu() 