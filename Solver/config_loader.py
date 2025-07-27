#!/usr/bin/env python3
"""
Configuration Loader for Tensegrity Drone Optimization

This module loads real configuration data from the MATLAB .mat file
and converts it to the format needed by the QUBO construction.
"""

import numpy as np
from scipy.io import loadmat
import os

class ConfigurationLoader:
    """
    Loads and processes tensegrity drone configuration data from .mat files.
    """
    
    def __init__(self, mat_file_path=None):
        if mat_file_path is None:
            # Try multiple locations for the .mat file
            possible_paths = [
                # Same directory as this file
                os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Sixbar_Drone_12vertices_8configs_NEW_V4.mat'),
                # Parent directory (project root)
                os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Sixbar_Drone_12vertices_8configs_NEW_V4.mat'),
                # Current working directory
                'Sixbar_Drone_12vertices_8configs_NEW_V4.mat'
            ]
            
            mat_file_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    mat_file_path = path
                    break
            
            if mat_file_path is None:
                raise FileNotFoundError("Could not find Sixbar_Drone_12vertices_8configs_NEW_V4.mat in any expected location")
        """
        Initialize the configuration loader.
        
        Args:
            mat_file_path (str): Path to the .mat file containing configurations
        """
        self.mat_file_path = mat_file_path
        self.configs_data = None
        self.num_configs = 0
        self.num_nodes = 0
        self.spatial_dim = 3  # 3D coordinates
        
    def load_configurations(self):
        """
        Load configuration data from the .mat file.
        
        Returns:
            dict: Processed configuration data ready for QUBO construction
        """
        if not os.path.exists(self.mat_file_path):
            raise FileNotFoundError(f"Configuration file not found: {self.mat_file_path}")
            
        print(f"Loading configuration data from {self.mat_file_path}")
        
        try:
            # Load MATLAB data
            mat_data = loadmat(self.mat_file_path)
            
            if 'configs' not in mat_data:
                raise ValueError("No 'configs' field found in .mat file")
            
            configs_raw = mat_data['configs']
            self.num_configs = configs_raw.shape[1]  # Should be 8
            
            print(f"Found {self.num_configs} configurations")
            
            # Extract node positions for each configuration
            config_positions = []
            
            for i in range(self.num_configs):
                config = configs_raw[0, i]  # configs is (1, 8) array
                nodes_position = config['nodes_position']
                
                # nodes_position is (3, 12) - 3D coords for 12 nodes
                if i == 0:
                    self.num_nodes = nodes_position.shape[1]
                    print(f"Each configuration has {self.num_nodes} nodes in {nodes_position.shape[0]}D space")
                
                # Transpose to get (12, 3) format: [node_id, [x, y, z]]
                nodes_coords = nodes_position.T  # Now (12, 3)
                config_positions.append(nodes_coords)
            
            self.configs_data = config_positions
            
            # Convert to format expected by QUBO construction
            processed_data = self._process_for_qubo()
            
            print("✅ Configuration data loaded successfully")
            return processed_data
            
        except Exception as e:
            print(f"❌ Error loading configuration data: {e}")
            raise
    
    def _process_for_qubo(self):
        """
        Process the loaded configurations into the format expected by QUBO construction.
        
        Returns:
            dict: Processed data with r_conf and P_node matrices
        """
        if self.configs_data is None:
            raise ValueError("No configuration data loaded. Call load_configurations() first.")
        
        # Create r_conf vectors: flatten each configuration's node positions
        r_conf_list = []
        for config_nodes in self.configs_data:
            # Flatten (12, 3) -> (36,) vector: [x1,y1,z1, x2,y2,z2, ..., x12,y12,z12]
            r_conf_vector = config_nodes.flatten()
            r_conf_list.append(r_conf_vector)
        
        # Create P_node matrices for extracting individual node coordinates
        # P_i is a (3, 36) matrix that extracts the 3D coordinates of node i
        P_node_list = []
        for node_i in range(self.num_nodes):
            P_i = np.zeros((self.spatial_dim, self.num_nodes * self.spatial_dim))
            start_idx = node_i * self.spatial_dim
            end_idx = start_idx + self.spatial_dim
            P_i[:, start_idx:end_idx] = np.eye(self.spatial_dim)
            P_node_list.append(P_i)
        
        processed_data = {
            'r_conf': r_conf_list,           # List of Q configuration vectors (each length 36)
            'P_node': P_node_list,           # List of n node extraction matrices (each 3×36)
            'num_configs': self.num_configs, # Q = 8
            'num_nodes': self.num_nodes,     # n = 12
            'spatial_dim': self.spatial_dim  # D = 3
        }
        
        print(f"Processed data:")
        print(f"  - {len(processed_data['r_conf'])} configuration vectors, each length {len(processed_data['r_conf'][0])}")
        print(f"  - {len(processed_data['P_node'])} node extraction matrices, each shape {processed_data['P_node'][0].shape}")
        
        return processed_data
    
    def get_configuration_summary(self):
        """
        Get a summary of the loaded configuration data.
        
        Returns:
            dict: Summary statistics
        """
        if self.configs_data is None:
            return {"error": "No data loaded"}
        
        summary = {
            'num_configurations': self.num_configs,
            'num_nodes': self.num_nodes,
            'spatial_dimensions': self.spatial_dim,
            'config_vector_length': self.num_nodes * self.spatial_dim
        }
        
        # Add statistics about node positions
        all_positions = np.vstack(self.configs_data)  # Stack all configs
        summary.update({
            'position_range': {
                'x': [float(all_positions[:, 0].min()), float(all_positions[:, 0].max())],
                'y': [float(all_positions[:, 1].min()), float(all_positions[:, 1].max())],
                'z': [float(all_positions[:, 2].min()), float(all_positions[:, 2].max())]
            },
            'position_mean': [float(all_positions[:, i].mean()) for i in range(3)],
            'position_std': [float(all_positions[:, i].std()) for i in range(3)]
        })
        
        return summary

def load_real_configurations(mat_file_path=None):
    if mat_file_path is None:
        # Try multiple locations for the .mat file
        possible_paths = [
            # Same directory as this file
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Sixbar_Drone_12vertices_8configs_NEW_V4.mat'),
            # Parent directory (project root)
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Sixbar_Drone_12vertices_8configs_NEW_V4.mat'),
            # Current working directory
            'Sixbar_Drone_12vertices_8configs_NEW_V4.mat'
        ]
        
        mat_file_path = None
        for path in possible_paths:
            if os.path.exists(path):
                mat_file_path = path
                break
        
        if mat_file_path is None:
            raise FileNotFoundError("Could not find Sixbar_Drone_12vertices_8configs_NEW_V4.mat in any expected location")
    """
    Convenience function to load configurations.
    
    Args:
        mat_file_path (str): Path to the .mat file
        
    Returns:
        dict: Processed configuration data
    """
    loader = ConfigurationLoader(mat_file_path)
    return loader.load_configurations()

if __name__ == "__main__":
    # Test the configuration loader
    try:
        loader = ConfigurationLoader()
        data = loader.load_configurations()
        summary = loader.get_configuration_summary()
        
        print("\nConfiguration Summary:")
        for key, value in summary.items():
            print(f"  {key}: {value}")
            
    except Exception as e:
        print(f"Error testing configuration loader: {e}") 