"""This work is in progress, it is not completed, and may not work yet."""

import numpy as np
import open3d as o3d
import os


pointcloud_directory = r'C:\Users\data\pointclouds'


def load_pointclouds_from_directory(directory):
    # load and convert .npy point clouds to Open3D point clouds
    combined_pointcloud = o3d.geometry.PointCloud()
    
    for filename in os.listdir(directory):
        if filename.endswith(".ply"):
            filepath = os.path.join(directory, filename)
            o3d_pc = o3d.io.read_point_cloud(filepath)

            # Filter out zero points (invalid depth)
            # valid_points = points[~np.isnan(points[:, 2])]
            # o3d_pc = o3d.geometry.PointCloud()
            # o3d_pc.points = o3d.utility.Vector3dVector(points)
            
            combined_pointcloud += o3d_pc

    return combined_pointcloud

combined_pc = load_pointclouds_from_directory(pointcloud_directory)

o3d.visualization.draw_geometries([combined_pc])
