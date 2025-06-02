import open3d as o3d

file_path = r"C:\data\pointclouds\pointcloud_0.ply"

try:
    pcd = o3d.io.read_point_cloud(file_path)
    print("Point cloud loaded successfully.")

    o3d.visualization.draw_geometries([pcd])
except Exception as e:
    print(f"Failed to load point cloud: {e}")