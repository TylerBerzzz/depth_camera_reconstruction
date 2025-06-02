import open3d as o3d
import numpy as np
import glob


def load_point_clouds(ply_files):
    pcds = []
    for file in ply_files:
        pcd = o3d.io.read_point_cloud(file)
        pcds.append(pcd)
    return pcds

def pairwise_registration(source, target):
    # Pairwise alignment using ICP. Compute normals for source and target point clouds
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

    threshold = 0.02  # Adjust based on scale of object
    trans_init = np.eye(4)  # Identity matrix for initial transformation
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)  # Convergence criteria

    icp = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria
    )
    transformation_icp = icp.transformation
    print("ICP alignment complete. Fitness:", icp.fitness)
    return transformation_icp


def merge_point_clouds(pcds):
    aligned_pcds = []
    for i in range(1, len(pcds)):
        source = pcds[i - 1]
        target = pcds[i]
        transformation_icp = pairwise_registration(source, target)
        target.transform(transformation_icp)
        aligned_pcds.append(target)
        
    combined = pcds[0]
    for pcd in aligned_pcds:
        combined += pcd
    
    combined = combined.voxel_down_sample(voxel_size=0.005)  # reduce noise
    return combined

# Surface reconstruction using Poisson
def surface_reconstruction(pcd):
    print("Performing surface reconstruction...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)
    
    # Remove low-density areas (noise)
    bbox = pcd.get_axis_aligned_bounding_box()
    mesh = mesh.crop(bbox)
    return mesh

def visualize(*objects):
    o3d.visualization.draw_geometries(objects)


if __name__ == "__main__":
    pointcloud_directory = r'C:\Users\data\pointclouds\**'

    ply_files = glob.glob(pointcloud_directory)  
    
    point_clouds = load_point_clouds(ply_files)
    
    print("Merging point clouds...")
    merged_cloud = merge_point_clouds(point_clouds)
    
    print("Reconstructing surface...")
    mesh = surface_reconstruction(merged_cloud)
    
    o3d.io.write_point_cloud("merged_cloud.ply", merged_cloud)
    o3d.io.write_triangle_mesh("reconstructed_mesh.ply", mesh)
    
    visualize(merged_cloud, mesh)
