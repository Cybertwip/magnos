import open3d as o3d
import numpy as np

def visualize_pcd(file_path):
    # Load PCD file
    pcd = o3d.io.read_point_cloud(file_path)

    # Create a rotation matrix for 90 degrees in the Z-axis
    rotation_matrix = np.array([[0, -1, 0, 0],
                                [-1, 0, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])

    # Apply the rotation to the point cloud
    pcd.transform(rotation_matrix)

    # Visualize the rotated point cloud
    o3d.visualization.draw_geometries([pcd])


# Specify the path to your PCD file
pcd_file_path = ""

# Visualize the PCD file
visualize_pcd(pcd_file_path)
