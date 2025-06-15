import open3d as o3d
import numpy as np
import copy

def calculate_surface_curvature(pcd, radius=1000, max_nn=25):
    pcd_n = copy.deepcopy(pcd)
    pcd_n.estimate_covariances(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )
    covs = np.asarray(pcd_n.covariances)
    vals, vecs = np.linalg.eig(covs)
    curvature = np.min(vals, axis=1) / np.sum(vals, axis=1)
    return curvature

def downsample_with_labels(points, labels=None, voxel_size=5):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    min_bound = points.min(axis=0)
    max_bound = points.max(axis=0)
    
    # 对点云进行体素下采样，生成下采样后的点云
    down_pcd, _, trace = pcd.voxel_down_sample_and_trace(
        voxel_size, min_bound, max_bound, approximate_class=True
    )

    downsampled_points = np.asarray(down_pcd.points)

    if labels is not None:
        downsampled_labels = []
        for i in range(len(downsampled_points)):
            voxel_center = downsampled_points[i]
            original_indices = trace[i]

            if len(original_indices) == 0:
                continue

            original_points = points[original_indices]
            distances = np.linalg.norm(original_points - voxel_center, axis=1)
            nearest_idx = original_indices[np.argmin(distances)]
            downsampled_labels.append(labels[nearest_idx])

        downsampled_labels = np.array(downsampled_labels)
    else:
        downsampled_labels = None

    down_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30),
        fast_normal_computation=True,
    )
    downsampled_normals = np.asarray(down_pcd.normals)
    downsampled_curvatures = calculate_surface_curvature(
        down_pcd, radius=500, max_nn=30
    )

    return (
        downsampled_points,
        downsampled_labels,
        downsampled_normals,
        downsampled_curvatures,
    )