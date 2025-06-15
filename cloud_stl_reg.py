# -*- coding: utf-8 -*-

import numpy as np
import open3d as o3d

import numpy as np
from sklearn.neighbors import NearestNeighbors
import os
from trans_stp import trans_stp
from downsample import downsample_with_labels

def best_fit_transform(A, B):
    """
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    """

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        Vt[m - 1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R, centroid_A.T)

    # homogeneous transformation
    T = np.identity(m + 1)
    T[:m, :m] = R
    T[:m, m] = t
    T_inv = np.identity(m + 1)
    T_inv[:m, :m] = R.T
    T_inv[:m, m] = -np.dot(R.T, t)

    return T, T_inv


def nearest_neighbor(src, dst):
    """
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    """

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.000001):
    """
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    """

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m + 1, A.shape[0]))
    dst = np.ones((m + 1, B.shape[0]))
    src[:m, :] = np.copy(A.T)
    dst[:m, :] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m, :].T, dst[:m, :].T)
        print(np.mean(distances))

        # compute the transformation between the current source and nearest destination points
        T, T_inv = best_fit_transform(src[:m, :].T, dst[:m, indices].T)

        # update the current source
        src = np.dot(T, src)

        
        # # check error
        # mean_error = np.mean(distances)
        # if np.abs(prev_error - mean_error) < tolerance:
        #     print(i, "small error improvement, break")
        #     break
        # prev_error = mean_error
        

    # calculate final transformation
    T, T_inv = best_fit_transform(A, src[:m, :].T)

    return T, T_inv, distances, i

def reg(stp_path, stl_path, pcd_path, save_file_path):

    # 1. 读取源点云
    pcd_data = np.loadtxt(pcd_path)[:, :3]
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(pcd_data[:, :3])
    pcd_points = np.array(source.points, order="F") 
    pcd_points = pcd_points[np.random.choice(list(range(len(pcd_points))), size=10000)] # 随机采样10000个点
    # pcd_points, downsampled_labels, downsampled_normals, downsampled_curvatures = downsample_with_labels(pcd_data)

    # 2. 读取STL文件并转换为点云
    try:
        mesh = o3d.io.read_triangle_mesh(stl_path)
    except:
        print(stl_path, "读取错误")
        return None
    target = mesh.sample_points_uniformly(number_of_points=10000)  # 将STL转换为点云
    stl_sampled = np.array(target.points, order="F")

    T, T_inv, distances, i = icp(pcd_points, stl_sampled, max_iterations=1000)
    print(np.mean(distances))

    trans_stp(stp_path, T_inv, save_file_path)


def main():
    '''
    stp_path = r"/home/dataset/cloud/split"
    stl_path = r"/home/dataset/cloud/stl"
    pcd_path = r"/home/dataset/cloud/npy"

    stp_files = [file for file in os.listdir(stp_path) if file.lower().endswith(".stp")]
    for stp_file in stp_files:
        stp_file_path = os.path.join(stp_path, stp_file)
        stl_file_name = pcd_file_dir = stp_file.split('.')[0]
        stl_file_path = os.path.join(stl_path, stl_file_name + ".stl")
        pcd_file_path = os.path.join(pcd_path, pcd_file_dir, "coord_sampled.npy")
        
        peizhun(stp_file_path, stl_file_path, pcd_file_path)
    '''
    
    '''
    pcd_file_path = r"/home/chenjiayi/peizhun/coord_pre_downsample.npy"
    stl_file_path = r"/home/chenjiayi/peizhun/00196.stl"
    stp_file_path = r"/home/chenjiayi/peizhun/00196.stp"
    peizhun(stp_file_path, stl_file_path, pcd_file_path)
    '''
    path = r"G:\Data\point_cloud"
    dirs = os.listdir(path)
    for dir in dirs:
        if "." in dir:
            continue
        
        stp_file_path = os.path.join(path, dir, '3d_model.step')
        stl_file_path = os.path.join(path, dir, '3d_model.stl') 
        pcd_file_path = os.path.join(path, dir, 'pre_downsample.txt')
        save_file_path = os.path.join(path, dir, '3d_model_reg.step')
        reg(stp_file_path, stl_file_path, pcd_file_path, save_file_path)

        """
        if os.path.exists(os.path.join(path, dir, "new_3d_model.step")):
            os.remove(os.path.join(path, dir, "new_3d_model.step"))

            print(dir, "done.")

        """
        break
    

if __name__ == "__main__":
    main()