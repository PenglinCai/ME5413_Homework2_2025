import numpy as np
import open3d as o3d
import datetime

def icp_core(point_cloud1, point_cloud2):   
    """
    Solve transformation from point_cloud2 to point_cloud1, T1_2
    :param point_cloud1: numpy array, size = n x 3, n is num of point
    :param point_cloud2: numpy array, size = n x 3, n is num of point
    :return: transformation matrix T, size = 4x4
    
    Note: point cloud should be in same size. Point with same index are corresponding points.
          For example, point_cloud1[i] and point_cloud2[i] are a pair of cooresponding points.
    
    """
    assert point_cloud1.shape == point_cloud2.shape, 'point cloud size not match'
    
    T1_2 = np.eye(4)
    # TODO: Finish icp based on SVD, you can refer the lecture slides. Please leave comments and explainations for each step.
    #  Compute the centroids of both point clouds
    centroid1 = np.mean(point_cloud1, axis=0)  #  centroid of point_cloud1
    centroid2 = np.mean(point_cloud2, axis=0)  #  centroid of point_cloud2
    
    #  Subtract the centroids to obtain centered point sets
    P1 = point_cloud1 - centroid1  #  centered points for point_cloud1
    P2 = point_cloud2 - centroid2  #  centered points for point_cloud2
    
    #  Compute the cross-covariance matrix between the two point sets
    H = np.dot(P2.T, P1)  #  cross-covariance matrix
    
    #  Perform Singular Value Decomposition (SVD) on H
    U, S, Vt = np.linalg.svd(H)  #  SVD decomposition of H
    
    #  Compute rotation matrix R from SVD components
    R = np.dot(Vt.T, U.T)  #  initial rotation matrix
    
    #  Ensure a proper rotation (determinant should be 1, not -1)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1  #  fix reflection case by flipping sign of the last row of Vt
        R = np.dot(Vt.T, U.T)  #  recompute R after adjustment
    
    #  Compute translation vector t that aligns the centroids after rotation
    t = centroid1 - np.dot(R, centroid2)  #  translation vector
    
    #  Construct the homogeneous transformation matrix T1_2
    T1_2[:3, :3] = R  #  set rotation part
    T1_2[:3, 3] = t   #  set translation part
    
    return T1_2


def solve_icp_with_known_correspondence(point_cloud1, point_cloud2):
    # Solve for transformation matrix
    T1_2 = icp_core(point_cloud1, point_cloud2)
    print('------------ transformation matrix T1_2 ------------')
    print(T1_2)

    # TODO: calculate transformed point_cloud2 based on T1_2 solved above
    #  Transform point_cloud2 using the homogeneous transformation T1_2
    ones = np.ones((point_cloud2.shape[0], 1))  #  add a column of ones for homogeneous coordinates
    points_hom = np.hstack((point_cloud2, ones))  #  convert to homogeneous coordinates
    points_transformed = (T1_2 @ points_hom.T).T  #  apply the transformation matrix
    point_cloud2_transformed = points_transformed[:, :3]  #  extract the 3D coordinates
    
    # Visualization
    mean_distance = mean_dist(point_cloud2_transformed, point_cloud1)
    print('mean_error= ' + str(mean_distance))

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(point_cloud2)
    pcd2_transformed = o3d.geometry.PointCloud()
    pcd2_transformed.points = o3d.utility.Vector3dVector(point_cloud2_transformed)
    
    pcd1.paint_uniform_color([1, 0, 0])  # Red for reference cloud
    pcd2.paint_uniform_color([0, 1, 0])  # Green for original cloud
    pcd2_transformed.paint_uniform_color([0, 0, 1])  # Blue for transformed cloud
    
    o3d.visualization.draw_geometries([pcd1, pcd2, pcd2_transformed, axis_pcd])


def solve_icp_without_known_correspondence(point_cloud1, point_cloud2, n_iter, threshold):
    point_cloud2_temp = point_cloud2.copy()
    T_1_2accumulated = np.eye(4)

    # viz
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd1.paint_uniform_color([0, 0, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(axis_pcd)
    vis.add_geometry(pcd1)
    
    total_time_cost = 0

    for i in range(n_iter):
        start_time = datetime.datetime.now()
        
        # TODO: Try to estimate correspondence of points between 2 point clouds, 
        #       and reindex point_cloud2 based on your estimated correspondence
        # For each point in point_cloud1, find the nearest neighbor in point_cloud2_temp
        indices = []  # list to hold nearest neighbor indices
        for p in point_cloud1:
            dists = np.linalg.norm(point_cloud2_temp - p, axis=1)  # compute Euclidean distances
            idx = np.argmin(dists)  # index of the closest point
            indices.append(idx)
        point_cloud2_reordered = point_cloud2_temp[indices]  # reorder point_cloud2_temp according to estimated correspondence
            
        # Solve ICP for current iteration
        T1_2_cur = icp_core(point_cloud1, point_cloud2_reordered)
        
        end_time = datetime.datetime.now()
        time_difference = (end_time - start_time).total_seconds()
        total_time_cost += time_difference
        
        # TODO: Update accumulated transformation
        # Compose the current transformation with the accumulated transformation
        T_1_2accumulated = T1_2_cur @ T_1_2accumulated
        
        print('-----------------------------------------')
        print('iteration = ' + str(i+1))
        print('time cost = ' + str(time_difference) + 's')
        print('total time cost = ' + str(total_time_cost) + 's')        
        print('T1_2_cur = ')
        print(T1_2_cur)
        print('accumulated T = ')
        print(T_1_2accumulated)
        
        # TODO: Update point cloud2 using transform from current iteration
        # Update point_cloud2_temp by applying T1_2_cur transformation
        ones = np.ones((point_cloud2_temp.shape[0], 1))  # create homogeneous coordinates for point_cloud2_temp
        points_hom = np.hstack((point_cloud2_temp, ones))  # convert to homogeneous coordinates
        points_transformed = (T1_2_cur @ points_hom.T).T  # apply transformation T1_2_cur
        point_cloud2_temp = points_transformed[:, :3]  # update point_cloud2_temp with transformed points
        
        mean_distance = mean_dist(point_cloud1, point_cloud2_temp)
        print('mean_error= ' + str(mean_distance))

        # Update visualization
        pcd2_transed = o3d.geometry.PointCloud()
        pcd2_transed.points = o3d.utility.Vector3dVector(point_cloud2_temp)
        pcd2_transed.paint_uniform_color([1, 0, 0])
        vis.add_geometry(pcd2_transed)
        vis.poll_events()
        vis.update_renderer()
        vis.remove_geometry(pcd2_transed)

        if mean_distance < 0.00001 or mean_distance < threshold:
            print('------- fully converged! -------')
            break
        
        if i == n_iter - 1:
            print('------- reach iteration limit -------')

    print('time cost: ' + str(total_time_cost) + ' s')
    
    vis.destroy_window()
    
    # Final visualization
    pcd2_final = o3d.geometry.PointCloud()
    pcd2_final.points = o3d.utility.Vector3dVector(point_cloud2_temp)
    pcd2_final.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([axis_pcd, pcd1, pcd2_final])

def mean_dist(point_cloud1, point_cloud2):
    dis_array = []
    for i in range(point_cloud1.shape[0]):
        dif = point_cloud1[i] - point_cloud2[i]
        dis = np.linalg.norm(dif)
        dis_array.append(dis)
        
    return np.mean(np.array(dis_array))

def main():
    print('start hw program')
    pcd1 = o3d.io.read_point_cloud('student_data/student_data_3/bunny1.ply') # change to your file path
    pcd2 = o3d.io.read_point_cloud('student_data/student_data_3/bunny2.ply') # change to your file path
    points1 = np.array(pcd1.points)
    points2 = np.array(pcd2.points)

    # uncomment the lines following task 1 or 2 to run the corresponding task
    # task 1:
    # solve_icp_with_known_correspondence(points1, points2)
    # task 2:
    solve_icp_without_known_correspondence(points1, points2, n_iter=1000, threshold=0.015975649843764835)

if __name__ == '__main__':
    main()
