import numpy as np
from sklearn.neighbors import NearestNeighbors
from plotter import plot
from scipy.linalg import logm, expm

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB) #correlation matrix
    U, S, Vt = np.linalg.svd(H) #singular value decomposition
    R = np.dot(Vt.T, U.T) #rotation matrix

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T) #translation vector

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()

def icp(A, B, init_pose=None, max_iterations=100, tolerance=0.01):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points ----> [(x1, y1), (x2, y2), ... ]
        B: Nxm numpy array of destination mD point ----> [(x1, y1), (x2, y2), ... ]
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    if (max_iterations <= 0):
        return init_pose, 0,0

    # get number of dimensions
    m = A.shape[1]

    # Check if A or B is empty
    if A.size == 0 or B.size == 0:
        print("Input arrays A and B must not be empty.")
        return np.eye(m+1), [], 0

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)
    
    return T, distances, i

'''
def main():

    data1 = {'f': [27, 26, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 27, 28, 28, 28, 29, 28, 28, 28, 28, 28, 28, 29, 29, 28, 29, 29, 30, 30, 46, 46, 45, 45, 46, 42, 42, 46, 46, 45, 45, 45, 45, 45, 45, 44, 45, 45, 45, 45, 46, 46, 46, 45, 44, 42, 41, 41, 41, 41, 41, 40, 41, 40, 41, 40, 40, 39, 39, 39, 39, 40, 39, 39, 39, 38, 38, 39, 39, 38, 38, 38, 38, 38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 38, 38, 38, 38, 38], 'b': [37, 37, 37, 36, 37, 37, 37, 37, 37, 37, 36, 37, 36, 36, 36, 37, 36, 36, 38, 37, 36, 40, 37, 40, 40, 40, 40, 41, 38, 54, 37, 53, 38, 39, 39, 39, 36, 41, 41, 39, 37, 38, 38, 38, 38, 38, 39, 39, 40, 40, 41, 41, 41, 41, 41, 43, 29, 0, 25, 29, 29, 29, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}
    data2 = {'f': [26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 27, 28, 28, 28, 28, 28, 29, 29, 28, 28, 29, 29, 29, 29, 29, 29, 31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 38, 38, 38, 38, 38, 39, 39, 39, 38, 38, 38, 38, 38, 38, 38], 'b': [37, 37, 37, 37, 37, 37, 37, 36, 37, 37, 37, 37, 38, 36, 37, 37, 36, 37, 37, 36, 36, 36, 36, 37, 36, 37, 37, 37, 39, 37, 40, 40, 40, 40, 40, 55, 54, 54, 38, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}
    data1 = plot(data1, 40, 140, 220, 320, 101)
    data2 = plot(data2, 40, 140, 220, 320, 101)

    #to numpy array
    A = np.array(list(data1))
    B = np.array(list(data2))

    # Perform ICP
    T_final, distances, iterations = icp(A, B)
    print(T_final)
    # Plot the results
    import matplotlib.pyplot as plt

    plt.scatter(A[:, 0], A[:, 1], c='r', label='Source Points')
    plt.scatter(B[:, 0], B[:, 1], c='b', label='Destination Points')
    A = np.dot(T_final, np.concatenate((A.T, np.ones((1, A.shape[0]))), axis=0)).T[:, :2]
    
    plt.scatter(A[:, 0], A[:, 1], c='r', marker='x', label='Transformed Source Points')
    #plot origin point orange
    plt.scatter(0, 0, c='orange', label='Origin')
    plt.legend()
    plt.title('Iterative Closest Point')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

if __name__ == "__main__":
    main()'''