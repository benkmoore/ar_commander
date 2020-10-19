import numpy as np
from numba import jit
from numba import njit
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from queue import Queue
import heapq
import time

@jit(nopython = True)
def numba_norm(x, axis = 0):
    # replacement for np.linalg.norm
    s = (x.conj() * x).real
    return np.sqrt(np.sum(s, axis = axis))

@jit(nopython = True)
def brute_force_search(f, point_cluster):
    theta_range = np.linspace(0,np.pi/2, 50)
    Q = []
    for theta in theta_range:
        cost = variance_criterion_wrapper(theta, point_cluster)
        heapq.heappush(Q, (cost, theta))
    optimal_cost, theta_opt = heapq.heappop(Q)
    return theta_opt

def _segment_points(point_arr, radius = 3):
    point_KDtree = cKDTree(point_arr)
    Q = Queue()
    checked_points = set()
    S = []
    for point in point_arr:
        C = set()
        Q.put(tuple(point))
        #then we find the points which lie within r distance to any of the points already in the cluster,
        #and again put the newly found points in this cluster;
        #we repeat this process until the cluster does not grow any more,
        # and this finalized cluster serves as one segmentation in the output

        while not Q.empty():
            cur_point = Q.get()
    
            if cur_point not in checked_points:               
                # find all neighbors
                neighbor_points = point_arr[point_KDtree.query_ball_point(cur_point, radius)]
                
                # add all neighbors to cluster and queue to check
                for neighbor in neighbor_points:
                    C.add(tuple(neighbor))
                    Q.put(tuple(neighbor))
                    
                # mark current point as checked
                checked_points.add(cur_point)
            
        # require at least 4 points
        if len(C) > 4:
            S.append(np.array(list(C)))
    return S

@jit(nopython = True)
def _variance_criterion(C1, C2):
    c1_max = np.max(C1)
    c2_max = np.max(C2)
    c1_min = np.min(C1)
    c2_min = np.min(C2)

    v1 = np.hstack((c1_max - C1, C1 - c1_min))
    v2 = np.hstack((c2_max - C2, C2 - c2_min))

    D1 = v1[:,np.argmin(numba_norm(v1))]
    D2 = v2[:, np.argmin(numba_norm(v2))]

    E1 = D1 < D2
    E2 = D2 < D1
    gamma = np.var(np.concatenate((D1[E1],D2[~E1]))) + np.var(np.concatenate((D2[E2],D1[~E2])))
    return gamma

@jit(nopython = True)
def variance_criterion_wrapper(theta, point_cluster):
    e1 = np.array([[np.cos(theta)],[np.sin(theta)]])
    e2 = np.array([[-np.sin(theta)],[np.cos(theta)]])
    C1 = point_cluster @ e1
    C2 = point_cluster @ e2
    cost = _variance_criterion(C1, C2)
    return cost

@jit(nopython = True)
def fibonacci_search(f, point_cluster, a, b, n, ):
    ε = 0.0001
    golden_ratio = (1 + np.sqrt(5))/2 
    s = (1-np.sqrt(5))/(1+np.sqrt(5))
    rho= 1 / (golden_ratio*(1-s**(n+1))/(1-s**n))
    d = rho*b + (1-rho)*a
    yd = f(d, point_cluster)
    for i in range(1, n):
        if i == n: # one evaluation left
            # put evaluation points next to each other
            c = ε*a + (1-ε)*d 
        # pick new evaluation point based on ratio of fibonnaci sequence
        else: 
            c = rho*a + (1-rho)*b

        yc = f(c, point_cluster)
        if yc < yd:
            b, d, yd = d, c, yc
        else:
            a, b = b, c

        rho= 1 / (golden_ratio*(1-s**(n-i+1))/(1-s**(n-i)))
    return  (a, b) if a < b else (b, a)

@jit(nopython = True)
def _find_corner(a, b, c):
    cvec = c.reshape(2, 1)
    analytical_inverse = (1 / (a[0] * b[1] - a[1] * b[0])) * np.array([[b[1], -b[0]], [-a[1], a[0]]])
    corner = analytical_inverse @ cvec
    return corner

class RectangleDetector:
    def __init__(self, point_arr: np.ndarray):
        self.point_arr = point_arr

        # list of corner arrays
        self.rectangle_list = None

        # search radius for segmentation
        self.radius = 3

    def update_points(self, point_arr):
        self.point_arr = point_arr
        self.rectangle_list = None

    def find_rectangles(self):
        cluster_list = _segment_points(point_arr, radius = 3)
        rectangle_list = []
        for cluster in cluster_list:
            corners, optimal_cost = self._fit_rectangle(cluster)
            # TODO: filter based on cost
            # could filter based on absolute distance from line?
            # distance shouldnt be large
            # filter based on size of detection
            rectangle_list.append(corners)
        self.rectangle_list = rectangle_list
        return self.rectangle_list

    def _fit_rectangle(self, point_cluster):
        theta_opt, _ = fibonacci_search(variance_criterion_wrapper,point_cluster, 0, np.pi/2, 10)
        e1_opt = np.array([[np.cos(theta_opt)],[np.sin(theta_opt)]])
        e2_opt  = np.array([[-np.sin(theta_opt)],[np.cos(theta_opt)]])
        C1_opt = point_cluster @ e1_opt
        C2_opt = point_cluster @ e2_opt

        a = np.array([np.cos(theta_opt), -np.sin(theta_opt), np.cos(theta_opt), -np.sin(theta_opt)])
        b = np.array([np.sin(theta_opt), np.cos(theta_opt), np.sin(theta_opt), np.cos(theta_opt)])
        c = np.array([np.min(C1_opt), np.min(C2_opt), np.max(C1_opt), np.max(C2_opt)])
        
        corners = self.get_rect_corners(a, b, c)
        optimal_cost = np.nan
        return corners, optimal_cost
        
    def get_rect_corners(self, a, b, c):
        point_1 = _find_corner(a[0:2], b[0:2], c[0:2])
        point_2 = _find_corner(a[1:3], b[1:3], c[1:3])
        point_3 = _find_corner(a[2:4], b[2:4], c[2:4])
        point_4 = _find_corner(np.array([a[3], a[0]]), np.array([b[3], b[0]]), np.array([c[3], c[0]]))
        return np.vstack((point_1.T, point_2.T, point_3.T, point_4.T))

    def plot(self):
        for corners in corner_list:
            # plot edges
            plot_points = np.vstack((corners, corners[0,:]))
            plt.plot(plot_points[:,0], plot_points[:,1], "-r")

        # plat raw data points
        plt.plot(self.point_arr[:,0], self.point_arr[:,1],'o')

if __name__ == "__main__":
    point_arr = np.array([
        [-5.1,0],
        [-5.5,0],
        [0, -5],
        [2, 11.5],
        [2, 12.5],
        [2.1, 13.5],
        [2.4, 15.6],
        [2.1, 16.7],
        [3.2, 16.7],
        [4.1, 16.8],
        [5.2, 17.1],
        [7.1, 17.0],
        [8, 16.9],
        [ -6.5, 7. ],
        [ -7.5, 7. ],
        [ -8.5, 7.1],
        [-10.6, 7.4],
        [-11.7, 7.1],
        [-11.7, 8.2],
        [-11.8, 9.1],
        [-12.11, 10.2],
        [-12., 12.1],
        [-11.9, 13. ],
        [-5, 0],
        [-3, 2],
        [-3, 1],
        [-4, -2],
        [-18, 20],
        [-21.71751442, -5.45405845],
        [-22.4246212 , -4.74695167],
        [-23.06101731, -3.96913421],
        [-24.33380951, -2.27207794],
        [-25.32375901, -1.70639251],
        [-24.54594155, -0.92857505],
        [-23.98025612, -0.22146827],
        [-23.4145707 , 0.76848122],
        [-22.00035713, 2.04127343],
        [-21.29325035, 2.60695885]
    ])

    detector = RectangleDetector(point_arr)
    corner_list = detector.find_rectangles()
    for i in range(8):
        start_time = time.time()
        detector.update_points(point_arr)
        corner_list = detector.find_rectangles()
        total_time =time.time() - start_time
        print(f"fit {point_arr.shape[0]} points in {total_time} seconds")

    detector.plot()
    plt.show()