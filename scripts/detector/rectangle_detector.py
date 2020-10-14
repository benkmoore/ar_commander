import numpy as np
from numba import jit
from scipy.spatial import KDTree
from queue import Queue
import heapq


class RectangleDetector:
    def __init__(self, point_arr: np.ndarray):
        self.point_arr = point_arr

        # list of corner arrays
        self.rectangle_list = []

        # search radius for segmentation
        self.radius = 3

    @staticmethod
    @jit(nopython = True)
    def _find_corner(a, b, c):
        cvec = c.reshape(2, 1)
        analytical_inverse = (1 / (a[0] * b[1] - a[1] * b[0])) * np.array([[b[1], -b[0]], [-a[1], a[0]]])
        corner = analytical_inverse @ cvec
        return corner

    def find_rectangles(self):
        cluster_list = self._segment_points()

        for cluster in cluster_list:
            corners = self.fit_rectangle(cluster)
            self.rectangle_list.append(corners)

        return self.rectangle_list


    def _segment_points(self):
        point_KDtree = KDTree(self.point_arr)
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
                    # search radius
                    self.radius = 3

                    # find all neighbors
                    neighbor_points = point_arr[point_KDtree.query_ball_point(cur_point, self.radius)]

                    # add all neighbors to cluster and queue to check
                    for neighbor in neighbor_points:
                        C.add(tuple(neighbor))
                        Q.put(tuple(neighbor))

                    # mark current point as checked
                    checked_points.add(cur_point)

            if len(C) > 4:
                S.append(np.array(list(C)))

        return S

    @staticmethod
    def _variance_criterion(C1, C2):
        c1_max = np.max(C1)
        c2_max = np.max(C2)
        c1_min = np.min(C1)
        c2_min = np.min(C2)

        v1 = np.hstack((c1_max - C1, C1 - c1_min))
        v2 = np.hstack((c2_max - C2, C2 - c2_min))
        D1 = v1[:,np.argmin(np.linalg.norm(v1,axis =0))]
        D2 = v2[:, np.argmin(np.linalg.norm(v2,axis =0))]

        E1 = D1 < D2
        E2 = D2 < D1

        gamma = np.var(np.concatenate((D1[E1],D2[~E1]))) + np.var(np.concatenate((D2[E2],D1[~E2])))
        return gamma

    def fit_rectangle(self, point_cluster):
        theta_range = np.linspace(0,np.pi/2, 50)
        Q = []
        for theta in theta_range:
            e1 = np.array([[np.cos(theta)],[np.sin(theta)]])
            e2 = np.array([[-np.sin(theta)],[np.cos(theta)]])
            C1 = point_cluster @ e1
            C2 = point_cluster @ e2
            cost = self._variance_criterion(C1, C2)
            heapq.heappush(Q, (cost, theta))

        theta_opt = heapq.heappop(Q)[1]
        e1_opt = np.array([[np.cos(theta_opt)],[np.sin(theta_opt)]])
        e2_opt  = np.array([[-np.sin(theta_opt)],[np.cos(theta_opt)]])
        C1_opt = point_cluster @ e1_opt
        C2_opt = point_cluster @ e2_opt
        a = np.array([np.cos(theta_opt), -np.sin(theta_opt), np.cos(theta_opt), -np.sin(theta_opt)])
        b = np.array([np.sin(theta_opt), np.cos(theta_opt), np.sin(theta_opt), np.cos(theta_opt)])
        c = np.array([np.min(C1_opt), np.min(C2_opt), np.max(C1_opt), np.max(C2_opt)])

        corners = self.get_rect_corners(a, b, c)
        return corners


    # @staticmethod
    # @jit(nopython = True)
    def get_rect_corners(self, a, b, c):
        point_1 = self._find_corner(a[0:2], b[0:2], c[0:2])
        point_2 = self._find_corner(a[1:3], b[1:3], c[1:3])
        point_3 = self._find_corner(a[2:4], b[2:4], c[2:4])
        point_4 = self._find_corner(np.array([a[3], a[0]]), np.array([b[3], b[0]]), np.array([c[3], c[0]]))
        return np.vstack((point_1.T, point_2.T, point_3.T, point_4.T))

    def plot(self):
        for corners in corner_list:
            # plot edges
            plot_points = np.vstack((corners, corners[0,:]))
            plt.plot(plot_points[:,0], plot_points[:,1], "-r")

        # plat raw data points
        plt.plot(self.point_arr[:,0], self.point_arr[:,1],'o')

