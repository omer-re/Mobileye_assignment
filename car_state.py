import path
import numpy as np
import math
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm

k = 0.1  # look-ahead gain
Lfc = 1.0  # look ahead distance
Kp = 1  # speed gain (proportional), for us it's const
dt = 0.002  # in secs = 200 milli sec

# car properties
wheel_base = 2.728  # L, [m]
length = 4.841  # 2010 model [m]
height = 1.455  # [m]


class Car:
    wheel_base = 2.728  # L, [m]
    length = 4.841  # 2010 model [m]
    height = 1.455  # [m]
    # car state vector
    x = 0.0  # x coordinate
    y = 0.0  # y coordinate
    yaw = 0.0  # "sivsuv"
    v = 0.0  # [m/sec] velocity is const
    points_done = 0

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # car state vector
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


# state is (x,y,yaw,v) vector
# returns the further point of the segment
def find_next_point(a_car, a_path, curr_x, curr_y, curr_ind):
    path_arr = a_path.get_path_array()[curr_ind:]
    min_dist = None
    next_wp = None
    next_wp_index = 0
    # for each line segment
    last_item = len(path_arr)
    for i in path_arr:

        # avoid getting out of boundaries
        if last_item == 1:
            next_wp_index = path_arr[-1]
            next_wp = a_path.get_path_array()[-1]
            return next_wp_index, next_wp

        point_a = path_arr[i - 1]
        point_b = path_arr[i]

        # distance from car to line segment
        dist_from_path = distance_numpy(point_a, point_b, (curr_x, curr_y))
        if min_dist is None or dist_from_path < min_dist:
            min_dist = dist_from_path

            # steer towards it
            next_wp = a_path.get_path_array()[i]
            next_wp_index = i
    last_item = last_item - 1

    return next_wp_index, next_wp


def update(_car, a, delta):
    _car.x = _car.x + _car.v * math.cos(_car.yaw) * dt
    _car.y = _car.y + _car.v * math.sin(_car.yaw) * dt
    _car.yaw = _car.yaw + _car.v / wheel_base * math.tan(delta) * dt
    _car.v = _car.v + a * dt

    return _car


def pure_pursuit_control(a_car, a_path, curr_x, curr_y, curr_ind):
    # the point we want to head to
    next_index, next_point = find_next_point(a_car, a_path, curr_x, curr_y, curr_ind)

    # alpha is the angle between the lines of:
    # rear axle through the center of the car pointing where the car is heading
    # and the line from the rear axle to the target point
    alpha = math.atan2(next_point[1] - a_car.y, next_point[0] - a_car.x) - a_car.yaw

    # forward velocity
    Lf = k * a_car.v + Lfc

    # delta is the steering angle of the wheels
    delta = math.atan2(2.0 * wheel_base * math.sin(alpha) / Lf, 1.0)

    return delta, next_point


# from: https://gist.github.com/nim65s/5e9902cd67f094ce65b0
def distance_numpy(A, B, P):
    """ segment line AB, point P, where each one is an array([x, y]) """
    if (A == P).all or (B == P).all:
        return 0
    if arccos(dot((P - A) / norm(P - A), (B - A) / norm(B - A))) > pi / 2:
        return norm(P - A)
    if arccos(dot((P - B) / norm(P - B), (A - B) / norm(A - B))) > pi / 2:
        return norm(P - B)
    return norm(cross(A - B, A - P)) / norm(B - A)
