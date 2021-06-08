import path
import numpy as np
import math
import matplotlib.pyplot as plt
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm


# car properties
wheel_base = 2.728 # L, [m]
length=4.841 #2010 model [m]
height=1.455 # [m]


class Car:
    wheel_base = 2.728 # L, [m]
    length=4.841 #2010 model [m]
    height=1.455 # [m]
    # car state vector
    x=0.0 # x coordinate
    y=0.0 # y coordinate
    yaw= 0.0 # "sivsuv"
    v = 0.0 # [m/sec] velocity is const
    points_done=0

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # car state vector
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# state is (x,y,yaw,v) vector
# returns the further point of the segment
def find_next_point_index(a_car, a_path, curr_x ,curr_y):
    path_arr=a_path.get_path_array[a_car.points_done:]

    min_dist=None
    next_wp=None
    next_wp_index=0
    # for each line segment
    for i in path_arr:
        point_a=a_path[i]
        point_b=a_path[i+1]

        # distance from car to line segment
        dist_from_path=distance_numpy(point_a, point_b, (curr_x,curr_y))
        if min_dist is None or dist_from_path<min_dist:
            min_dist=dist_from_path
            # steer towards it
            next_wp=a_path[i+1]
            next_wp_index=i+1

    return next_wp_index,next_wp

def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / wheel_base * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state



def pure_pursuit_control(a_car, a_path,state, curr_x, curr_y, pind):
    # the point we want to head to
    next_index,next_point=find_next_point_index(a_car, a_path, state, curr_x, curr_y)

    # alpha is the angle between the lines of:
    # rear axle through the center of the car pointing where the car is heading
    # and the line from the rear axle to the target point
    alpha= math.atan2(next_point[1] - state.y, next_point[0] - state.x) - state.yaw

    # forward velocity
    Lf= k*state.v +Lfc

    # delta is the steering angle of the wheels
    delta = math.atan2(2.0 * wheel_base * math.sin(alpha) / Lf, 1.0)

    return delta, next_point



# from: https://gist.github.com/nim65s/5e9902cd67f094ce65b0
def distance_numpy(A, B, P):
    """ segment line AB, point P, where each one is an array([x, y]) """
    if all(A == P) or all(B == P):
        return 0
    if arccos(dot((P - A) / norm(P - A), (B - A) / norm(B - A))) > pi / 2:
        return norm(P - A)
    if arccos(dot((P - B) / norm(P - B), (A - B) / norm(A - B))) > pi / 2:
        return norm(P - B)
    return norm(cross(A-B, A-P))/norm(B-A)
