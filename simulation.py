from car_state import Car
import car_state
import path
import numpy as np
import math
import matplotlib.pyplot as plt



# Simulation parameters
global k
global Lfc
global Kp
global dt
k= 0.1 # look-ahead gain
Lfc= 1.0 # look ahead distance
Kp = 1 # speed gain (proportional), for us it's const
dt = 0.002 # in secs = 200 milli sec

speed_kmh= 10.0
speed_msec= speed_kmh/3.6


# initialize car
car1=car_state.Car(0, 0, 0, 0)
_path=path.path()

frames= len(_path.path_points)

x,y,yaw,v=(car1.x,car1.y,car1.yaw,car1.v)
t=0.0
target_point=car_state.find_next_point(car1,_path, _path.path_points[0][0], _path.path_points[0][1], 0)

for counter,t_seg in enumerate(frames):
    ai = Kp * (speed_msec - car1.v)  # PID controller
    delta_i, target_point= car1.pure_pursuit_control(car1, _path, car1.x, car1.y, counter)
    car1 = car_state.update(car1, ai, delta_i)

    plt.cla()
    plt.plot(car1.x, car1.y, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.plot(target_point[0], target_point[1], "xg", label="target")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Speed[km/h]:" + str(car1.v * 3.6)[:4])
    plt.pause(0.001)
    plt.show()
