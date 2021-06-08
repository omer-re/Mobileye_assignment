import car
import path
import numpy as np
import math
import matplotlib.pyplot as plt



# Simulation parameters
global k
global Lfc
global kP
global dt
k= 0.1 # look-ahead gain
Lfc= 1.0 # look ahead distance
kP = 1 # speed gain (proportional), for us it's const
dt = 0.002 # in secs = 200 milli sec

speed_kmh= 10.0
speed_msec= speed_kmh/3.6

T =100 # how many seconds will we simulate
iterations = T/dt # number of "frames"

# initialize car
car1=car.Car()
_path=path.path()

x,y,yaw,v=(car1.x,car1.y,car1.yaw,car1.v)
t=0.0
target_point=car.find_next_point_index(car1, _path.path_points[0][0],_path.path_points[0][1])
