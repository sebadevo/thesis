# import pandas as pd

import random, math
import time, sys, os
import logging

sys.path += [os.environ['EXPERIMENTFOLDER'] + '/controllers', \
             os.environ['EXPERIMENTFOLDER'] + '/loop_functions', \
             os.environ['EXPERIMENTFOLDER']]

from loop_params import params as cp
from erandb import ERANDB
from aux import Logger

"""
To reduce the number of waypoints stored in memory,
consider two robot positions distinct if they are
at least MIN_DISTANCE away from each other
This constant is expressed in meters
"""
MIN_DISTANCE = 0.05
"Convenience constant to avoid calculating the square root in PostStep()"
MIN_DISTANCE_SQUARED = MIN_DISTANCE ** 2
"Length of the sequences (in number of time steps)"
SEQ_LENGTH = 100


class Trajectory(object):
    def __init__(self, robot_id, tracked_robot, start_time, prev_time, start_position, start_orientation,
                 waypoints) -> None:
        self.robot_id = robot_id
        self.tracked_robot = tracked_robot
        self.start_time = start_time
        self.prev_time = prev_time
        self.start_position = start_position
        self.start_orientation = start_orientation
        self.waypoints = [waypoints]

    def add_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def update_Prev_Time(self, current_clock):
        self.prev_time = current_clock

    def get_Start_Time(self):
        return self.start_time

    def get_Prev_Time(self):
        return self.prev_time

    def get_Start_Position(self):
        return self.start_position

    def get_Start_Orientation(self):
        return self.start_orientation

    def get_Tracked_Robot(self):
        return self.tracked_robot

    def get_waypoints(self):
        return self.waypoints


class RecordedData(object):
    def __init__(self, erb: ERANDB, logger: Logger) -> None:
        self.erb = erb
        self.logger = logger
        self.potential_data = dict.fromkeys([i + 1 for i in range(len(allrobots))])
        self.saved_data = []

    def add_saved_data(self, trajectory: Trajectory):
        self.saved_data.append(trajectory)

    def add_potential_data(self, trajectory: Trajectory):
        self.potential_data[trajectory.tracked_robot] = trajectory

    def send_data(self, line):
        self.logger.log(line)

    def get_erb(self):
        return self.erb

    def get_Logger(self):
        return self.logger

    def get_saved_data(self):
        return self.saved_data

    def get_potential_data(self):
        return self.potential_data

    def set_potential_data(self, value, nID):
        self.potential_data[nID] = value

    def set_saved_data(self, value):
        self.saved_data = value


def convert_to_euler(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(+1.0, t2))
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def rotateZ(angle, waypoint):
    fSin = math.sin(angle)
    fCos = math.cos(angle)
    x, y = waypoint
    fNewX = x * fCos - y * fSin
    fNewY = x * fSin + y * fCos
    return [fNewX, fNewY]


# clock = 0
# AllData = {}


def init():
    pass
    # print("bonjour je suis dans init de loop")

    # global clock

    # for robot in allrobots:
    #     robotID = str(int(robot.variables.get_id()[2:]) + 1)
    #     header = ['robotID', 'neighborID', 'time', 'pos_x', 'pos_y']
    #     file_name = f'results/data/e{robotID}_collected_data.csv'
    #     erb = ERANDB(robot, cp['erbDist'], cp['erbtFreq'])
    #     logger = Logger(file_name, header)
    #     AllData[robotID] = RecordedData(erb, logger)


def pre_step():
    pass


def post_step():
    pass
    # print("bonjour je suis dans le post_step")
    # global clock, AllTrajectories

    # for robot in allrobots:
    #     robotID = str(int(robot.variables.get_id()[2:]) + 1)
    #     readings = robot.epuck_range_and_bearing.get_readings()
    #     for i in range(len(readings)):
    #         neighborID = readings[i][0][0]

    #         # If potential trajectory started 
    #         if itTrajectory := AllData[robotID].get_potential_data()[neighborID]:
    #             # If discontinuous, clear potential trajectory
    #             if itTrajectory.get_Prev_Time() != clock - 1:
    #                 AllData[robotID].set_potential_data(None, neighborID)
    #                 continue
    #             elif (itTrajectory.get_Prev_Time() - itTrajectory.get_Start_Time() == SEQ_LENGTH - 1):
    #                 AllData[robotID].add_saved_data(itTrajectory)
    #                 AllData[robotID].set_potential_data(None, neighborID)
    #             else:
    #                 # get the orientation (the angle along the Z axis (aligned with the world axis))
    #                 rZangle = robot.position.get_orientation()  # add the code in the py_actusensor_wrapper_epuck.cpp
    #                 nZangle = itTrajectory.get_Start_Orientation()

    #                 # compute the offset between current and start frame
    #                 current_position = robot.position.get_position()[:2]
    #                 # Compute the offset between current and start frame (not used...)
    #                 # ]]]]]]]]]pos_displacement = current_position - itTrajectory.get_Start_Position()
    #                 # Get vector to point of interest in current frame
    #                 rRange, rVerticalBearing = readings[i][1:]
    #                 waypoint = [rRange * 0.01, math.pi / 2 - rVerticalBearing]
    #                 # Get vector to point of interest in start frame
    #                 waypoint = rotateZ(rZangle, waypoint) + current_position
    #                 subtracted = [element1 - element2 for (element1, element2) in zip(waypoint, itTrajectory.get_Start_Position())]
    #                 cPO = rotateZ(-nZangle, subtracted)
    #                 itTrajectory.add_waypoint(cPO)
    #                 itTrajectory.update_Prev_Time(clock)
    #         # Else, start a potential trajectory
    #         else:
    #             tracked_robot = neighborID
    #             start_time = clock
    #             prev_time = clock
    #             start_position = robot.position.get_position()[:2]
    #             start_orientation = robot.position.get_orientation()  # don't forget to check how to add the code in the epuck_wrapper
    #             # robot_erb = AllData[robotID].get_erb()
    #             rRange, rVerticalBearing = readings[i][1:]
    #             waypoints = [rRange * 0.01, math.pi / 2 - rVerticalBearing]
    #             itTrajectory = Trajectory(robotID, tracked_robot, start_time, prev_time, start_position, start_orientation, waypoints)
    #             AllData[robotID].add_potential_data(itTrajectory)

    #     if len(AllData[robotID].get_saved_data()) > 10:
    #         AllData[robotID].get_Logger().begin()
    #         for trajectory in AllData[robotID].get_saved_data():
    #             count_time = 0
    #             for waypoint in trajectory.get_waypoints():
    #                 neighborID = trajectory.get_Tracked_Robot()
    #                 time = trajectory.get_Start_Time() + count_time
    #                 line = f'{robotID},{neighborID},{time},{waypoint[0]},{waypoint[1]}\n'
    #                 AllData[robotID].send_data(line)
    #                 count_time += 1
    #         AllData[robotID].set_saved_data([])

    # clock += 1
    # print(clock)


def is_experiment_finished():
    pass


def reset():
    pass
    # init()


def destroy():
    pass


def post_experiment():
    print("Finished from Python!")
