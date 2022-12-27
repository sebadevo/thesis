from erandb import ERANDB
from aux import Logger

class Trajectory(object):
    def __init__(self, robot_id, tracked_robot, start_time, prev_time, waypoints) -> None:
        self.robot_id = robot_id
        self.tracked_robot = tracked_robot
        self.start_time = start_time
        self.prev_time = prev_time
        self.waypoints = [waypoints]

    def add_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def update_Prev_Time(self, current_clock):
        self.prev_time = current_clock

    def get_Start_Time(self):
        return self.start_time

    def get_Prev_Time(self):
        return self.prev_time

    def get_Tracked_Robot(self):
        return self.tracked_robot

    def get_waypoints(self):
        return self.waypoints


class RecordedData(object):
    def __init__(self, erb: ERANDB, logger: Logger) -> None:
        self.erb = erb
        self.logger = logger
        self.potential_data = {}#dict.fromkeys([i + 1 for i in range(len(allrobots))])
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