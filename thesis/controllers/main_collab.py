

#!/usr/bin/env python3
# This is the main control loop running in each argos robot

# /* Import Packages */
#######################################################################
import random, math, copy
import time, sys, os
import logging

experimentFolder = os.environ['EXPERIMENTFOLDER']
sys.path += [os.environ['EXPERIMENTFOLDER']+'/controllers', \
             os.environ['EXPERIMENTFOLDER']+'/loop_functions', \
             os.environ['EXPERIMENTFOLDER']]


from movement import RandomWalk
from control_params import params as cp
from data_structure import Trajectory, RecordedData
from erandb import ERANDB
from aux import Logger, Peer, identifersExtract
from control_params import params as cp
from console import init_web3

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

# /* Logging Levels for Console and File */
#######################################################################
loglevel = 10
logtofile = False 
recorded_data = None

def rotateZ(angle, waypoint):
    fSin = math.sin(angle)
    fCos = math.cos(angle)
    x, y = waypoint
    fNewX = x * fCos - y * fSin
    fNewY = x * fSin + y * fCos
    return [fNewX, fNewY]

def compute_position(rRange, rBearing, orientation, position):
    angle = rBearing + orientation
    x = rRange*math.cos(angle) + position[0]
    y = rRange*math.sin(angle) + position[1]
    return [x, y]


def init():
    global rw, clock, robotID, recorded_data, w3, me
    robotID = str(int(robot.variables.get_id()[2:])+1)
    robotIP = identifersExtract(robotID, 'IP')
    # /* Initialize Console Logging*/
    #######################################################################
    log_folder = f'{experimentFolder}/logs/{robotID}/'

    # Monitor logs (recorded to file)
    name =  'monitor.log'
    os.makedirs(os.path.dirname(log_folder+name), exist_ok=True)
    logging.basicConfig(filename=log_folder + name, filemode='w+', format=f'[{robotID} %(levelname)s %(name)s %(relativeCreated)d] %(message)s')
    robot.log = logging.getLogger('main')
    robot.log.setLevel(loglevel)
    
    if True:
        # /* Init web3.py */
        robot.log.info('Initialising Python Geth Console...')
        w3 = init_web3(robotID)

        # /* Init an instance of peer for this Pi-Puck */
        robot.log.info('Initialising peer instance for e-puck')
        me = Peer(robotID, robotIP, w3.enode, w3.key)

    # /* Init the random walk methods of the robot
    robot.log.info("robot initialised")
    rw = RandomWalk(robot, cp['scout_speed'])

    header = ['robotID', 'neighborID', 'time', 'x', 'y']
    file_name = f'results/data/e{robotID}_collected_data.csv'
    erb = ERANDB(robot, cp['erbDist'], cp['erbtFreq'])
    logger = Logger(file_name, header)
    recorded_data = RecordedData(erb, logger)
    clock = 0




def controlstep():
    global clock, robotID, recorded_data
    rw.step()

    readings = robot.epuck_range_and_bearing.get_readings()
    for i in range(len(readings)):
        neighborID = readings[i][0][0]
        # If potential trajectory started 
        if neighborID in recorded_data.get_potential_data() and recorded_data.get_potential_data()[neighborID] is not None:
            itTrajectory = recorded_data.get_potential_data()[neighborID]
            # If discontinuous, clear potential trajectory
            if itTrajectory.get_Prev_Time() != clock - 1:
                recorded_data.set_potential_data(None, neighborID)
                continue
            elif (itTrajectory.get_Prev_Time() - itTrajectory.get_Start_Time() == SEQ_LENGTH - 1):
                recorded_data.add_saved_data(itTrajectory)
                recorded_data.set_potential_data(None, neighborID)
            else:
                position = robot.position.get_position()[:2] # the robot position
                orientation = robot.position.get_orientation() # the robot orientation in the world axis 
                rRange, rVerticalBearing = readings[i][1:] # range and bearing with the neighboring robot
                waypoint = compute_position(rRange, rVerticalBearing, orientation, position)
                itTrajectory.add_waypoint(waypoint)
                itTrajectory.update_Prev_Time(clock)
        # Else, start a potential trajectory
        else:
            tracked_robot = neighborID
            start_time = clock
            prev_time = clock

            position = robot.position.get_position()[:2]
            orientation = robot.position.get_orientation()
            rRange, rVerticalBearing = readings[i][1:]
            waypoints = compute_position(rRange, rVerticalBearing, orientation, position)

            itTrajectory = Trajectory(robotID, tracked_robot, start_time, prev_time, waypoints)
            recorded_data.add_potential_data(itTrajectory)

    if len(recorded_data.get_saved_data()) > 10:
        recorded_data.get_Logger().begin()
        for trajectory in recorded_data.get_saved_data():
            neighborID = trajectory.get_Tracked_Robot()
            time = trajectory.get_Start_Time()
            for waypoint in trajectory.get_waypoints():
                line = f'{robotID},{neighborID},{time},{waypoint[0]},{waypoint[1]}\n'
                recorded_data.send_data(line)
                time += 1
            robot.log.info(f'Recorded data of {neighborID} saved.')
        recorded_data.set_saved_data([])
        
    clock += 1


def reset():
    pass

def destroy():
    pass




