import rpyc
import os
import sys
from hexbytes import HexBytes
experimentFolder = os.environ["EXPERIMENTFOLDER"]
sys.path.insert(1, experimentFolder)

from aux import identifersExtract


def init_web3(robotID):

    # Get ID from argos
    robotID = int(robotID)

    # Connect to the RPYC which hosts web3.py (port 4xxx where xxx is robot ID)
    dockerIP = identifersExtract(robotID, 'IP')
    
    conn = rpyc.connect(dockerIP, 4000, config = {"allow_all_attrs" : True})
    w3 = conn.root

    return w3


if __name__=='__main__':

    w3 = init_web3(robotID = 3)