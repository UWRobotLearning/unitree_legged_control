#!/usr/bin/env python3
import rospy
import rosbag
import rospkg
import os
import pandas as pd


package_path = rospkg.RosPack().get_path('unitree_legged_control')

bagpath = package_path + "/launch/bagfiles/"
bags = os.listdir(bagpath)

csvpath = package_path + "/csvs"
csvs = os.listdir(csvpath)

def bag2csv(bag):
    
    bagName = bag[ : -len(".bag")]

    bagfile = rosbag.Bag(os.path.join(bagpath, bag))

    data = [] 

    first_t = -1

    time_secs = 0.0

    for topic, msg, time in  bagfile.read_messages():

        if topic == "/low_state":

            if first_t > 0: time_secs = time.to_sec() - first_t
            else:           first_t = time.to_sec()
            print(msg)
            data.append({   'R'   :msg.imu.rpy[0],            'P' :msg.imu.rpy[1],             'Y' :msg.imu.rpy[2],
                            "dR"  :msg.imu.gyroscope[0],     "dP" :msg.imu.gyroscope[1],      "dP" :msg.imu.gyroscope[2],  
                            "qx"  :msg.imu.quaternion[0],    "qy" :msg.imu.quaternion[1],     "qz" :msg.imu.quaternion[2], "qw" :msg.imu.quaternion[3],
                         
                            "ddx" :msg.imu.accelerometer[0],"ddy" :msg.imu.accelerometer[1], "ddz" : msg.imu.accelerometer[2],

                            "FR0 q" :msg.motorState[0].q , "FR0 dq" :msg.motorState[0].q , "FR0 tauEst" :msg.motorState[0].tauEst,
                            "FR1 q" :msg.motorState[1].q , "FR1 dq" :msg.motorState[1].q , "FR1 tauEst" :msg.motorState[1].tauEst,
                            "FR2 q" :msg.motorState[2].q , "FR2 dq" :msg.motorState[2].q , "FR2 tauEst" :msg.motorState[2].tauEst,

                            "FL0 q" :msg.motorState[3].q , "FL0 dq" :msg.motorState[3].q , "FL0 tauEst" :msg.motorState[3].tauEst,
                            "FL1 q" :msg.motorState[4].q , "FL1 dq" :msg.motorState[4].q , "FL1 tauEst" :msg.motorState[4].tauEst,
                            "FL2 q" :msg.motorState[5].q , "FL2 dq" :msg.motorState[5].q , "FL2 tauEst" :msg.motorState[5].tauEst,

                            "RR0 q" :msg.motorState[6].q , "RR0 dq" :msg.motorState[6].q , "RR0 tauEst" :msg.motorState[6].tauEst,
                            "RR1 q" :msg.motorState[7].q , "RR1 dq" :msg.motorState[7].q , "RR1 tauEst" :msg.motorState[7].tauEst,
                            "RR2 q" :msg.motorState[8].q , "RR2 dq" :msg.motorState[8].q , "RR2 tauEst" :msg.motorState[8].tauEst,

                            "RL0 q" :msg.motorState[9].q , "RL0 dq" :msg.motorState[9].q , "RL0 tauEst" :msg.motorState[9].tauEst,
                            "RL1 q" :msg.motorState[10].q , "RL1 dq" :msg.motorState[10].q , "RL1 tauEst" :msg.motorState[10].tauEst,
                            "RL2 q" :msg.motorState[11].q , "RL2 dq" :msg.motorState[11].q , "RL2 tauEst" :msg.motorState[11].tauEst,
                            
                            "FR foot force" : msg.footForceEst[0], "FL foot force" : msg.footForceEst[1], "RR foot force" : msg.footForceEst[2], "RL foot force" : msg.footForceEst[3],
                            "time" : time_secs
                         })
            
    DataFrame = pd.DataFrame(data)

    DataFrame.to_csv(os.path.join(csvpath, bagName + ".csv"), index=False)

    print("{}.csv created!".format(bagName))


def main():

    for bag in bags:
        print("bag name is ", bag)
        if bag.endswith(".bag"):
            
            """
            Check if bag is already converted
            """

            if not (bag[ : -len(".bag")]+ ".csv" in  csvs):
                bag2csv(bag)
            
            

if __name__ == "__main__":

    """
    How this file works?

    => By running this file as it is,
       It will convert each bag into its corresponding csv
       Simple.
    """
    main()
    pass

