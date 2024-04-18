#!/usr/bin/env python3
import rosbag
import rospkg
import rospy
from tf.transformations import euler_from_quaternion
import os
import pandas as pd
import argparse


package_path = rospkg.RosPack().get_path('Legged_bags')

bagpath = package_path + "/Bags/"
bags = os.listdir(bagpath)

csvpath = package_path + "/Csvs"
csvs = os.listdir(csvpath)

def bag2csv(bag):
    
    bagName = bag[ : -len(".bag")]

    bagfile = rosbag.Bag(os.path.join(bagpath, bag))

    data = [] 

    first_t = -1

    time_secs = 0.0
    dt        = 0.02

    dx, dy, dz, x, y, z = 0, 0, 0, 0, 0, 0

    for topic, msg, time in  bagfile.read_messages():

        if topic == "/low_state":

            if first_t > 0: time_secs = time.to_sec() - first_t
            else:           first_t = time.to_sec()

            data.append({   'R'   :msg.imu.rpy[0],            'P' :msg.imu.rpy[1],             'Y' :msg.imu.rpy[2],
                            "dR"  :msg.imu.gyroscope[0],     "dP" :msg.imu.gyroscope[1],      "dY" :msg.imu.gyroscope[2],  
                            "qx"  :msg.imu.quaternion[0],    "qy" :msg.imu.quaternion[1],     "qz" :msg.imu.quaternion[2], "qw" :msg.imu.quaternion[3],
                         
                            "ddx" :msg.imu.accelerometer[0],"ddy" :msg.imu.accelerometer[1], "ddz" : msg.imu.accelerometer[2] - 9.81,

                            "dx" : dx, "dy" : dy, "dz" : dz,

                            "x" : x, "y" : y, "z" : z,

                            "FR0 q" :msg.motorState[0].q , "FR0 dq" :msg.motorState[0].dq , "FR0 tauEst" :msg.motorState[0].tauEst,
                            "FR1 q" :msg.motorState[1].q , "FR1 dq" :msg.motorState[1].dq , "FR1 tauEst" :msg.motorState[1].tauEst,
                            "FR2 q" :msg.motorState[2].q , "FR2 dq" :msg.motorState[2].dq , "FR2 tauEst" :msg.motorState[2].tauEst,

                            "FL0 q" :msg.motorState[3].q , "FL0 dq" :msg.motorState[3].dq , "FL0 tauEst" :msg.motorState[3].tauEst,
                            "FL1 q" :msg.motorState[4].q , "FL1 dq" :msg.motorState[4].dq , "FL1 tauEst" :msg.motorState[4].tauEst,
                            "FL2 q" :msg.motorState[5].q , "FL2 dq" :msg.motorState[5].dq , "FL2 tauEst" :msg.motorState[5].tauEst,

                            "RR0 q" :msg.motorState[6].q , "RR0 dq" :msg.motorState[6].dq , "RR0 tauEst" :msg.motorState[6].tauEst,
                            "RR1 q" :msg.motorState[7].q , "RR1 dq" :msg.motorState[7].dq , "RR1 tauEst" :msg.motorState[7].tauEst,
                            "RR2 q" :msg.motorState[8].q , "RR2 dq" :msg.motorState[8].dq , "RR2 tauEst" :msg.motorState[8].tauEst,

                            "RL0 q" :msg.motorState[9].q , "RL0 dq" :msg.motorState[9].dq , "RL0 tauEst" :msg.motorState[9].tauEst,
                            "RL1 q" :msg.motorState[10].q , "RL1 dq" :msg.motorState[10].dq , "RL1 tauEst" :msg.motorState[10].tauEst,
                            "RL2 q" :msg.motorState[11].q , "RL2 dq" :msg.motorState[11].dq , "RL2 tauEst" :msg.motorState[11].tauEst,
                            
                            "FR foot force" : msg.footForce[0], "FL foot force" : msg.footForce[1], "RR foot force" : msg.footForce[2], "RL foot force" : msg.footForce[3],
                            "time" : time_secs,
                            # "lx"   : msg.wirelessRemote[16],
                            # "rx"   : msg.wirelessRemote[17],
                            # "ry"   : msg.wirelessRemote[18],
                            # "ly"   : msg.wirelessRemote[20]
                         })
            
            #Deadreckoning
            dx +=msg.imu.accelerometer[0] * dt
            x  += dx*dt + 0.5*msg.imu.accelerometer[0]*dt*dt

            dy +=msg.imu.accelerometer[1] * dt
            y  += dy*dt + 0.5*msg.imu.accelerometer[1]*dt*dt

            dz += (msg.imu.accelerometer[2] - 9.81)* dt
            z  += dz*dt + 0.5*(msg.imu.accelerometer[2] - 9.81)*dt*dt
            #Deadreckoning
        
        #NOTE: Can access only one state
        elif topic == "/high_state":
            if first_t > 0: time_secs = time.to_sec() - first_t
            else:           first_t = time.to_sec()

            data.append({   'R'   :msg.imu.rpy[0],            'P' :msg.imu.rpy[1],             'Y' :msg.imu.rpy[2],
                            "dR"  :msg.imu.gyroscope[0],     "dP" :msg.imu.gyroscope[1],      "dY" :msg.imu.gyroscope[2],  
                            "qx"  :msg.imu.quaternion[0],    "qy" :msg.imu.quaternion[1],     "qz" :msg.imu.quaternion[2], "qw" :msg.imu.quaternion[3],
                        
                            "ddx" :msg.imu.accelerometer[0],"ddy" :msg.imu.accelerometer[1], "ddz" : msg.imu.accelerometer[2] - 9.81,

                            "dx" : msg.velocity[0], "dy" : msg.velocity[1], "dz" : msg.velocity[2],

                            "x" : msg.position[0], "y" : msg.position[1], "z" : msg.position[2],

                            "FR0 q" :msg.motorState[0].q , "FR0 dq" :msg.motorState[0].dq , "FR0 tauEst" :msg.motorState[0].tauEst,
                            "FR1 q" :msg.motorState[1].q , "FR1 dq" :msg.motorState[1].dq , "FR1 tauEst" :msg.motorState[1].tauEst,
                            "FR2 q" :msg.motorState[2].q , "FR2 dq" :msg.motorState[2].dq , "FR2 tauEst" :msg.motorState[2].tauEst,

                            "FL0 q" :msg.motorState[3].q , "FL0 dq" :msg.motorState[3].dq , "FL0 tauEst" :msg.motorState[3].tauEst,
                            "FL1 q" :msg.motorState[4].q , "FL1 dq" :msg.motorState[4].dq , "FL1 tauEst" :msg.motorState[4].tauEst,
                            "FL2 q" :msg.motorState[5].q , "FL2 dq" :msg.motorState[5].dq , "FL2 tauEst" :msg.motorState[5].tauEst,

                            "RR0 q" :msg.motorState[6].q , "RR0 dq" :msg.motorState[6].dq , "RR0 tauEst" :msg.motorState[6].tauEst,
                            "RR1 q" :msg.motorState[7].q , "RR1 dq" :msg.motorState[7].dq , "RR1 tauEst" :msg.motorState[7].tauEst,
                            "RR2 q" :msg.motorState[8].q , "RR2 dq" :msg.motorState[8].dq , "RR2 tauEst" :msg.motorState[8].tauEst,

                            "RL0 q" :msg.motorState[9].q , "RL0 dq" :msg.motorState[9].dq , "RL0 tauEst" :msg.motorState[9].tauEst,
                            "RL1 q" :msg.motorState[10].q , "RL1 dq" :msg.motorState[10].dq , "RL1 tauEst" :msg.motorState[10].tauEst,
                            "RL2 q" :msg.motorState[11].q , "RL2 dq" :msg.motorState[11].dq , "RL2 tauEst" :msg.motorState[11].tauEst,
                            
                            "FR foot force" : msg.footForce[0], "FL foot force" : msg.footForce[1], "RR foot force" : msg.footForce[2], "RL foot force" : msg.footForce[3],
                            
                            "gaitType" : msg.gaitType,

                            "yawSpeed" : msg.yawSpeed,

                            "footRaiseHeight" : msg.footRaiseHeight,

                            "bodyHeight" : msg.bodyHeight,
                            
                            "time" : time_secs,
                            # "lx"   : msg.wirelessRemote[16],
                            # "rx"   : msg.wirelessRemote[17],
                            # "ry"   : msg.wirelessRemote[18],
                            # "ly"   : msg.wirelessRemote[20]
                         })
            
        #NOTE: Hound data 
        elif topic == "/sensors/core":
            if first_t > 0: time_secs = time.to_sec() - first_t
            else:           first_t = time.to_sec()

            data.append({
                        "Speed" : msg.state.speed,
                        "Input Current" : msg.state.current_input,
                        "Input Voltage"  : msg.state.voltage_input,
                        "temperature_pcb" : msg.state.temperature_pcb,
                         "time" : time_secs,
                        })
            
        elif topic == "/mavros/imu/data":
            if first_t > 0: time_secs = time.to_sec() - first_t
            else:           first_t = time.to_sec()
            roll, pitch, yaw = euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
            data.append({
                        "R" : roll, "P" : pitch, "Y" : yaw,
                        "dR" : msg.angular_velocity.x,  "dP" : msg.angular_velocity.y,  "dY" : msg.angular_velocity.z,
                        "ddx" :  msg.linear_acceleration.x, "ddy" :  msg.linear_acceleration.y, "ddz" :  msg.linear_acceleration.z,
                        "time" : time_secs,
                        })
            
        elif topic == "/mavros/rc/out":
            if first_t > 0: time_secs = time.to_sec() - first_t
            else:           first_t = time.to_sec()
            data.append({
                        "steering_angle" : msg.channels[3] - 1500,
                        "throttle"       : msg.channels[2] - 1500,
                        "time" : time_secs,
                        })

            
    df = pd.DataFrame(data)
    df.ffill(inplace=True)
    df.interpolate(method="linear", inplace=True)
    df.bfill(inplace=True)
    df.to_csv(os.path.join(csvpath, bagName + ".csv"), index=False)

    print("{}.csv created!".format(bagName))


def main(bag_name):

    #check if user has provided a bag to get converted.
    if bag_name != "_.bag":

        for bag in bags:
            
                if bag.endswith(".bag"):

                        if bag == bag_name:
                            """
                            Check if bag is already converted
                            """
                            if not (bag[ : -len(".bag")]+ ".csv" in  csvs):
                                bag2csv(bag)
                            else:
                                print("{}.csv already created!".format(bag[ : -len(".bag")]))
                
    else:

        for bag in bags:

            if bag.endswith(".bag"):
                
                """
                Check if bag is already converted
                """

                if not (bag[ : -len(".bag")]+ ".csv" in  csvs):
                    bag2csv(bag)
                else:
                    print("{}.csv already created!".format(bag[ : -len(".bag")]))

            
            

if __name__ == "__main__":

    """
    How this file works?

    => By running this file as it is,
       It will convert each bag into its corresponding csv
       Simple.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag_name",
        type=str,
        default="_.bag",
        help="murray bag number to be used",
    )
    args = parser.parse_args()
    bag_name = args.bag_name

    main(bag_name)
    pass

