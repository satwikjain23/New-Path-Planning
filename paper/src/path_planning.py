#!/usr/bin/env python3

import rospy
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from clustering.msg import Coordinates
from chut.msg import slam
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu
import tf.transformations
from chut.msg import pid_input
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R
import numpy as np
from chut.msg import path

leftcone=[]
rightcone=[]

forward_projection = 0.3  #0.5	
vel = 15 		
error = 0.0		
car_length = 0.50 
b=True
flag = 0
car_coordinate=[0,0]
yaw=0

test=[0,0]

path_pub= rospy.Publisher('/path',path,queue_size=100)
pub = rospy.Publisher('/err', pid_input, queue_size=100)

# translation = np.array([1.58, 0.0, -0.1])
# quat = [0.0, 0.008726535498373935, 0.0, 0.9999619230641713]  # x, y, z, w

# # Create a Rotation object from quaternion
# rotation = R.from_quat(quat)  # [x, y, z, w]

t1 = np.array([1.58, 0.0, -0.1])
r1 = R.from_quat([0.0, 0.008726535498373935, 0.0, 0.9999619230641713])

# Transform 2: velodyne_base_link -> velodyne
t2 = np.array([0.0, 0.0, 0.0377])
r2 = R.from_quat([0.0, 0.0, 0.0, 1.0])  # identity rotation

# Combined transform: base_link -> velodyne
# R_total = R1 * R2
# T_total = R1.apply(t2) + t1
r_total = r1 * r2
t_total = r1.apply(t2) + t1

# Now invert the full transform: velodyne <- base_link
r_inv = r_total.inv()
t_inv = -r_inv.apply(t_total)

def gazebo_callback(msg):
    global car_coordinate, b, leftcone, rightcone, yaw, test
    # x_base = msg.pose[2].position.x
    # y_base = msg.pose[2].position.y
    # z_base = msg.pose[2].position.z  # If available, else set to 0

    # pos_base = np.array([x_base, y_base, z_base])

    # pos_lidar = rotation.inv().apply(pos_base - translation)
    x = msg.pose[2].position.x
    y = msg.pose[2].position.y
    z = msg.pose[2].position.z 

    pos_base = np.array([x, y, z])

    # Transform GPS to velodyne frame
    pos_lidar = r_inv.apply(pos_base) + t_inv
    car_coordinate[0]=pos_lidar[0]
    car_coordinate[1]=pos_lidar[1]

    # print("GPS coordinate in velodyne frame:", car_coordinate)
    
    if(b):
        test[0]=car_coordinate[0]
        test[1]=car_coordinate[1]
        leftcone.append(car_coordinate)
        rightcone.append(car_coordinate)
        b = False
         
    

    midpoints=[]
    if(len(leftcone)>=2):
        for i in range(2):
            mid=[]
            min_left= leftcone[i]
            min_right= rightcone[i]
            mid.append((min_left[0]+min_right[0])/2)
            mid.append((min_left[1]+min_right[1])/2)
            midpoints.append(mid)
        midpoints[0][0]=(midpoints[0][0] +test[0])/2
        midpoints[0][1]=(midpoints[0][1] +test[1])/2
        m=path()
        m.first=midpoints[0]
        m.second=midpoints[1]
        path_pub.publish(m)

        num=midpoints[1][1]-midpoints[0][1]
        den=(midpoints[1][0]-midpoints[0][0])

        slope= abs(num/den)
        ideal_orientation=math.atan(slope)

        dist_midpoints=abs(math.sqrt(num**2 + den**2))
        ideal=math.acos(den/dist_midpoints)
		
        val=midpoints[1][1]-midpoints[0][1]

        if(val<0):
            ideal=ideal*(-1)

        A=midpoints[0][1]-midpoints[1][1]
        B=midpoints[1][0]-midpoints[0][0]
        C=(midpoints[0][0]*midpoints[1][1])-(midpoints[1][0]*midpoints[0][1])
        err=(abs((A*car_coordinate[0])+(B*car_coordinate[1]+C))/math.sqrt((A**2)+(B**2)))
        # d=((car_coordinate[0]-midpoints[0][0])*(midpoints[1][1]-midpoints[0][1]))-((car_coordinate[1]-midpoints[0][1])*(midpoints[1][0]-midpoints[0][1]))
        d = ((midpoints[1][1] - midpoints[0][1])* car_coordinate[0]) - ((midpoints[1][0] - midpoints[0][0])*car_coordinate[1]) + (midpoints[1][0]*midpoints[0][1]) - (midpoints[0][0]*midpoints[1][1])
        Alpha= yaw-ideal
        if(d<0):
            print("left")
            error=forward_projection*math.sin(Alpha)-err
        else:
            print("right")
            error=forward_projection*math.sin(Alpha)+err
        print("err",err)
        print("alpha=",math.degrees(Alpha),math.sin(Alpha),"projection",forward_projection*math.sin(Alpha))
        print(midpoints)
        print("Error ", error)
        A1=leftcone[1][1]-rightcone[1][1]
        B1=rightcone[1][0]-leftcone[1][0]
        C1=(leftcone[1][0]*rightcone[1][1])-(rightcone[1][0]*leftcone[1][1])
        d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))

        if(d>0 and d<0.3):
            test[0]=car_coordinate[0]
            test[1]=car_coordinate[1]
            leftcone.pop(0)
            rightcone.pop(0)
        msg= pid_input()
        msg.pid_error=error
        msg.pid_vel=vel
        pub.publish(msg)
        print("----------------------------")
 
    


def imu_callback(msg):
    global yaw
    # init_theta = 0
    quart = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (_, _, yaw) = tf.transformations.euler_from_quaternion(quart)
	
    # yaw = math.degrees(yaw)
    # print("yaw: ", yaw)

def call(data):
    global leftcone,rightcone, test, car_coordinate
    # test[0]=car_coordinate[0]
    # test[1]=car_coordinate[1]
    leftcone.append(data.leftcone)
    rightcone.append(data.rightcone)



if __name__ == "__main__":
	print("Path_Planning")
	rospy.init_node('Path_Planning')
	rospy.Subscriber('/gazebo/model_states', ModelStates, gazebo_callback)
	rospy.Subscriber("/slam_to_distfinder", slam, call)
	rospy.Subscriber('/imu', Imu, imu_callback)
	
	rospy.spin()