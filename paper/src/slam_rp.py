#!/usr/bin/env python3
import rospy
import math
import numpy as np
from clustering.msg import CoordinateList
from clustering.msg import Coordinates
from perception.msg import uday
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from perception.msg import imu
# from datetime import datetime
# from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion, quaternion_from_euler

car_coordinate=[0.0,0.0]
min_distance=10000
min_left_coordinate=[-1000,-1000]
min_right_coordinate=[-1000,-1000]
left_cone_coordinates=[]
right_cone_coordinates=[]
pub= rospy.Publisher('/slam_to_distfinder',uday,queue_size=10)
car_coordinate_pub = rospy.Publisher('/CarCoordinate',Coordinates,queue_size=1)
imu_pub=rospy.Publisher('/Imu_yaw',imu,queue_size=1)
prev_yaw=0 
prev_m=0
prev_s=0
prev_ns=0
delta_t=0
yaw_t1=0
yaw=0
b=True
theta=0
a=True
clus = True
cones=[]
orig_theta = 0
temp=[]


def distance(x,y):
	d=math.sqrt(((x[0]-y[0])**2)+((x[1]-y[1])**2))
	return d

def Imucall(data):
	global roll, pitch, yaw,a, theta, yaw_t1, orig_theta
	orientation_q = data.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	# print("Y",yaw)
	yaw = math.degrees(yaw)
	# print("1",yaw)
	
	if yaw<0:
		yaw +=360
	# print("Yaw",yaw)
	# print("OrigTheta",orig_theta)

	if(a):
		orig_theta = yaw
		# theta=yaw - orig_theta
		# yaw_t1=yaw - orig_theta
		a=False
	# print("Yaw ",yaw)
	# print("2",orig_theta)
	# t=(yaw-orig_theta)
	# print(t)
	theta =math.radians(yaw-orig_theta)
	# print("theta", yaw-orig_theta)
	message=imu()
	message.yaw=theta
	
	imu_pub.publish(message)
	

def call(data):
	global theta,car_coordinate,min_distance, min_left_coordinate,min_right_coordinate,left_cone_coordinates,right_cone_coordinates,b,cones, yaw, yaw_t1, orig_theta,cones
	# d=0.205
	if (clus):
		return
	d = data.data
	print("d=",d)
	print("theta=",math.degrees(theta))
	

	# print("theta",theta)
	# yaw_t1=yaw
	car_coordinate[0] += d*np.cos(theta)
	car_coordinate[1] += d*np.sin(theta)
	current_coordinate = Coordinates()
	current_coordinate.x = car_coordinate[0]
	current_coordinate.y = car_coordinate[1]
	current_coordinate.z = theta
	print("Current Coordinates ", current_coordinate)
	# current_coordinate.z = current_coordinate.color = 0
	car_coordinate_pub.publish(current_coordinate)
	# print(cones)
	for i in range(len(cones)):
		temp_x = cones[i][0]
		temp_y = cones[i][1]
		cones[i][0] = temp_x*np.cos(theta) - temp_y*np.sin(theta)
		cones[i][1] = temp_x*np.sin(theta) + temp_y*np.cos(theta)
		cones[i][0]+= car_coordinate[0]
		cones[i][1]+= car_coordinate[1]
	print("------------------------------")
	print("cones=",cones)
	
	# car_coordinate[1]+=d
	# print("car_x",car_coordinate[0])
	# print("car_y",car_coordinate[1])
	if(len(left_cone_coordinates)==0 or len(right_cone_coordinates)==0):
		reference_x=car_coordinate[0]
		reference_y=car_coordinate[1]
	else:
		reference_x=(left_cone_coordinates[-1][0]+right_cone_coordinates[-1][0])/2
		reference_y=(left_cone_coordinates[-1][1]+right_cone_coordinates[-1][1])/2
	reference=[reference_x,reference_y]
	print("reference",reference)
	cones_new=[]
	flag=[1]*len(cones)
	if(len(left_cone_coordinates)!=0 and len(right_cone_coordinates)!=0):
		for i in range(0,len(cones)):
			for j in range(0,len(left_cone_coordinates)):
				p=[cones[i][0],cones[i][1]]
				# print("distleft",left_cone_coordinates[j],p,distance(left_cone_coordinates[j],p))
				# print("distright",right_cone_coordinates[j],p,distance(right_cone_coordinates[j],p))
				if(distance(left_cone_coordinates[j],p)<2 or distance(right_cone_coordinates[j],p)<2 ):
					# print("removed",p)
					flag[i]=0
					#cones_new.append(cones[i])

		for i in range(0,len(cones)):
			if(flag[i]==1):
				cones_new.append(cones[i])
		cones=cones_new
	for i in range(0,len(cones)):
		for j in range(i+1,len(cones)):
			print("i=",cones[i]," j=",cones[j])
			z=True
			vec1_i=1
			vec1_j=1
			vec2_i=1
			vec2_j=1
			angle=180
			if(len(left_cone_coordinates)!=0):
				if(distance(cones[i],left_cone_coordinates[-1])<1.4):
					z=False
					cones[i][0]=left_cone_coordinates[-1][0]
					cones[i][1]=left_cone_coordinates[-1][1]
					vec1_i=cones[j][0]-left_cone_coordinates[-1][0]
					vec1_j=cones[j][1]-left_cone_coordinates[-1][1]
					vec2_i=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
					vec2_j=right_cone_coordinates[-1][1]-left_cone_coordinates[-1][1]
				elif(distance(cones[i],right_cone_coordinates[-1])<1.4):
					z=False
					cones[i][0]=right_cone_coordinates[-1][0]
					cones[i][1]=right_cone_coordinates[-1][1]
					vec1_i=cones[j][0]-right_cone_coordinates[-1][0]
					vec1_j=cones[j][1]-right_cone_coordinates[-1][1]
					vec2_i=left_cone_coordinates[-1][0]-right_cone_coordinates[-1][0]
					vec2_j=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
				elif(distance(cones[j],left_cone_coordinates[-1])<1.4):
					z=False
					cones[j][0]=left_cone_coordinates[-1][0]
					cones[j][1]=left_cone_coordinates[-1][1]
					vec1_i=cones[i][0]-left_cone_coordinates[-1][0]
					vec1_j=cones[i][1]-left_cone_coordinates[-1][1]
					vec2_i=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
					vec2_j=right_cone_coordinates[-1][1]-left_cone_coordinates[-1][1]
				elif(distance(cones[j],right_cone_coordinates[-1])<1.4):
					z=False
					cones[j][0]=right_cone_coordinates[-1][0]
					cones[j][1]=right_cone_coordinates[-1][1]
					vec1_i=cones[i][0]-right_cone_coordinates[-1][0]
					vec1_j=cones[i][1]-right_cone_coordinates[-1][1]
					vec2_i=left_cone_coordinates[-1][0]-right_cone_coordinates[-1][0]
					vec2_j=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
			if(z==False):
				dot=(vec1_i*vec2_i)+(vec1_j*vec2_j)
				mag1=math.sqrt((vec1_i**2)+(vec1_j**2))
				mag2=math.sqrt((vec2_i**2)+(vec2_j**2))
				angle=math.acos(dot/(mag1*mag2))
				angle=math.degrees(angle)
	  

			left_angle=1
			if(len(left_cone_coordinates)>=2 and distance(left_cone_coordinates[-1],cones[j])>0.5):
				# left_angle=180
				k=-2
				while(distance(left_cone_coordinates[k],left_cone_coordinates[-1])<0.3):
					k-=1
					if(abs(k)>len(left_cone_coordinates)):
						k=0
						break
				if(k!=0):
					v1_i=cones[j][0]-left_cone_coordinates[-1][0]
					v1_j=cones[j][1]-left_cone_coordinates[-1][1]
					v2_i=left_cone_coordinates[-1][0]-left_cone_coordinates[k][0]
					v2_j=left_cone_coordinates[-1][1]-left_cone_coordinates[k][1]
					dot=(v1_i*v2_i)+(v1_j*v2_j)
					mag1=math.sqrt((v1_i**2)+(v1_j**2))
					mag2=math.sqrt((v2_i**2)+(v2_j**2))
					# print("mag1=",mag1," mag2=",mag2," dot=",dot," theta=",dot/(mag1*mag2))
					left_angle=math.acos(dot/(mag1*mag2))
					left_angle=math.degrees(left_angle)

			right_angle=1
			if(len(right_cone_coordinates)>=2 and distance(right_cone_coordinates[-1],cones[i])>0.5):
				# right_angle=180
				k=-2
				while(distance(right_cone_coordinates[k],right_cone_coordinates[-1])<0.3):
					k-=1
					if(abs(k)>len(right_cone_coordinates)):
						k=0
						break
				if(k!=0):
					v1_i=cones[i][0]-right_cone_coordinates[-1][0]
					v1_j=cones[i][1]-right_cone_coordinates[-1][1]
					v2_i=right_cone_coordinates[-1][0]-right_cone_coordinates[k][0]
					v2_j=right_cone_coordinates[-1][1]-right_cone_coordinates[k][1]
					dot=(v1_i*v2_i)+(v1_j*v2_j)
					mag1=math.sqrt((v1_i**2)+(v1_j**2))
					mag2=math.sqrt((v2_i**2)+(v2_j**2))
					# print("mag1=",mag1," mag2=",mag2," dot=",dot," theta=",dot/(mag1*mag2))
					right_angle=math.acos(dot/(mag1*mag2))
					right_angle=math.degrees(right_angle)
			ang=0
			if(len(left_cone_coordinates)>1):
				prev_dir_i=reference_x-((left_cone_coordinates[-2][0]+right_cone_coordinates[-2][0])/2)
				prev_dir_j=reference_y-((left_cone_coordinates[-2][1]+right_cone_coordinates[-2][1])/2)
				new_dir_i=((cones[i][0]+cones[j][0])/2)-reference_x
				new_dir_j=((cones[i][1]+cones[j][1])/2)-reference_y
				m1=math.sqrt((prev_dir_i**2)+(prev_dir_j**2))
				m2=math.sqrt((new_dir_i**2)+(new_dir_j**2))
				do=(prev_dir_i*new_dir_i)+(prev_dir_j*new_dir_j)
				ang=math.acos(do/(m1*m2))
				ang=math.degrees(ang)

			u=True
			if(len(left_cone_coordinates)>1):
				print("u_left=",distance(left_cone_coordinates[-1],cones[j]),"u_right=",distance(right_cone_coordinates[-1],cones[i]))
				u=False
				if(distance(left_cone_coordinates[-1],cones[j])<5.5):
					if(distance(right_cone_coordinates[-1],cones[i])<5.0):
						u=True

			
			mid_x=(cones[i][0]+cones[j][0])/2
			mid_y=(cones[i][1]+cones[j][1])/2
			mid=[mid_x,mid_y]
			minimum=distance(reference,mid)
			print("minimum distance=",minimum)
			print("angle=",angle,"left_angle=",left_angle," right_angle=",right_angle," ang=",ang," z=",z," u=",u," distance=",distance(cones[i],cones[j]))
			if ((ang<80) and (left_angle<45) and (right_angle<60) and (z  or (angle<55 and angle>15)) and u and distance(cones[i],cones[j])<9.3):
				print("inside 1")
				if((minimum<min_distance) and (minimum>0.8)):
					print("inside 2")
					min_distance=minimum
					l=[0,0]
					r=[0,0]
					l[0]=cones[j][0]
					l[1]=cones[j][1]
					r[0]=cones[i][0]
					r[1]=cones[i][1]
					min_left_coordinate=l
					min_right_coordinate=r
	print("min left coordinate=",min_left_coordinate)
	print("min right coordinate=",min_right_coordinate)


	if ((len(left_cone_coordinates)==0) and (distance(reference,car_coordinate)<1)):
		left_cone_coordinates.append(min_left_coordinate)
		right_cone_coordinates.append(min_right_coordinate)
		message=uday()
		message.leftcone=min_left_coordinate
		message.rightcone=min_right_coordinate
		pub.publish(message)
		print("published")
		min_distance=10000
		min_left_coordinate=[-1000,-1000]
		min_right_coordinate=[-1000,-1000]
		
	else:
		A1=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
		B1=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
		C1=(left_cone_coordinates[-1][0]*right_cone_coordinates[-1][1])-(right_cone_coordinates[-1][0]*left_cone_coordinates[-1][1])
		print("A1=",A1," B1=",B1," c1=",C1)
		d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))
		# print("d", d)
		if(d<0.3):
			left_cone_coordinates.append(min_left_coordinate)
			right_cone_coordinates.append(min_right_coordinate)
			message=uday()
			message.leftcone=min_left_coordinate
			message.rightcone=min_right_coordinate
			pub.publish(message)
			print("published")
			min_distance=10000
			min_left_coordinate=[-1000,-1000]
			min_right_coordinate=[-1000,-1000]
					

	
	# print("c",cones)
	# for i in range(0,len(cones)):
	# 	for j in range(i+1,len(cones)):
	# 		mid_x=(cones[i][0]+cones[j][0])/2
	# 		mid_y=(cones[i][1]+cones[j][1])/2
	# 		mid=[mid_x,mid_y]
	# 		minimum=distance(reference,mid)
	# 		print("==")
	# 		print("first cone=",cones[i])
	# 		print("second cone=",cones[j])
	# 		print("distance between them=",minimum)
	# 		print("--")
	# 		if(minimum<min_distance):
	# 			min_distance=minimum
	# 			#the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
	# 			#be the position of right cone and cones[j] will ve the position of left cone
	# 			l=[0,0]
	# 			r=[0,0]
	# 			l[0]=cones[j][0]
	# 			l[1]=cones[j][1]
	# 			r[0]=cones[i][0]
	# 			r[1]=cones[i][1]
	# 			min_left_coordinate=l
	# 			min_right_coordinate=r
	# print("min left coordinate=",min_left_coordinate)
	# print("min right coordinate=",min_right_coordinate)

	# if(len(left_cone_coordinates)==0 and distance(reference,car_coordinate)<1):
	# 	left_cone_coordinates.append(min_left_coordinate)
	# 	right_cone_coordinates.append(min_right_coordinate)
	# 	message=uday()
	# 	message.leftcone=min_left_coordinate
	# 	message.rightcone=min_right_coordinate
	# 	pub.publish(message)
	# 	print("published")
	# 	min_distance=10000
	# 	min_left_coordinate=[-1,-1]
	# 	min_right_coordinate=[-1,-1]
		
	# else:
	# 	A1=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
	# 	B1=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
	# 	C1=(left_cone_coordinates[-1][0]*right_cone_coordinates[-1][1])-(right_cone_coordinates[-1][0]*left_cone_coordinates[-1][1])
	# 	d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))
	# 	# print("d", d)
	# 	if(d<1):
	# 		left_cone_coordinates.append(min_left_coordinate)
	# 		right_cone_coordinates.append(min_right_coordinate)
	# 		message=uday()
	# 		message.leftcone=min_left_coordinate
	# 		message.rightcone=min_right_coordinate
	# 		pub.publish(message)
	# 		print("published")
	# 		min_distance=10000
	# 		min_left_coordinate=[-1,-1]
	# 		min_right_coordinate=[-1,-1]

	# # print(left_cone_coordinates)
	# # print(right_cone_coordinates)
	# # print("----------------")


def callbackabx(data):
	global clus
	clus = False
	# print("hi")
	global cones
	cones = []
	for i in data.ConeCoordinates:
		cones.append([i.x, i.y])
	# print(cones)
	

if __name__ == "__main__":
	print("SLAM")
	rospy.init_node('Slam')
	rospy.Subscriber("/Clusters", CoordinateList, callbackabx)
	# rospy.Subscriber("/FusedCoordinates", CoordinateList, callbackabx)
	rospy.Subscriber("/distance_hall",Float32, call)
	rospy.Subscriber("/imu/data", Imu, Imucall)
	rospy.spin()