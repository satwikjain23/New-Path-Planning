#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from chut.msg import slam  # Custom message with leftcone and rightcone

car_coordinate = [0, 0]
count = 0
total = []  # Store list of tuples like ("left", [x, y]) or ("right", [x, y])

def call(data):
    global total
    if len(data.leftcone) == 2:
        total.append(("left", data.leftcone))
    if len(data.rightcone) == 2:
        total.append(("right", data.rightcone))

def main():
    rospy.init_node("ConeMarker")
    marker_pub = rospy.Publisher("/marker_visualization_marker", MarkerArray, queue_size=1)
    rate = rospy.Rate(1)
    rospy.Subscriber("/slam_to_distfinder", slam, call)

    global count, total

    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        print(total)

        for tag, coord in total:
            marker = Marker()

            marker.header.frame_id = "velodyne"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "cone_markers"
            marker.id = count
            count += 1

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.0)

            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4

            # Color: green for leftcones, blue for rightcones
            if tag == "left":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            elif tag == "right":
                marker.color.r = 1.0
                marker.color.g = 0.76
                marker.color.b = 0.03
                marker.color.a = 1.0

            marker_array.markers.append(marker)

        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == "__main__":
    main()






























# #!/usr/bin/env python3

# import rospy
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray
# from geometry_msgs.msg import Point
# from chut.msg import slam

# car_coordinate = [0, 0]
# prev = [0, 0]
# total = []
# pres = []
# diff = 0

# def call(data):
#     global total
#     total.append(data.leftcone)
#     total.append(data.rightcone)

# count = 0

# def main():
#     rospy.init_node("ConeMarker")
#     marker_pub = rospy.Publisher("/marker_visualization_marker", MarkerArray, queue_size=1)
#     rate = rospy.Rate(1)
#     rospy.Subscriber("/slam_to_distfinder", slam, call)

#     while not rospy.is_shutdown():
#         markerArraylala = MarkerArray()

#         global count,total
#         print(total)
#         for i in total:
#             marker = Marker()

#             # Set the frame ID and timestamp
#             marker.header.frame_id = "velodyne"
#             marker.header.stamp = rospy.Time.now()

#             # Set the namespace and ID for this marker
#             marker.ns = "basic_shapes"
#             marker.id = count
#             count += 1

#             # Set the marker type
#             marker.type = Marker.CYLINDER  # Use Marker.CYLINDER instead of Marker.Cylinder

#             # Set the marker action
#             marker.action = Marker.ADD
#             marker.lifetime = rospy.Duration(0.0)

#             # Set the pose of the marker
#             marker.pose.position.x = i[0]  # Set the position from the data
#             marker.pose.position.y = i[1]  # Set the position from the data
#             marker.pose.position.z = 0
#             marker.pose.orientation.x = 0.0
#             marker.pose.orientation.y = 0.0
#             marker.pose.orientation.z = 0.0
#             marker.pose.orientation.w = 1.0

#             # Set the scale of the marker
#             marker.scale.x = 0.4
#             marker.scale.y = 0.4  # Set scale for y and z to make it visible
#             marker.scale.z = 0.4

#             # Set the color
#             marker.color.r = 0.0
#             marker.color.g = 1.0
#             marker.color.b = 0.0
#             marker.color.a = 1.0

#             # Define the points (note that `points` is not used for cylindrical markers, so you can remove this section)
#             # p = Point()
#             # p.x = i[0]
#             # p.y = i[1]
#             # marker.points.append(p)

#             markerArraylala.markers.append(marker)    
#         marker_pub.publish(markerArraylala)

#         # Cycle between different shapes
#         rate.sleep()

# if __name__ == "__main__":
#     main()