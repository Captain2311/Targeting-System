#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf

rospy.init_node('light_trail_publisher')
marker_pub = rospy.Publisher('light_trail_marker', Marker, queue_size=10)
listener = tf.TransformListener()

marker = Marker()
marker.header.frame_id = "laser_link"
marker.type = Marker.LINE_LIST
marker.action = Marker.ADD
marker.scale.x = 0.005  # Line width
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 1.0
marker.color.a = 1.0

# Define line points
point1 = Point()
point1.x = 0.0
point1.y = 0.0
point1.z = 0.0

rate = rospy.Rate(20)

while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()
    try:
        (laser_to_weed_trans, laser_to_weed_rot) = listener.lookupTransform('laser_link', 'weed_link', rospy.Time(0))     
    except tf.LookupException as e:
            rospy.loginfo('TF2 lookup error: {}'.format(str(e)))
            rate.sleep()
            continue
    except tf.ConnectivityException as e:
            rospy.logerr('TF2 connectivity error: {}'.format(str(e)))
            rate.sleep()
            continue
    except tf.ExtrapolationException as e:
            rospy.logerr('TF2 extrapolation error: {}'.format(str(e)))
            rate.sleep()
            continue   
           
    point2 = Point()
    point2.x = laser_to_weed_trans[0]
    point2.y = laser_to_weed_trans[1]
    point2.z = laser_to_weed_trans[2]
    marker.points.clear()
    marker.points.append(point1)
    marker.points.append(point2)
    marker_pub.publish(marker)
    rate.sleep()
