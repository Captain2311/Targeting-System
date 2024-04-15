#!/usr/bin/env python3 
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point

def weed_pose_callback(point_msg):
    global weed_pose_x, weed_pose_y, weed_pose_z
    weed_pose_x = point_msg.x
    weed_pose_y = point_msg.y
    weed_pose_z = point_msg.z
    rospy.loginfo("x: %.2f, y: %.2f, z: %.2f", weed_pose_x, weed_pose_y, weed_pose_z)

if __name__ == '__main__':
    rospy.init_node('weed_frame_broadcaster')
    weed_pose_x = 0.45
    weed_pose_y = 0.45
    weed_pose_z = 0.0
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    pose_sub = rospy.Subscriber("/weed_real_xyz", Point, weed_pose_callback)

    t.header.frame_id = "camera_rgb_optical_frame"
    t.child_frame_id = "weed_link"

    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = weed_pose_x
        t.transform.translation.y = weed_pose_y
        t.transform.translation.z = weed_pose_z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)
        rate.sleep()
        