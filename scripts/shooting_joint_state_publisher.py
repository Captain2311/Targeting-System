#!/usr/bin/env python3  

import rospy
import tf2_ros
import math 
from sensor_msgs.msg import JointState

def shooting_joint_state_publisher():
    tfBuffer = tf2_ros.Buffer()
    trans_listener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(20.0)  # 10Hz

    while not rospy.is_shutdown():
        
        try:
            base_to_weed = tfBuffer.lookup_transform('base', 'weed', rospy.Time())
            #base_to_shoulder = tfBuffer.lookup_transform('base', 'shoulder_motor', rospy.Time())
        except tf2_ros.LookupException as e:
            rospy.logerr('TF2 lookup error: {}'.format(str(e)))
            rate.sleep()
            continue
        except tf2_ros.ConnectivityException as e:
            rospy.logerr('TF2 connectivity error: {}'.format(str(e)))
            rate.sleep()
            continue
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr('TF2 extrapolation error: {}'.format(str(e)))
            rate.sleep()
            continue

        # transformation needed from base to elbow, where 0.05 = base_to_shoulder.transform.translation.x
        # and z = base_to_shoulder.transform.translation.z + shoulder_to_elbow.transform.translation.z
        theta = math.atan2(base_to_weed.transform.translation.x, base_to_weed.transform.translation.y)
        x = base_to_weed.transform.translation.x - 0.05 * math.cos(theta) 
        y = base_to_weed.transform.translation.y + 0.05 * math.sin(theta)
        theta_2 = math.atan2(x, y) 
        z = 0.08
        # hypotenuse of x and y 
        hyp_xy = math.sqrt((base_to_weed.transform.translation.x )**2 + base_to_weed.transform.translation.y**2)
        
        # shoulder_yaw = math.atan2(base_to_weed.transform.translation.y,
        #                       base_to_weed.transform.translation.x) - math.pi/2 
        
        shoulder_yaw = - theta_2 
        
        elbow_roll = math.atan2((base_to_weed.transform.translation.z + z), hyp_xy) 

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['base_to_shoulder', 'shoulder_to_elbow']
        joint_state.position = [shoulder_yaw, elbow_roll]
        joint_state.velocity = [0.5, 0.5]
        joint_state.effort = [0.0, 0.0]

        pub.publish(joint_state)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        rospy.init_node('shooting_joint_state_publisher')
        shooting_joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
    
