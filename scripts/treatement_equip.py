#!/usr/bin/env python3  

import rospy
import tf
import math 
from sensor_msgs.msg import JointState

def shooting_joint_state_publisher():
    listener = tf.TransformListener()
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    shoulder_pitch = 0.0
    holder_roll = 0.0
    
    rate = rospy.Rate(10.0)  # Hz

    while not rospy.is_shutdown():
        
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['base_to_shoulder_joint', 'shoulder_to_holder_joint']
        joint_state.position = [shoulder_pitch, holder_roll]
        joint_state.velocity = [0.0, 0.0]
        joint_state.effort = [0.0, 0.0]
        
        pub.publish(joint_state)
        
        try:            
            (base_to_weed_trans, _) = listener.lookupTransform('base_link', 'weed_link', rospy.Time(0))
            (shoulder_to_weed_trans, _) = listener.lookupTransform('shoulder_link', 'weed_link', rospy.Time(0))
            (shoulder_to_laser_trans, _) = listener.lookupTransform('shoulder_link', 'laser_link', rospy.Time(0))           
            
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
        
        if base_to_weed_trans is not None:
            base_to_weed_hyp = math.sqrt(base_to_weed_trans[0]**2 + base_to_weed_trans[1]**2)
            shoulder_pitch = - math.pi/2 - math.atan2(base_to_weed_trans[0], base_to_weed_trans[1]) - math.atan2(shoulder_to_laser_trans[2], base_to_weed_hyp + shoulder_to_laser_trans[0])
            
            holder_roll = math.atan2(shoulder_to_weed_trans[1] - shoulder_to_laser_trans[1], base_to_weed_hyp + shoulder_to_laser_trans[0])
        
        rate.sleep()
        

if __name__ == '__main__':
    try:
        rospy.init_node('treatement_equip_joint_state_publisher')
        shooting_joint_state_publisher()
    except rospy.ROSInterruptException:
        pass