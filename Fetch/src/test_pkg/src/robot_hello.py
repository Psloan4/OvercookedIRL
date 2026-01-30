#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def joint_states_cb(msg):
    # Print something at most once per second so it doesn't spam
    rospy.loginfo_throttle(1.0, "Heard joint_states with %d joints", len(msg.name))

def main():
    rospy.init_node('robot_hello')
    rospy.Subscriber('/joint_states', JointState, joint_states_cb)
    rospy.loginfo("robot_hello node started, waiting for /joint_states...")
    rospy.spin()

if __name__ == "__main__":
    main()
