#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction

def main():
    rospy.init_node("test_traj_client", anonymous=True)

    action_name = "/arm_with_torso_controller/follow_joint_trajectory"
    client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)

    rospy.loginfo("Created SimpleActionClient for %s", action_name)
    rospy.loginfo("Calling wait_for_server(5.0s)...")

    ok = client.wait_for_server(rospy.Duration(5.0))

    rospy.loginfo("wait_for_server returned: %s", ok)
    if not ok:
        rospy.logwarn("Client could not detect the server. Check ROS_MASTER_URI/ROS networking.")
    else:
        rospy.loginfo("Action server is reachable and ready.")

if __name__ == "__main__":
    main()
