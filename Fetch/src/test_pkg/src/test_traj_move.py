#!/usr/bin/env python3

import rospy
import actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

ACTION_NAME = "/arm_with_torso_controller/follow_joint_trajectory"

ARM_WITH_TORSO_JOINT_NAMES = [
    "torso_lift_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "upperarm_roll_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
]

latest_joint_positions = {}
have_arm_state = False

def joint_states_cb(msg):
    global latest_joint_positions, have_arm_state
    for name, pos in zip(msg.name, msg.position):
        latest_joint_positions[name] = pos

    if all(j in latest_joint_positions for j in ARM_WITH_TORSO_JOINT_NAMES):
        have_arm_state = True

def main():
    rospy.init_node("test_traj_move", anonymous=True)

    rospy.Subscriber("/joint_states", JointState, joint_states_cb, queue_size=1)

    client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for %s...", ACTION_NAME)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Action server %s not available", ACTION_NAME)
        return
    rospy.loginfo("Connected to %s.", ACTION_NAME)

    # Wait to get a full arm state
    rospy.loginfo("Waiting for joint_states...")
    while not rospy.is_shutdown() and not have_arm_state:
        rospy.sleep(0.1)
    if rospy.is_shutdown():
        return

    rospy.loginfo("Got arm joint state, building goal...")

    # Start with the current positions
    positions = [latest_joint_positions[j] for j in ARM_WITH_TORSO_JOINT_NAMES]

    # Nudge one joint so we can see clear motion (e.g. elbow_flex)
    elbow_index = ARM_WITH_TORSO_JOINT_NAMES.index("elbow_flex_joint")
    positions[elbow_index] += 0.3  # bend elbow by +0.3 rad (~17 deg)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ARM_WITH_TORSO_JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(3.0)  # 3 seconds to reach pose
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()

    rospy.loginfo("Sending goal positions: %s", positions)
    client.send_goal(goal)

    finished = client.wait_for_result(rospy.Duration(5.0))
    state = client.get_state()
    rospy.loginfo("Action finished=%s, state=%s", finished, state)


if __name__ == "__main__":
    main()
