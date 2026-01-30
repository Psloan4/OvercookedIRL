#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# ---- ROS imports ----
import rospy
import actionlib
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# ============================================================
#                  KINEMATICS + ANGLE MAPPING
# ============================================================

# Visualization arm lengths
L1 = 1.0
L2 = 0.8
L3 = 0.6

def forward_kinematics(theta1, theta2, theta3):
    """
    theta1, theta2, theta3 are in radians.
    Planar 3R arm, returns x,y of base, joint1, joint2, end-effector.
    """
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    x3 = x2 + L3 * np.cos(theta1 + theta2 + theta3)
    y3 = y2 + L3 * np.sin(theta1 + theta2 + theta3)

    return [0.0, x1, x2, x3], [0.0, y1, y2, y3]

# Slider → internal angle mapping
# 0° on slider = straight down = -90° in math frame
def joint1_to_rad(angle_deg):
    return np.deg2rad(angle_deg - 90.0)

def joint_to_rad(angle_deg):
    return np.deg2rad(angle_deg)

# ============================================================
#          OFFSETS: GUI internal → Fetch joint angles
# ============================================================
# From your /joint_states snapshot at the GUI default pose (30, -30, -90)
# and the internal angles at that pose, we computed:
#   offset = fetch_angle_at_pose - gui_internal_angle_at_pose

OFFSET_SHOULDER_PAN  = 2.6172662619  # ≈ 150°
OFFSET_SHOULDER_LIFT = 1.5749423464  # ≈  90°
OFFSET_ELBOW_FLEX    = 2.3531147630  # ≈ 135°

# ============================================================
#                      ROS SETUP
# ============================================================

rospy.init_node("arm_gui", anonymous=True)

# Publish simulated arm pose
sim_pub = rospy.Publisher("/sim_arm/joint_states", JointState, queue_size=1)

# Cache of the latest /joint_states
latest_joint_positions = {}
have_arm_state = False

# arm_with_torso_controller joint order
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

def joint_states_cb(msg):
    global latest_joint_positions, have_arm_state
    for name, pos in zip(msg.name, msg.position):
        latest_joint_positions[name] = pos

    if all(j in latest_joint_positions for j in ARM_WITH_TORSO_JOINT_NAMES):
        have_arm_state = True

rospy.Subscriber("/joint_states", JointState, joint_states_cb, queue_size=1)

# Action client to arm_with_torso_controller
client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
rospy.loginfo("Waiting for %s...", ACTION_NAME)
if not client.wait_for_server(rospy.Duration(5.0)):
    rospy.logerr("Action server %s not available, GUI will still run but Send won't work.", ACTION_NAME)
else:
    rospy.loginfo("Connected to %s.", ACTION_NAME)

# ============================================================
#                     MATPLOTLIB GUI
# ============================================================

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.35)

# Initial sliders: your current working values
INIT_THETA1_DEG = 30
INIT_THETA2_DEG = -30
INIT_THETA3_DEG = -90

theta1_0 = joint1_to_rad(INIT_THETA1_DEG)
theta2_0 = joint_to_rad(INIT_THETA2_DEG)
theta3_0 = joint_to_rad(INIT_THETA3_DEG)

xs, ys = forward_kinematics(theta1_0, theta2_0, theta3_0)
(line,) = ax.plot(xs, ys, marker='o')

max_reach = L1 + L2 + L3
ax.set_xlim(-max_reach, max_reach)
ax.set_ylim(-max_reach, max_reach)
ax.set_aspect('equal', 'box')
ax.grid(True)
ax.set_title("3-DOF Planar Robotic Arm (GUI + Fetch)")

# Sliders
axcolor = 'lightgoldenrodyellow'
ax_theta1 = plt.axes([0.1, 0.25,  0.8, 0.03], facecolor=axcolor)
ax_theta2 = plt.axes([0.1, 0.20, 0.8, 0.03], facecolor=axcolor)
ax_theta3 = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor=axcolor)

s_theta1 = Slider(ax_theta1, "θ1 (deg)", 0, 135,  valinit=INIT_THETA1_DEG)
s_theta2 = Slider(ax_theta2, "θ2 (deg)", -135, 135, valinit=INIT_THETA2_DEG)
s_theta3 = Slider(ax_theta3, "θ3 (deg)", -135, 135, valinit=INIT_THETA3_DEG)

# ============================================================
#                     ROS HELPERS
# ============================================================

def publish_sim_state(t1, t2, t3):
    if rospy.is_shutdown():
        return
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ["sim_joint1", "sim_joint2", "sim_joint3"]
    msg.position = [t1, t2, t3]
    sim_pub.publish(msg)

# Slider callback
def update(val):
    j1 = s_theta1.val
    j2 = s_theta2.val
    j3 = s_theta3.val

    t1 = joint1_to_rad(j1)
    t2 = joint_to_rad(j2)
    t3 = joint_to_rad(j3)

    xs, ys = forward_kinematics(t1, t2, t3)
    line.set_xdata(xs)
    line.set_ydata(ys)
    fig.canvas.draw_idle()

    publish_sim_state(t1, t2, t3)

s_theta1.on_changed(update)
s_theta2.on_changed(update)
s_theta3.on_changed(update)

# Publish once at startup
publish_sim_state(theta1_0, theta2_0, theta3_0)

# ============================================================
#                  SEND-TO-FETCH BUTTON
# ============================================================

button_ax = plt.axes([0.75, 0.05, 0.15, 0.05])
send_button = Button(button_ax, "Send")

def send_to_fetch(event):
    if rospy.is_shutdown():
        return

    if not have_arm_state:
        rospy.logwarn("No complete arm+torso joint state yet; can't send trajectory.")
        return

    # Read current slider values (deg)
    j1 = s_theta1.val
    j2 = s_theta2.val
    j3 = s_theta3.val

    # GUI internal radians
    t1 = joint1_to_rad(j1)
    t2 = joint_to_rad(j2)
    t3 = joint_to_rad(j3)

    # Map GUI internal → Fetch joint angles with calibrated offsets
    shoulder_pan   = t1 + OFFSET_SHOULDER_PAN
    shoulder_lift  = t2 + OFFSET_SHOULDER_LIFT
    elbow_flex     = t3 + OFFSET_ELBOW_FLEX

    # Build full 8-DOF torso+arm vector, keeping others at current positions
    positions = []
    for name in ARM_WITH_TORSO_JOINT_NAMES:
        if name == "shoulder_pan_joint":
            positions.append(shoulder_pan)
        elif name == "shoulder_lift_joint":
            positions.append(shoulder_lift)
        elif name == "elbow_flex_joint":
            positions.append(elbow_flex)
        else:
            positions.append(latest_joint_positions.get(name, 0.0))

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ARM_WITH_TORSO_JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(3.0)  # 3 seconds to reach pose
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()

    rospy.loginfo("Sending arm_with_torso goal: %s", positions)
    client.send_goal(goal)

    # Wait for the result like in test_traj_move
    finished = client.wait_for_result(rospy.Duration(5.0))
    state = client.get_state()
    rospy.loginfo("Action finished=%s, state=%s", finished, state)

send_button.on_clicked(send_to_fetch)

# ============================================================
#                         MAIN LOOP
# ============================================================

plt.show()
