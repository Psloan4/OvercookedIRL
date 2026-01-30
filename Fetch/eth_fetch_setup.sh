#!/usr/bin/env bash
#set -e

###############################################
# Fetch ROS Environment Setup
###############################################

# --- 1) Base ROS ---
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "ERROR: ROS Noetic not found at /opt/ros/noetic"
    return 1 2>/dev/null || exit 1
fi

# --- 2) Overlay your personal workspace ---
if [ -f "$HOME/Fetch/personal_ws/devel/setup.bash" ]; then
    source "$HOME/Fetch/personal_ws/devel/setup.bash"
else
    echo "ERROR: $HOME/Fetch/personal_ws/devel/setup.bash not found."
    echo "Run: catkin_make inside personal_ws first."
    return 1 2>/dev/null || exit 1
fi

# --- 3) Connect to Fetch master (qiborg) ---
export ROS_MASTER_URI=http://qiborg.cs.byu.edu:11311

# --- 4) Set YOUR machine's IP (from your ifconfig) ---
# --- Set YOUR machine's IP dynamically (works with eth OR wifi) ---
MY_IP=$(ip route get 1.1.1.1 | awk '{for (i=1;i<=NF;i++) if ($i=="src") print $(i+1)}' | head -n1)

if [ -z "$MY_IP" ]; then
    echo "ERROR: Could not determine local IP. Are you connected to any network?"
else
    export ROS_IP="$MY_IP"
    export ROS_HOSTNAME="$MY_IP"
fi

export ROS_MASTER_URI=http://qiborg.cs.byu.edu:11311


###############################################
echo ""
echo "==========================================="
echo " Fetch environment configured!"
echo " ROS_MASTER_URI = $ROS_MASTER_URI"
echo " ROS_IP         = $ROS_IP"
echo " ROS_HOSTNAME   = $ROS_HOSTNAME"
echo "==========================================="
echo ""
echo "You can now run:"
echo "  rostopic list"
echo "  roslaunch test_pkg my_robot.launch"
echo ""
