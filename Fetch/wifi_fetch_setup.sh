#!/usr/bin/env bash
# ROS environment setup on tin to talk to Fetch over Wi-Fi AP
# USAGE:  source ~/wifi_fetch_env.sh

#set -e

# IP of qiborg's hotspot (from fetch_wifi_ap.sh output)
FETCH_AP_IP="10.42.0.1"

# Source ROS + your workspace
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "ERROR: /opt/ros/noetic/setup.bash not found."
    return 1 2>/dev/null || exit 1
fi

if [ -f "$HOME/Fetch/personal_ws/devel/setup.bash" ]; then
    source "$HOME/Fetch/personal_ws/devel/setup.bash"
else
    echo "WARNING: $HOME/Fetch/personal_ws/devel/setup.bash not found."
    echo "         Did you run catkin_make in personal_ws?"
fi

# Find tin's IP used to reach the Fetch AP
MY_IP=$(ip -4 route get "$FETCH_AP_IP" 2>/dev/null | awk '/src/ {for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); break}}')

if [ -z "$MY_IP" ]; then
    echo "ERROR: Could not determine local IP to reach $FETCH_AP_IP."
    echo "       Are you connected to the 'FetchBot' Wi-Fi?"
    return 1 2>/dev/null || exit 1
fi

export ROS_MASTER_URI="http://$FETCH_AP_IP:11311"
export ROS_IP="$MY_IP"
export ROS_HOSTNAME="$MY_IP"

echo "==> ROS environment on tin set:"
echo "    ROS_MASTER_URI = $ROS_MASTER_URI"
echo "    ROS_IP         = $ROS_IP"
echo "    ROS_HOSTNAME   = $ROS_HOSTNAME"
echo ""
echo "Try:  rostopic list"
