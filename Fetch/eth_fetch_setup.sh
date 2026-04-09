#!/usr/bin/env bash

###############################################
# Fetch ROS Environment Setup (WSL2-friendly)
###############################################

# --- 0) Guard: must be sourced, not executed ---
# (prevents the script from killing your shell on error)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Please run:  source ${BASH_SOURCE[0]}"
  exit 1
fi

# --- 0.5) Guard: must be Linux/WSL (not Git Bash / Windows) ---
if [[ "$(uname -s)" != "Linux" ]]; then
  echo "ERROR: This script must be run in Linux (WSL/Ubuntu) with ROS installed."
  echo "You're currently in: $(uname -s)"
  echo ""
  echo "Fix: open Ubuntu (WSL), then run:"
  echo "  source /path/to/eth_fetch_setup.sh"
  return 1
fi

# Detect WSL
IS_WSL=0
if grep -qi microsoft /proc/version 2>/dev/null || [[ -n "${WSL_DISTRO_NAME:-}" ]]; then
  IS_WSL=1
fi

# --- 1) Base ROS ---
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
else
  echo "ERROR: ROS Noetic not found at /opt/ros/noetic"
  echo "Install ROS Noetic inside Ubuntu/WSL first."
  return 1
fi

# --- 2) Overlay your personal workspace ---
WS_PATH="$HOME/Fetch/personal_ws/devel/setup.bash"
if [ -f "$WS_PATH" ]; then
  source "$WS_PATH"
else
  echo "ERROR: $WS_PATH not found."
  echo "Run:"
  echo "  cd \$HOME/Fetch/personal_ws && catkin_make"
  return 1
fi

# --- 3) Connect to Fetch master (qiborg) ---
export ROS_MASTER_URI="http://qiborg.cs.byu.edu:11311"

# --- 4) Set YOUR machine IP (WSL2 needs Windows host IP ideally) ---
get_linux_ip() {
  ip route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") print $(i+1)}' | head -n1
}

get_windows_host_ip() {
  # Pick the Windows interface used for default route, then get its IPv4
  local ifindex
  ifindex=$(powershell.exe -NoProfile -Command \
    "(Get-NetRoute -DestinationPrefix '0.0.0.0/0' | Sort-Object RouteMetric | Select-Object -First 1 -ExpandProperty ifIndex)" \
    2>/dev/null | tr -d '\r')

  if [[ -z "$ifindex" ]]; then
    return 1
  fi

  powershell.exe -NoProfile -Command \
    "(Get-NetIPAddress -InterfaceIndex $ifindex -AddressFamily IPv4 | Where-Object { \$_.IPAddress -notlike '169.254*' } | Select-Object -First 1 -ExpandProperty IPAddress)" \
    2>/dev/null | tr -d '\r'
}

MY_IP=""

if [[ "$IS_WSL" -eq 1 ]]; then
  # Prefer Windows host LAN IP so robot can reach you if needed
  MY_IP="$(get_windows_host_ip)"
fi

# Fallback to Linux/WSL interface IP
if [[ -z "$MY_IP" ]]; then
  MY_IP="$(get_linux_ip)"
fi

if [[ -z "$MY_IP" ]]; then
  echo "ERROR: Could not determine local IP. Are you connected to a network?"
  return 1
fi

export ROS_IP="$MY_IP"
export ROS_HOSTNAME="$MY_IP"

echo ""
echo "==========================================="
echo " Fetch environment configured!"
echo " ROS_MASTER_URI = $ROS_MASTER_URI"
echo " ROS_IP         = $ROS_IP"
echo " ROS_HOSTNAME   = $ROS_HOSTNAME"
echo " WSL            = $IS_WSL"
echo "==========================================="
echo ""
echo "Try:"
echo "  rostopic list | head"
echo "  rostopic echo -n 1 /undock/status"
echo ""