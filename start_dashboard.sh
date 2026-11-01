#!/bin/bash
# ==========================================================
# AMR Delivery Robot - Dashboard Service Runner
# ==========================================================

# 1. Source ROS 2 Foxy
source /opt/ros/foxy/setup.bash

# 2. Source Workspace
WORKSPACE_DIR="/home/jetson/deliver_amr"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# 3. Environment ROS 2 & CycloneDDS Tailscale (Domain 0)
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$WORKSPACE_DIR/src/amr_simulation/config/cyclonedds_jetson.xml"

# 4. Jalankan Dashboard Studio (Flask + Waypoint Manager)
exec ros2 launch amr_dashboard dashboard.launch.py mode:=real use_sim_time:=False
