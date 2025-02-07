#!/bin/bash

# Ensure script is run as root or with sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root or use sudo."
  exit 1
fi

# Get the directory of the current script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

# Source ROS 2 Humble and local setup files
source "$SCRIPT_DIR/../../../ros2_humble/install/setup.bash"
source "$SCRIPT_DIR/../install/setup.bash"

# Compile the project
cd "$SCRIPT_DIR/../"
colcon build --symlink-install --cmake-args -DPICAS=TRUE -DLATENCY_MGMT=TRUE -DPICAS_THREAD_AFFINITY=TRUE

# Source the local setup file after build
source "$SCRIPT_DIR/../install/setup.bash"

# Run the latency management program and pipe the output to a file
"$SCRIPT_DIR/../build/latency_mgmt/latency_mgmt 1" > "$SCRIPT_DIR/latency_mgmt_output.txt" &
#"$SCRIPT_DIR/build/latency_mgmt/latency_mgmt"

# after some 10 minutes, kill the executor process
sleep 300
pkill -f latency_mgmt

#recompile without latency management or thread affinity and run again
cd "$SCRIPT_DIR/../"
colcon build --symlink-install --cmake-args -DPICAS=TRUE -DLATENCY_MGMT=FALSE -DPICAS_THREAD_AFFINITY=TRUE

source "$SCRIPT_DIR/../install/setup.bash"

"$SCRIPT_DIR/../build/latency_mgmt/latency_mgmt 1" > "$SCRIPT_DIR/picas.txt" &
sleep 300
pkill -f latency_mgmt

# recompile without picas and run again
cd "$SCRIPT_DIR/../"
colcon build --symlink-install --cmake-args -DPICAS=TRUE -DLATENCY_MGMT=FALSE -DPICAS_THREAD_AFFINITY=FALSE
source "$SCRIPT_DIR/../install/setup.bash"

"$SCRIPT_DIR/../build/latency_mgmt/latency_mgmt 1" > "$SCRIPT_DIR/default.txt" &

sleep 300

pkill -f latency_mgmt
