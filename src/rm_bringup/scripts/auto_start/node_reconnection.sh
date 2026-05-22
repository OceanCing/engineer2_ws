#!/bin/bash
source /opt/ros/noetic/setup.bash
sudo chown dynamicx:dynamicx "$0"
source /home/dynamicx/rm_ws/devel/setup.bash
source /home/dynamicx/.bashrc

declare -A nodes=(
#    ["/node_name"]="package node_executable"
    ["/rm_dbus"]="rm_dbus rm_dbus"
    ["/rm_referee"]="rm_referee rm_referee"
)

check_and_restart_node() {
    local node=$1
    local package_exec=($2)
    local package=${package_exec[0]}
    local executable=${package_exec[1]}

    if ! rosnode list | grep -q $node; then
        echo "node ${node} not run,trying to restart..."
        /opt/ros/noetic/bin/rosrun $package $executable &
        echo "Executing $package $executable..."
    fi
}

while true; do
    for node in "${!nodes[@]}"; do
        check_and_restart_node $node "${nodes[$node]}"
    done
    sleep 3
done

