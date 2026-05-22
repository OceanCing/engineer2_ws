#!/bin/bash

target_ip="192.168.100.2/24"
restarted=false

while true; do
    if ! ip addr show | grep -q -w "$target_ip"; then
        if [ "$restarted" = false ]; then
            echo "IP $target_ip not found."
            echo "Restarting services: rm_ecat_start.service, start_master.service, vision_start_service..."
            sudo systemctl restart rm_ecat_start.service start_master.service vision_start.service
            echo "Services restarted successfully.Exiting."
            restarted=true
            break
        fi
    else
        echo "IP $target_ip exists. No action needed."
    fi
    sleep 20
done



