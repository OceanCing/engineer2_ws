#!/bin/bash

SYS_LOG_DIR="/home/dynamicx/Documents/sys_logs/"
if [[ ! -d $SYS_LOG_DIR ]]; then
    mkdir -p $SYS_LOG_DIR
fi
cd $SYS_LOG_DIR

LOG_FILE="/var/log/syslog"
MAX_FILE_SIZE=5242880000  # 5 GB
MEMORY_LIMIT=60  # 60% memory usage

START_TIME=$(date "+%Y%m%d_%H%M")
OUTPUT_FILE="${SYS_LOG_DIR}${START_TIME}.log"
touch "$OUTPUT_FILE"  # Ensure the file exists to avoid 'stat' errors

# Function to parse log lines to extract the node name and message
function parse_log {
    local node=$(echo "$1" | grep -oP '(?<=:)\s*\/\w+')
    local msg=$(echo "$1" | sed -r 's/[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9]{2}:[0-9]{2}:[0-9]{2}//g' | sed -r 's/[0-9]{2}:[0-9]{2}:[0-9]{2}\.?[0-9]*//g')  # Removing specific timestamp formats
    echo "$node" "$msg"
}

# Function to check if two messages are similar
function is_similar {
    local msg1="$1"
    local msg2="$2"
    if [ "$msg1" == "$msg2" ]; then
        echo "true"
    else
        echo "false"
    fi
}

declare -A last_msg_from_node  # Associative array to store the last message from each node

tail -n 0 -f "$LOG_FILE" | while read line; do
    read node msg <<< $(parse_log "$line")

    # Check memory usage and stop if it's above the threshold
    mem_usage=$(free | grep Mem | awk '{print $3/$2 * 100.0}')
    if (( $(echo "$mem_usage > $MEMORY_LIMIT" | bc -l) )); then
        echo "Memory usage is above the limit, stopping log recording."
        break
    fi

    # Check if the log file size exceeds the maximum allowed size
    if [ $(stat -c %s "$OUTPUT_FILE") -ge $MAX_FILE_SIZE ]; then
        echo "Output file size is above the limit, stopping log recording."
        break
    fi

    # Apply log recording logic based on whether it's a ROS node message or system environment message
    if [[ -n "$node" ]]; then  # If it's a ROS node log
        if [[ ${last_msg_from_node[$node]} && $(is_similar "${last_msg_from_node[$node]}" "$msg") == "true" ]]; then
            continue  # Skip if similar
        fi
        last_msg_from_node[$node]="$msg"  # Update the last message
    fi

    echo "$line" >> "$OUTPUT_FILE"  # Write to file
done