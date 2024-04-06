#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <number of times to launch>"
    exit 1
fi

# Extract the number of times to launch
num_launches=$1

# Loop to launch the command x times
for ((i = 1; i <= num_launches; i++)); do
    ros2 launch rpi5_radar_visual ros2_rpi5_radar_client.yaml &
    wait $!
done