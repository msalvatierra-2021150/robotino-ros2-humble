#!/bin/bash

PI_USER="robotino"
PI_IP="192.168.0.100"

LAPTOP_TIME="$(date +"%Y-%m-%d %H:%M:%S")"

echo "Syncing Pi time to laptop time: $LAPTOP_TIME"
ssh -t ${PI_USER}@${PI_IP} "sudo timedatectl set-ntp false && sudo date -s '$LAPTOP_TIME'"

echo "Restarting LiDAR service..."
ssh -t ${PI_USER}@${PI_IP} "sudo systemctl restart slider.service"

echo "Done. Check /scan timestamp now:"
echo "ros2 topic echo /scan --once | grep -A 4 stamp"
echo "date +%s"