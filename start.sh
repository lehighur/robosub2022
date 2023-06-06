#!/bin/bash

ros2 launch launch_files/main.yaml &
sleep 5
ros2 run mavros mav sys mode -c STABILIZE &
sleep 5 
ros2 launch launch_files/test.yaml
