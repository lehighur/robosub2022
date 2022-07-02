#!/bin/bash

ros2 run brain state_machine \
&& ros2 run sub movement \
&& ros2 run mavros mavros_node --ros-args --params-file mavros_param.yaml
