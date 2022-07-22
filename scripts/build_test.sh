#!/bin/bash

rosdep install -i --from-path src/test --rosdistro foxy -y
colcon build --packages-select test

echo ""
echo "run '. install/local_setup.bash'"
echo ""
echo "then './scripts/build.sh'"
echo ""
