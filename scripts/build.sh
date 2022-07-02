#!/bin/bash

# rm -rf build

rosdep install -i --from-path brain/src --rosdistro foxy -y
rosdep install -i --from-path sub/src --rosdistro foxy -y

colcon build

echo ""
echo "run '. install/local_setup.bash'"
echo ""
