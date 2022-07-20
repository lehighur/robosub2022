#!/bin/bash

# rm -rf build
# rm -rf install

#rosdep install -i --from-path brain/src --rosdistro foxy -y
#rosdep install -i --from-path sub/src --rosdistro foxy -y
#rosdep install -i --from-path test/src --rosdistro foxy -y

rosdep install -i --from-path src --rosdistro foxy -y

colcon build

echo ""
echo "run '. install/local_setup.bash'"
echo "or  '. install/setup.bash'"
echo ""
