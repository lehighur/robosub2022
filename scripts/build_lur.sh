#!/bin/bash

rosdep install -i --from-path src/lur_pkg --rosdistro foxy -y
colcon build --packages-select lur_pkg

echo ""
echo "run '. install/local_setup.bash'"
echo ""
echo "then './scripts/build.sh'"
echo ""
