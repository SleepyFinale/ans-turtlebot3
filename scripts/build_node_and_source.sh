echo "Building workspace (colcon build --packages-select turtlebot3_node --symlink-install --parallel-workers 1)..."
colcon build --packages-select turtlebot3_node --symlink-install --parallel-workers 1
echo "sourcing install/setup.bash"
source install/setup.bash
