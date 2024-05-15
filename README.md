# li-slam
A package to investigate lidar-inhibited SLAM narrowing the FOV of a 2D lidar with slam_toolbox

### How to use
1. Build the package and ensure that the packages turtlebot3, turtlebot3-msgs, and all dependencies are installed and build on the humble-devel branch.
2. Source the workspace
3. Export the robot type using the command `export TURTLEBOT3_MODEL=<type>` where `<type>` can be burger or waffle.
4. Run the command `ros2 launch turtlebot_control launch_sim.launch.xml` to launch Gazebo classic, the intercept node, and Rviz. 
5. If you would like to move the robot, run teleop_twist_keyboard. Autonomous exploration coming soon.