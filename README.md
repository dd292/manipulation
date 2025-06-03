To build the docker 
docker build -t dual_franka_sim:latest .

To start the docker 

docker run -it \
  --gpus all \
  --name dual_franka_sim_dev \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_projects:/root/ros2_ws/src \
  dual_franka_sim:latest

  docker run -it --rm dual_franka_sim:latest

To remove an old container 

docker rm dual_franka_sim_dev 

  To change the permission of the docker mounted directory 

  sudo chown -R $USER$:$USER$ ~/ros2_projects

To build the package
cd ~/ros2_ws 
rm -rf build install log # to clean the previous build
source /opt/ros/foxy/setup.bash 
colcon build --packages-select dual_franka_model
source install/setup.bash
ros2 launch dual_franka_model sim.launch.py

Some helpful ros commands

ros2 node list
ros2 topic list
ros2 topic echo /joint_state