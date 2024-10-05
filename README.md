# Grasp_Planning
Project repo for Vision Based Manipulation course @WPI

Package Details:


1) Build the src
   ```bash
   colcon build --symlink-install

2) Launch the Gazebo:
   ```bash
   ros2 launch vbm_project_env simulation.launch.py

3) Run RVIZ:
   ```bash
   ros2 run rviz2 rviz2

4) Launch Grasp Pose Generator:
   ```bash
   ros2 launch grasp_pose_generator grasp_pose_generator_launch.py 

5) Start the Grasp Generation Service:
   ```bash
   ros2 run grasp_gen_service grasp_gen_service

6) Service call:
   ```bash
   ros2 service call /gen_gr_grasp grasp_gen_interface/srv/GraspGen input:\ 'generate_grasp'\ 
   
   
