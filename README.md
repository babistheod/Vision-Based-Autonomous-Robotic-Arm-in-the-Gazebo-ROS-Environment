# Steps to use the pkg

Before following the instructions, please ensure that ROS workspace is configured and build properly. 

Build the directory before running, please build the workspace to auto-generate the custom messages and services

`catkin_make    # in ROS workspace`   


1. Run the UR robot and Gazebo utilities by:

    `roslaunch ur5_pick_place_gazebo bringup_ur5_gazebo.launch`

2. Run the task helper services to accomplised the task in other terminal  

    `roslaunch ur5_pick_place_gazebo task_services.launch `

3. Now, run the main node to execute the pick and place task to build a stack in other terminal

    `rosrun ur5_pick_place_gazebo state_machine.py`
    
4. To view the FSM

   rosrun smach_viewer smach_viewer.py

