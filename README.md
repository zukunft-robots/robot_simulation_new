# Robot_simulation

This project focuses on developing a graphical user interface (GUI) to control a simulated robot for both remote control and autonomous navigation tasks.

# Install the packages

- **[ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)**
- **Teleop_twist_keyboard**

        sudo apt install ros-humble-teleop_twist_keyboard
- **NAV2 Stack**

        sudo apt install ros-humble-navigation2

        sudo apt install ros-humble-nav2-bringup

- **Slam-toolbox**

        sudo apt install ros-humble-slam-toolbox

- **PYQT5**

        pip3 install pyQt5


# Create a workspace

        mkdir -p ~/sim_ws/src

# Clone the repository

        cd sim_ws/src

        git clone https://github.com/zukunft-robots/robot_simulation_new.git

# Build and Source the workspace

        cd sim_ws

        colcon build

        source install/setup.bash

# Run the GUI

        python3 src/robot_simulation_new/gui.py

# Remote Control

Once you  run the python file , the ```GUI``` will be displayed.

- Press `Remote Mode` button.
- Press `Spawn robot` button to open gazebo and launch the robot.
- Press `Open Rviz2` button to visualize the robot.
- Press `Start Teleop` button to open a terminal to control the robot using keyboard.

        TELEOPERATION
        ---------------------------
        Moving around:
        u    i    o
        j    k    l
        m    ,    .

        anything else : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit
 
# Mapping

- Press `Autonomous Mode` button
- Press `Spawn robot` button to open gazebo and launch the robot.
- Press `Open Rviz2` button to visualize the robot.
- Press `Start Teleop` button to open a terminal to control the robot using keyboard.
- Press `Mapping` button.
- Press `Start Mapping` button to map the environment


        TELEOPERATION
        ---------------------------
        Moving around:
        u    i    o
        j    k    l
        m    ,    .

        anything else : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        
- Once you map the complete environment, Press `Save Map` button.

# Autonomous Navigation

- Press `Autonomous Mode` button
- Press `Spawn robot` button to open gazebo and launch the robot.
- Press `Open Rviz2` button to visualize the robot.
- Press `Autonomous Navigation` button.
- Select the map using `Select Map` button.
- Now Press `Start navigation`.
- Now provide the goal to reach for the robot using `2D-Goal-pose` icon on Rviz2.
- Now the robot will reach the targeted position.

