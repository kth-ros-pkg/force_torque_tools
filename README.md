force_torque_tools
======================


Overview
---------------------------------------------
Tools for gravity compensation and sensor calibration for wrist-mounted force-torque sensors in robot manipulators.


Installing
---------------------------------------------

Make sure you have a working catkin workspace, as described at:
http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

Also make sure you have git installed:

    sudo apt-get install git-core

Change directory to the source folder of your catkin workspace.
If, for instance, your workspace is `~/catkin_ws`, make sure there is
a `src/` folder within it, then execute:

    cd ~/catkin_ws/src

Download the metapackage from the github repository (<ros_distro> may be `groovy` or `hydro`):

    git clone -b <ros_distro> https://github.com/kth-ros-pkg/force_torque_tools.git

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make



force_torque_sensor_calib
---------------------------------------------

This package can calibrate through least-squares the following parameters related to a **wrist-mounted force-torque sensor**:

    Bias of the F/T sensor
    Mass of the gripper
    Location of the center of mass of the attached gripper


Running this software assumes that you have an **accelerometer/imu** already calibrated with respect to the robot manipulator. It also assumes that you have a manipulator previously configured to use **MoveIt!**. 

The software calibrates the F/T sensor by moving the manipulator into a number of different poses and using
the resulting F/T sensor and accelerometer signals.
Calibration can be done by either manually specifying the poses in the parameter server or executing N random poses.

You can look at the configuration/launch files in the `config` and `launch` folders of the force_torque_sensor_calib for examples on how to set the parameters and launch the software for your robot.

### Running on CVAP's Dumbo Robot###

Make sure that the robot is clear of any objects in its surroundings. To calibrate e.g. the left arm run:
    roslaunch force_torque_sensor_calib dumbo_left_arm_ft_calib.launch

You can look at the configuration/launch files in the `config` and `launch` folders of the force_torque_sensor_calib for examples on how to launch this software.

gravity_compensation
---------------------------------------------
Once the F/T sensors have been calibrated using the **force_torque_sensor_calib** package, the yaml file that this package produces can be used as input parameters for gravity compensation.

You can look at the configuration/launch files in the `config` and `launch` folders for examples on how to launch this software with CVAP's Dumbo robot.


References
---------------------------------------------
Equations are based on **"On-line Rigid Object Recognition and Pose Estimation Based on Inertial Parameters"**, D. Kubus, T. Kroger, F. Wahl, IROS 2008



