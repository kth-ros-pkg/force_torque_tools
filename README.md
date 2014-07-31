force_torque_tools
======================


Overview
---------------------------------------------
Tools for gravity compensation and sensor calibration for wrist-mounted force-torque sensors in robot manipulators.

* force_torque_sensor_calib: calibrates force-torque sensor bias, gripper mass and center of mass of the gripper.
* gravity_compensation: performs gravity compensation to force-torque sensor measurements after calibrating with the  **force_torque_sensor_calib** package.


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

### Overview ###

This package can calibrate through least-squares the following parameters related to a **wrist-mounted force-torque sensor**:

    Bias of the F/T sensor
    Mass of the gripper
    Location of the center of mass of the attached gripper


Running this software assumes that you have an **accelerometer/imu** whose reference frame is already calibrated with respect to the e.g. base frame of the robot manipulator. It also assumes that you have a manipulator previously configured to be controlled through **MoveIt!**. 

The software calibrates the F/T sensor by moving the manipulator into a number of different poses and using
the resulting F/T sensor and accelerometer signals for computing a least-squares estimate of the parameters.
Calibration can be done by either manually specifying explicitly the manipulator poses in the parameter server (parameters **pose0, pose1, pose2, ... poseN**) or by executing N random poses.


### Running the calibration node ###
Make sure that the robot is still and other objects don't obstruct the arms while they move into the calibration poses.

You can look at the configuration/launch files in the `config` and `launch` folders for examples on how to set the parameters and launch the software for your robot. For more details on the parameters required to launch the calibration node visit the [ROS wiki page] [1] of the **force_torque_sensor_calib** package and the [tutorials page] [2].

The calibration software will produce a **yaml** calibration file that can later be used for gravity compensation. By default the file will be written in the **~/.ros/ft_calib/** directory.

[1]: http://wiki.ros.org/force_torque_sensor_calib
[2]: http://wiki.ros.org/force_torque_tools/Tutorials


gravity_compensation
---------------------------------------------
Compensates gravity forces measured by a force-torque sensor.
Uses the **yaml** calibration file produced by the **force_torque_sensor_calib** package and gravity measurements given by an accelerometer whose reference frame has been previously calibrated to the frame of the robot.

You can look at the configuration/launch files in the `config` and `launch` folders for examples on how to launch this software.

For more details on running the software visit the package's [ROS wiki page] [3] and the [tutorials page] [2].
[3]: http://wiki.ros.org/gravity_compensation


References
---------------------------------------------
Equations are based on **"On-line Rigid Object Recognition and Pose Estimation Based on Inertial Parameters"**, D. Kubus, T. Kroger, F. Wahl, IROS 2008



