^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package force_torque_sensor_calib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2018-03-27)
------------------
* [fix] Updated deprecated moveit headers in force_torque_sensor_calib
* [feature] Set RRT Connect as default planner
* Contributors: Diogo Almeida

1.1.0 (2016-01-25)
------------------
* [sys] Updated for jade and k-turtle eigen dependency (`#12 <https://github.com/kth-ros-pkg/force_torque_tools/pull/12>`_ and discussion `#11 <https://github.com/kth-ros-pkg/force_torque_tools/issues/11>`_)
* Contributors: Aris Synodinos

1.0.2 (2015-08-17)
------------------
* Add missing install rules
* Contributors: Isaac IY Saito

1.0.1 (2015-05-17)
------------------
* Compilable setting on ROS Indigo.
* fixed yaml files
* added saving of gravity and force-torque measurements to text file
* fixed launch file to accept arguments
  put an example calibration data file in the force_torque_sensor_calib package
* added example config/launch files
  updated readme with links to ros wiki tutorials
  changed default value of calib_file_name parameter in ft_calib_node
* removing gripper_com_child_frame_id parameter from ft calib
  gravity compensation fixed and working
* ft_sensor calib node tested and working
* added poses_frame_id to parameters
* minor fixes
* fixed launch and config files for dumbo ft sensor calib
  fixed some bugs in ft_calib_node
* debugging with moveit
* added force_torque_sensor_calib package
* Contributors: Francisco Vina, Isaac IY Saito, fevb
