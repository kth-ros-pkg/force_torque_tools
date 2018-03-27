/*
 *  ft_calib_node.cpp
 *
 *
 *  Created on: Sep 26, 2012
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <force_torque_sensor_calib/ft_calib.h>
#include <eigen3/Eigen/Core>

using namespace Calibration;


class FTCalibNode
{


public:
	ros::NodeHandle n_;
	ros::AsyncSpinner *spinner;
	ros::Subscriber topicSub_ft_raw_;
	ros::Subscriber topicSub_Accelerometer_;

	FTCalibNode()
	{
		n_ = ros::NodeHandle("~");
		spinner = new ros::AsyncSpinner(1);
		spinner->start();



		topicSub_ft_raw_ = n_.subscribe("ft_raw", 1, &FTCalibNode::topicCallback_ft_raw, this);
		topicSub_Accelerometer_ = n_.subscribe("imu", 1, &FTCalibNode::topicCallback_imu, this);

		m_pose_counter = 0;
		m_ft_counter = 0;

		m_received_ft = false;
		m_received_imu = false;

		m_finished = false;
		m_tf_listener = new tf::TransformListener();

		m_ft_calib = new FTCalib();
	}

	~FTCalibNode()
	{
		saveCalibData();
		delete spinner;
		delete m_group;
		delete m_ft_calib;
		delete m_tf_listener;
	}


	bool getROSParameters()
	{

		// Get the moveit group name
		if(n_.hasParam("moveit_group_name"))
		{
			n_.getParam("moveit_group_name", m_moveit_group_name);
		}

		else
		{
			ROS_ERROR("No moveit_group_name parameter, shutting down node...");
			n_.shutdown();
			return false;
		}

		// Get the name of output calibration file
		if(n_.hasParam("calib_file_name"))
		{
			n_.getParam("calib_file_name", m_calib_file_name);
		}

		else
		{
			ROS_WARN("No calib_file_name parameter, setting to default 'ft_calib.yaml'");
			m_calib_file_name = std::string("ft_calib_data.yaml");
		}

        // Get the name of calibration file directory
        if(n_.hasParam("calib_file_dir"))
        {
            n_.getParam("calib_file_dir", m_calib_file_dir);
        }

        else
        {
            ROS_WARN("No calib_file_dir parameter, setting to default '~/.ros/ft_calib' ");
            m_calib_file_dir = std::string("~/.ros/ft_calib");
        }


        // Get the name of file to store the gravity and F/T measurements
        if(n_.hasParam("meas_file_name"))
        {
            n_.getParam("meas_file_name", m_meas_file_name);
        }

        else
        {
            ROS_WARN("No meas_file_name parameter, setting to default 'ft_calib_meas.txt'");
            m_meas_file_name = std::string("ft_calib_meas.txt");
        }

        // Get the name of directory to save gravity and force-torque measurements
        if(n_.hasParam("meas_file_dir"))
		{
            n_.getParam("meas_file_dir", m_meas_file_dir);
		}

		else
		{
            ROS_WARN("No meas_file_dir parameter, setting to default '~/.ros/ft_calib' ");
            m_meas_file_dir = std::string("~/.ros/ft_calib");
		}

		if(n_.hasParam("poses_frame_id"))
		{
			n_.getParam("poses_frame_id", m_poses_frame_id);
		}

        else
        {
            ROS_ERROR("No poses_frame_id parameter, shutting down node ...");
            n_.shutdown();
            return false;
        }


        // whether the user wants to use random poses
        n_.param("random_poses", m_random_poses, false);

        // number of random poses
        n_.param("number_random_poses", m_number_random_poses, 30);


        // initialize the file with gravity and F/T measurements

        // expand the path
        if (!m_meas_file_dir.empty() && m_meas_file_dir[0] == '~') {
            assert(m_meas_file_dir.size() == 1 or m_meas_file_dir[1] == '/');  // or other error handling
            char const* home = getenv("HOME");
            if (home or (home = getenv("USERPROFILE"))) {
                m_meas_file_dir.replace(0, 1, home);
            }
            else {
                char const *hdrive = getenv("HOMEDRIVE"),
                        *hm_meas_file_dir = getenv("HOMEPATH");
                assert(hdrive);  // or other error handling
                assert(hm_meas_file_dir);
                m_meas_file_dir.replace(0, 1, std::string(hdrive) + hm_meas_file_dir);
            }
        }

        std::ofstream meas_file;
        meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(), std::ios::out);

        std::stringstream meas_file_header;

        meas_file_header << "\% gravity , f/t measurements all expressed in F/T sensor frame\n";
        meas_file_header << "\% [gx, gy, gz, fx, fy, fz, tx, ty, tz]\n";

        meas_file << meas_file_header.str();

        meas_file.close();

        return true;
    }
    // connects to the move arm servers
	void init()
	{
		m_group = new moveit::planning_interface::MoveGroupInterface(m_moveit_group_name);
	}


	// Calibrates the FT sensor by putting the arm in several different positions
	bool moveNextPose()
	{

		std::stringstream ss;
		ss << m_pose_counter;
		Eigen::Matrix<double, 6, 1> pose;

		// either find poses from the parameter server
		// poses should be in "pose%d" format (e.g. pose0, pose1, pose2 ...)
		// and they should be float arrays of size 6
		if(!m_random_poses)
		{
			if(!getPose("pose"+ss.str(), pose))
			{
				ROS_INFO("Finished group %s poses", m_group->getName().c_str());
				m_finished = true;
				return true;
			}

			geometry_msgs::Pose pose_;
			pose_.position.x = pose(0);
			pose_.position.y = pose(1);
			pose_.position.z = pose(2);

			tf::Quaternion q;
			q.setRPY((double)pose(3), (double)pose(4), (double)pose(5));

			tf::quaternionTFToMsg(q, pose_.orientation);

			geometry_msgs::PoseStamped pose_stamped;
			pose_stamped.pose = pose_;
			pose_stamped.header.frame_id = m_poses_frame_id;
			pose_stamped.header.stamp = ros::Time::now();

			m_group->setPoseTarget(pose_stamped);

		}
		else // or execute random poses
		{
			if(m_pose_counter<m_number_random_poses)
			{
				m_group->setRandomTarget();
				ROS_INFO("Executing pose %d",m_pose_counter);
			}

			else
			{
				ROS_INFO("Finished group %s random poses", m_group->getName().c_str());
				m_finished = true;
				return true;
			}
		}


		m_pose_counter++;
		m_group->move();
		ROS_INFO("Finished executing pose %d", m_pose_counter-1);
		return true;
	}

	// gets the next pose from the parameter server
	// pose in [x y z r p y] format ([m], [rad])
	bool getPose(const std::string &pose_param_name, Eigen::Matrix<double, 6, 1> &pose)
	{
		XmlRpc::XmlRpcValue PoseXmlRpc;
		if(n_.hasParam(pose_param_name))
		{
			n_.getParam(pose_param_name, PoseXmlRpc);
		}

		else
		{
			ROS_WARN("Pose parameter %s not found", pose_param_name.c_str());
			return false;
		}

		if(PoseXmlRpc.size()!=6)
		{
			ROS_ERROR("Pose parameter %s wrong size (must be 6)", pose_param_name.c_str());
			return false;
		}

		for(unsigned int i=0; i<6; i++)
			pose(i) = (double)PoseXmlRpc[i];

		return true;
	}

	// prints out the pose (3-D positions) of the calibration frame at each of the positions
	// of the left arm
	void saveCalibData()
	{
		double mass;
		Eigen::Vector3d COM_pos;
		Eigen::Vector3d f_bias;
		Eigen::Vector3d t_bias;

		getCalib(mass, COM_pos, f_bias, t_bias);

		XmlRpc::XmlRpcValue bias;
		bias.setSize(6);
		for(unsigned int i=0; i<3; i++)
			bias[i] = (double)f_bias(i);

		for(unsigned int i=0; i<3; i++)
			bias[i+3] = (double)t_bias(i);

		XmlRpc::XmlRpcValue COM_pose;
		COM_pose.setSize(6);
		for(unsigned int i=0; i<3; i++)
			COM_pose[i] = (double)COM_pos(i);

		for(unsigned int i=0; i<3; i++)
			COM_pose[i+3] = 0.0;

		// set the parameters in the parameter server
		n_.setParam("/ft_calib/bias", bias);
		n_.setParam("/ft_calib/gripper_mass", mass);
		n_.setParam("/ft_calib/gripper_com_frame_id", m_ft_raw.header.frame_id.c_str());
		n_.setParam("/ft_calib/gripper_com_pose", COM_pose);

		// dump the parameters to YAML file
		std::string file = m_calib_file_dir + std::string("/") + m_calib_file_name;

		// first create the directory
		std::string command = std::string("mkdir -p ") + m_calib_file_dir;
		std::system(command.c_str());

		// now dump the yaml file
		command.clear();
		command = std::string("rosparam dump ") + file + std::string(" /ft_calib");
		std::system(command.c_str());
	}

    // saves the gravity and force-torque measurements to a file for postprocessing
    void saveMeasurements(geometry_msgs::Vector3Stamped gravity, geometry_msgs::WrenchStamped ft_meas)
    {
        std::ofstream meas_file;
        meas_file.open((m_meas_file_dir + "/" + m_meas_file_name).c_str(), std::ios::out | std::ios::app);

        std::stringstream meas_file_text;

        meas_file_text << gravity.vector.x << " " << gravity.vector.y << " " << gravity.vector.z << " ";
        meas_file_text << ft_meas.wrench.force.x << " " << ft_meas.wrench.force.y << " " << ft_meas.wrench.force.z << " ";
        meas_file_text << ft_meas.wrench.torque.x << " " << ft_meas.wrench.torque.y << " " << ft_meas.wrench.torque.z << "\n";

        meas_file << meas_file_text.str();

        meas_file.close();
    }

	// finished moving the arm through the poses set in the config file
	bool finished()
	{
		return(m_finished);
	}

	void topicCallback_ft_raw(const geometry_msgs::WrenchStamped::ConstPtr &msg)
	{
		ROS_DEBUG("In ft sensorcallback");
		m_ft_raw = *msg;
		m_received_ft = true;
	}


	// gets readings from accelerometer and transforms them to the FT sensor frame
	void topicCallback_imu(const sensor_msgs::Imu::ConstPtr &msg)
	{
		ROS_DEBUG("In accelerometer read callback");

		m_imu= *msg;
		m_received_imu = true;
	}

	void addMeasurement()
	{

		m_ft_avg.wrench.force.x = -m_ft_avg.wrench.force.x/(double)m_ft_counter;
		m_ft_avg.wrench.force.y = -m_ft_avg.wrench.force.y/(double)m_ft_counter;
		m_ft_avg.wrench.force.z = -m_ft_avg.wrench.force.z/(double)m_ft_counter;

		m_ft_avg.wrench.torque.x = -m_ft_avg.wrench.torque.x/(double)m_ft_counter;
		m_ft_avg.wrench.torque.y = -m_ft_avg.wrench.torque.y/(double)m_ft_counter;
		m_ft_avg.wrench.torque.z = -m_ft_avg.wrench.torque.z/(double)m_ft_counter;


		m_ft_counter = 0;

		if(!m_received_ft)
		{
			ROS_ERROR("Haven't received F/T sensor measurements");
			return;
		}

		if(!m_received_imu)
		{
			ROS_ERROR("Haven't received accelerometer readings");
			return;
		}

		// express gravity vector in F/T sensor frame
		geometry_msgs::Vector3Stamped gravity;
		gravity.header.stamp = ros::Time();
		gravity.header.frame_id = m_imu.header.frame_id;
		gravity.vector = m_imu.linear_acceleration;

		geometry_msgs::Vector3Stamped gravity_ft_frame;

		try
		{
			m_tf_listener->transformVector(m_ft_raw.header.frame_id, gravity, gravity_ft_frame);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("Error transforming accelerometer reading to the F/T sensor frame");
			ROS_ERROR("%s.", ex.what());
			return;
		}

		m_ft_calib->addMeasurement(gravity_ft_frame, m_ft_avg);
        saveMeasurements(gravity_ft_frame, m_ft_avg);
	}

	void getCalib(double &mass, Eigen::Vector3d &COM_pos, Eigen::Vector3d &f_bias, Eigen::Vector3d &t_bias)
	{

		Eigen::VectorXd ft_calib = m_ft_calib->getCalib();

		mass = ft_calib(0);
		if(mass<=0.0)
		{
			ROS_ERROR("Error in estimated mass (<= 0)");
			//		return;
		}

		Eigen::Vector3d center_mass_position(ft_calib(1)/mass,
				ft_calib(2)/mass,
				ft_calib(3)/mass);

		COM_pos = center_mass_position;

		f_bias(0) = -ft_calib(4);
		f_bias(1) = -ft_calib(5);
		f_bias(2) = -ft_calib(6);
		t_bias(0) = -ft_calib(7);
		t_bias(1) = -ft_calib(8);
		t_bias(2) = -ft_calib(9);

	}

	void averageFTMeas()
	{

		if(m_ft_counter==0)
		{

			m_ft_avg = m_ft_raw;

		}

		else
		{

			m_ft_avg.wrench.force.x = m_ft_avg.wrench.force.x + m_ft_raw.wrench.force.x;
			m_ft_avg.wrench.force.y = m_ft_avg.wrench.force.y + m_ft_raw.wrench.force.y;
			m_ft_avg.wrench.force.z = m_ft_avg.wrench.force.z + m_ft_raw.wrench.force.z;

			m_ft_avg.wrench.torque.x = m_ft_avg.wrench.torque.x + m_ft_raw.wrench.torque.x;
			m_ft_avg.wrench.torque.y = m_ft_avg.wrench.torque.y + m_ft_raw.wrench.torque.y;
			m_ft_avg.wrench.torque.z = m_ft_avg.wrench.torque.z + m_ft_raw.wrench.torque.z;

		}
		m_ft_counter++;
	}


private:

	moveit::planning_interface::MoveGroupInterface *m_group;

	unsigned int m_pose_counter;
	unsigned int m_ft_counter;

	bool m_finished;

	bool m_received_ft;
	bool m_received_imu;

	// ft calib stuff
	FTCalib *m_ft_calib;

	// expressed in FT sensor frame
	geometry_msgs::WrenchStamped m_ft_raw;
	geometry_msgs::WrenchStamped m_ft_avg; // average over 100 measurements

	// accelerometer readings
	sensor_msgs::Imu m_imu;

	tf::TransformListener *m_tf_listener;

	//	***** ROS parameters ***** //
	// name of the moveit group
	std::string m_moveit_group_name;

	// name of output calibration file
	std::string m_calib_file_name;

	// name of output directory
	// default: ~/.ros/ft_calib
	std::string m_calib_file_dir;

    // name of file with recorded gravity and F/T measurements
    std::string m_meas_file_name;

    // name of directory for saving gravity and F/T measurements
    // default: ~/.ros/ft_calib
    std::string m_meas_file_dir;

	// frame id of the poses to be executed
	std::string m_poses_frame_id;

	// if the user wants to execute just random poses
	// default: false
	bool m_random_poses;

	// number of random poses
	// default: 30
	int m_number_random_poses;

};

int main(int argc, char **argv)
{
	ros::init (argc, argv, "ft_calib_node");
	ros::NodeHandle nh;

	FTCalibNode ft_calib_node;
	if(!ft_calib_node.getROSParameters())
	{
		ft_calib_node.n_.shutdown();
		ROS_ERROR("Error getting ROS parameters");

	}
	ft_calib_node.init();

	/// main loop
	double loop_rate_;
	ft_calib_node.n_.param("loop_rate", loop_rate_, 650.0);
	ros::Rate loop_rate(loop_rate_); // Hz

	// waiting time after end of each pose to take F/T measurements
	double wait_time;
	ft_calib_node.n_.param("wait_time", wait_time, 4.0);

	bool ret = false;
	unsigned int n_measurements = 0;

	ros::Time t_end_move_arm = ros::Time::now();

	while (ft_calib_node.n_.ok() && !ft_calib_node.finished())
	{

		//		Move the arm, then calibrate sensor
		if(!ret)
		{
			ret = ft_calib_node.moveNextPose();
			t_end_move_arm = ros::Time::now();
		}

		// average 100 measurements to calibrate the sensor in each position
		else if ((ros::Time::now() - t_end_move_arm).toSec() > wait_time)
		{
			n_measurements++;
			ft_calib_node.averageFTMeas(); // average over 100 measurements;

			if(n_measurements==100)
			{
				ret = false;
				n_measurements = 0;

				ft_calib_node.addMeasurement(); // stacks up measurement matrices and FT measurementsa
				double mass;
				Eigen::Vector3d COM_pos;
				Eigen::Vector3d f_bias;
				Eigen::Vector3d t_bias;

				ft_calib_node.getCalib(mass, COM_pos, f_bias, t_bias);
				std::cout << "-------------------------------------------------------------" << std::endl;
				std::cout << "Current calibration estimate:" << std::endl;
				std::cout << std::endl << std::endl;

				std::cout << "Mass: " << mass << std::endl << std::endl;

				std::cout << "Center of mass position (relative to FT sensor frame):" << std::endl;
				std::cout << "[" << COM_pos(0) << ", " << COM_pos(1) << ", " << COM_pos(2) << "]";
				std::cout << std::endl << std::endl;


				std::cout << "FT bias: " << std::endl;
				std::cout << "[" << f_bias(0) << ", " << f_bias(1) << ", " << f_bias(2) << ", ";
				std::cout << t_bias(0) << ", " << t_bias(1) << ", " << t_bias(2) << "]";
				std::cout << std::endl << std::endl;


				std::cout << "-------------------------------------------------------------" << std::endl << std::endl << std::endl;
				ft_calib_node.saveCalibData();
			}

		}


		ros::spinOnce();
		loop_rate.sleep();
	}

	ft_calib_node.saveCalibData();
	ros::shutdown();
	return 0;
}
