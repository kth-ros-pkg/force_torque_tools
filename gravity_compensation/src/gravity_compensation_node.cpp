/*
 *  gravity_compensation.cpp
 *
 *  Created on: Nov 12, 2013
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2013, Francisco Viña, CVAP, KTH
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
#include <gravity_compensation/gravity_compensation.h>
#include <gravity_compensation/gravity_compensation_params.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread.hpp>


class GravityCompensationNode
{
public:
	ros::NodeHandle n_;

	// subscribe to accelerometer (imu) readings
	ros::Subscriber topicSub_imu_;
	ros::Subscriber topicSub_ft_raw_;

	ros::Publisher topicPub_ft_zeroed_;
	ros::Publisher topicPub_ft_compensated_;

	tf::TransformBroadcaster tf_br_;


	GravityCompensationNode()
	{
		n_ = ros::NodeHandle("~");
		m_g_comp_params  = new GravityCompensationParams();
		m_g_comp = NULL;

		// subscribe to accelerometer topic and raw F/T sensor topic
		topicSub_imu_ = n_.subscribe("imu", 1, &GravityCompensationNode::topicCallback_imu, this);
		topicSub_ft_raw_ = n_.subscribe("ft_raw", 1, &GravityCompensationNode::topicCallback_ft_raw, this);

		/// implementation of topics to publish
		topicPub_ft_zeroed_ = n_.advertise<geometry_msgs::WrenchStamped> ("ft_zeroed", 1);
		topicPub_ft_compensated_ = n_.advertise<geometry_msgs::WrenchStamped> ("ft_compensated", 1);
	}

	~GravityCompensationNode()
	{
		delete m_g_comp;
		delete m_g_comp_params;
	}

	bool getROSParameters()
	{
		/// Get F/T sensor bias
		XmlRpc::XmlRpcValue biasXmlRpc;
		Eigen::Matrix<double, 6, 1> bias;
		if (n_.hasParam("bias"))
		{
			n_.getParam("bias", biasXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter 'bias' not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		if(biasXmlRpc.size()!=6)
		{
			ROS_ERROR("Invalid F/T bias parameter size (should be size 6), shutting down node");
			n_.shutdown();
			return false;
		}

		for (int i = 0; i < biasXmlRpc.size(); i++)
		{
			bias(i) = (double)biasXmlRpc[i];
		}


		// get the mass of the gripper
		double gripper_mass;
		if (n_.hasParam("gripper_mass"))
		{
			n_.getParam("gripper_mass", gripper_mass);
		}

		else
		{
			ROS_ERROR("Parameter 'gripper_mass' not available");
			n_.shutdown();
			return false;
		}

		if(gripper_mass<0.0)
		{
			ROS_ERROR("Parameter 'gripper_mass' < 0");
			n_.shutdown();
			return false;
		}

		// get the pose of the COM of the gripper
		// we assume that it is fixed with respect to FT sensor frame
		// first get the frame ID
		tf::StampedTransform gripper_com;
		std::string gripper_com_frame_id;
		if (n_.hasParam("gripper_com_frame_id"))
		{
			n_.getParam("gripper_com_frame_id", gripper_com_frame_id);
		}

		else
		{
			ROS_ERROR("Parameter 'gripper_com_frame_id' not available");
			n_.shutdown();
			return false;
		}

		gripper_com.frame_id_ = gripper_com_frame_id;

		// now get the CHILD frame ID
		std::string gripper_com_child_frame_id;
		if (n_.hasParam("gripper_com_child_frame_id"))
		{
			n_.getParam("gripper_com_child_frame_id", gripper_com_child_frame_id);
		}

		else
		{
			ROS_ERROR("Parameter 'gripper_com_child_frame_id' not available");
			n_.shutdown();
			return false;
		}

		gripper_com.child_frame_id_ = gripper_com_child_frame_id;

		// now get the actual gripper COM pose
		Eigen::Matrix<double, 6, 1> gripper_com_pose;
		XmlRpc::XmlRpcValue gripper_com_pose_XmlRpc;
		if (n_.hasParam("gripper_com_pose"))
		{
			n_.getParam("gripper_com_pose", gripper_com_pose_XmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter 'gripper_com_pose' not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		if(gripper_com_pose_XmlRpc.size()!=6)
		{
			ROS_ERROR("Invalid 'gripper_com_pose' parameter size (should be size 6), shutting down node");
			n_.shutdown();
			return false;
		}

		for(unsigned int i=0; i<gripper_com_pose_XmlRpc.size(); i++)
		{
			gripper_com_pose(i) = gripper_com_pose_XmlRpc[i];
		}

		tf::Vector3 p;
		tf::vectorEigenToTF(gripper_com_pose.topRows(3), p);
		tf::Quaternion q;
		q.setRPY((double)gripper_com_pose(3),
				(double)gripper_com_pose(4),
				(double)gripper_com_pose(5));

		gripper_com = tf::Transform(q, p);


		n_.param("gripper_com_broadcast_frequency",
				m_gripper_com_broadcast_frequency, 100.0);

		m_g_comp_params->setBias(bias);
		m_g_comp_params->setGripperMass(gripper_mass);
		m_g_comp_params->setGripperCOM(gripper_com);

		m_g_comp = new GravityCompensation(m_g_comp_params);
		return true;
	}

	void topicCallback_imu(const sensor_msgs::Imu::ConstPtr &msg)
	{
		m_imu = *msg;
	}

	void topicCallback_ft_raw(const geometry_msgs::WrenchStamped::ConstPtr &msg)
	{
		geometry_msgs::WrenchStamped ft_zeroed;
		m_g_comp->Zero(*msg, ft_zeroed);
		topicPub_ft_zeroed_.publish(ft_zeroed);


		geometry_msgs::WrenchStamped ft_compensated;
		m_g_comp->Compensate(ft_zeroed, m_imu, ft_compensated);
		topicPub_ft_compensated_.publish(ft_compensated);
	}

	void publish_gripper_com_tf()
	{

		try
		{
			ROS_DEBUG("Publishing gripper COM tf");
			tf::StampedTransform gripper_com = m_g_comp_params->getGripperCOM();
			gripper_com.stamp_ = ros::Time::now();
			tf_br_.sendTransform(gripper_com);
			boost::this_thread::sleep(boost::posix_time::milliseconds((1/m_gripper_com_broadcast_frequency)*1000));
		}

		catch(boost::thread_interrupted&)
		{
			return;
		}

		if(!ros::ok())
		{
			return;
		}
	}

private:

	GravityCompensationParams *m_g_comp_params;
	GravityCompensation *m_g_comp;
	sensor_msgs::Imu m_imu;
	double m_gripper_com_broadcast_frequency;

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "gravity_compensation");
	GravityCompensationNode g_comp_node;

	if(!g_comp_node.getROSParameters())
	{
		ROS_ERROR("Error getting ROS parameters");
		return 0;
	}

	// loop frequency
	double loop_frequency;
	g_comp_node.n_.param("loop_frequency", loop_frequency, 1000.0);
	ros::Rate loop_rate(loop_frequency);

	// add a thread for publishing the
	boost::thread t_tf(boost::bind(&GravityCompensationNode::publish_gripper_com_tf, &g_comp_node));

	ros::AsyncSpinner s(2);
	s.start();

	while(ros::ok())
	{
		loop_rate.sleep();
	}

	t_tf.join();


	return 0;
}

