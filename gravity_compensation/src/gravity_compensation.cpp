/*
 *  gravity_compensation.cpp
 *
 *  Created on: Nov 11, 2013
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
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>


GravityCompensation::GravityCompensation(GravityCompensationParams *g_comp_params)
{
    m_g_comp_params = g_comp_params;
    m_tf_listener = new tf::TransformListener();
}

GravityCompensation::~GravityCompensation()
{
    delete m_tf_listener;
}


void GravityCompensation::Zero(const geometry_msgs::WrenchStamped &ft_raw,
                          geometry_msgs::WrenchStamped &ft_zeroed)
{
    std::vector<double> bias = m_g_comp_params->getBias();

    ft_zeroed = ft_raw;
    ft_zeroed.wrench.force.x = ft_raw.wrench.force.x-bias[0];
    ft_zeroed.wrench.force.y = ft_raw.wrench.force.y-bias[1];
    ft_zeroed.wrench.force.z = ft_raw.wrench.force.z-bias[2];
    ft_zeroed.wrench.torque.x = ft_raw.wrench.torque.x-bias[3];
    ft_zeroed.wrench.torque.y = ft_raw.wrench.torque.y-bias[4];
    ft_zeroed.wrench.torque.z = ft_raw.wrench.torque.z-bias[5];

    return ft_zeroed;
}


bool GravityCompensation::Compensate(const geometry_msgs::WrenchStamped &ft_zeroed,
                                     const sensor_msgs::Imu &gravity,
                                     geometry_msgs::WrenchStamped &ft_compensated)
{

    geometry_msgs::Vector3Stamped g;
    g.vector = gravity.linear_acceleration;
    g.header = gravity.header;
    g.header.stamp = ros::Time();

    // convert the accelerometer reading to the F/T sensor frame
    geometry_msgs::Vector3Stamped g_ft_frame;
    try
    {
        m_tf_listener->transformVector(ft_zeroed.header.frame_id, g, g_ft_frame);
    }

    catch(tf::TransformException &ex)
    {
        ROS_ERROR("Error transforming gravity vector to ft sensor frame...");
        ROS_ERROR("%s.", ex.what());
        return false;
    }

    geometry_msgs::WrenchStamped gripper_wrench;
    gripper_wrench.header.frame_id = ft_zeroed.header.frame_id;
    double gripper_mass = m_g_comp_params->getGripperMass();

    gripper_wrench.wrench.force.x = g_ft_frame.vector.x * gripper_mass;
    gripper_wrench.wrench.force.y = g_ft_frame.vector.y * gripper_mass;
    gripper_wrench.wrench.force.z = g_ft_frame.vector.z * gripper_mass;


    tf::StampedTransform gripper_com = m_g_comp_params->getGripperCOM();
    geometry_msgs::PoseStamped gripper_com_pose;
    tf::poseTFToMsg(gripper_com, gripper_com_pose.pose);
    gripper_com_pose.header.frame_id = gripper_com.child_frame_id_;
    gripper_com_pose.header.stamp = ros::Time();

    // make sure the gripper COM is expressed with respect to the F/T sensor frame
    geometry_msgs::PoseStamped ft_gripper_com_pose;

    try
    {
        m_tf_listener->transformPose(ft_zeroed.header.frame_id,
                                     gripper_com_pose,
                                     ft_gripper_com_pose);
    }

    catch(tf::TransformException &ex)
    {
        ROS_ERROR("Error looking up transform between the gripper COM and the ft sensor frame");
        ROS_ERROR("%s.", ex.what());
        return false;
    }

    // convert the geometry_msgs::PoseStamped to tf::Transform
    tf::Transform T_ft_gripper_com;
    tf::poseMsgToTF(ft_gripper_com_pose.pose, T_ft_gripper_com);


    // convert to Eigen to do cross product to compute the torque
    Eigen::Vector3d r;
    tf::pointMsgToEigen(ft_gripper_com_pose.pose.position, r);

    Eigen::Vector3d gripper_weight_eigen;
    tf::vectorMsgToEigen(gripper_wrench.wrench.force, gripper_weight_eigen);

    // compute torque generated by weight of the gripper
    Eigen::Vector3d gripper_torque_eigen = r.cross(gripper_weight_eigen);
    tf::vectorEigenToMsg(gripper_torque_eigen, gripper_wrench.wrench.torque);

    // compensate force+torque
    Eigen::Matrix<double, 6, 1> ft_zeroed_eigen;
    Eigen::Matrix<double, 6, 1> ft_compensated_eigen;

    tf::wrenchMsgToEigen(ft_zeroed.wrench, ft_zeroed_eigen);

    ft_compensated_eigen =

    // compensated FT in FT sensor frame
    geometry_msgs::Vector3Stamped FT_Fvec_comp;
    geometry_msgs::Vector3Stamped FT_Tvec_comp;
    FT_Fvec_comp.header.frame_id = EndEffector_Weight.header.frame_id;
    FT_Tvec_comp.header.frame_id = EndEffector_Weight.header.frame_id;
    FT_Fvec_comp.header.stamp = ros::Time();
    FT_Tvec_comp.header.stamp = ros::Time();

    FT_Fvec_comp.vector.x = FT_raw.wrench.force.x - EndEffector_Weight.wrench.force.x;
    FT_Fvec_comp.vector.y = FT_raw.wrench.force.y - EndEffector_Weight.wrench.force.y;
    FT_Fvec_comp.vector.z = FT_raw.wrench.force.z - EndEffector_Weight.wrench.force.z;

    FT_Tvec_comp.vector.x = FT_raw.wrench.torque.x - EndEffector_Weight.wrench.torque.x;
    FT_Tvec_comp.vector.y = FT_raw.wrench.torque.y - EndEffector_Weight.wrench.torque.y;
    FT_Tvec_comp.vector.z = FT_raw.wrench.torque.z - EndEffector_Weight.wrench.torque.z;


    // now expressed in the FT frame
    FT_comp.wrench.force = FT_Fvec_comp.vector;
    FT_comp.wrench.torque = FT_Tvec_comp.vector;
    FT_comp.header.stamp = FT_raw.header.stamp;
    FT_comp.header.frame_id = FT_raw.header.frame_id;

    return true;


    return ft_compensated;
}
