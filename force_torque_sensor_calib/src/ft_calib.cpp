/*
 *  ft_calib.cpp
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
#include <force_torque_sensor_calib/ft_calib.h>
#include <eigen3/Eigen/Dense>
#include <kdl/frameacc.hpp>
#include <kdl/frames.hpp>

namespace Calibration{

FTCalib::FTCalib()
{
	m_num_meas = 0;
}

FTCalib::~FTCalib(){

}

void FTCalib::addMeasurement(const geometry_msgs::Vector3Stamped &gravity,
		const geometry_msgs::WrenchStamped &ft_raw)
{
	if(gravity.header.frame_id != ft_raw.header.frame_id)
	{
		ROS_ERROR("Gravity vector and ft raw expressed in different frames (%s, %s)!",
				gravity.header.frame_id.c_str(), ft_raw.header.frame_id.c_str());
		return;
	}

	m_num_meas++;

	Eigen::MatrixXd h = GetMeasurementMatrix(gravity);
	Eigen::VectorXd z = Eigen::Matrix<double, 6, 1>::Zero();
	z(0) = ft_raw.wrench.force.x;
	z(1) = ft_raw.wrench.force.y;
	z(2) = ft_raw.wrench.force.z;

	z(3) = ft_raw.wrench.torque.x;
	z(4) = ft_raw.wrench.torque.y;
	z(5) = ft_raw.wrench.torque.z;



	if(m_num_meas==1)
	{
		H = h;
		Z = z;
	}

	else
	{
		Eigen::MatrixXd H_temp = H;
		Eigen::VectorXd Z_temp = Z;

		H.resize(m_num_meas*6, 10);
		Z.resize(m_num_meas*6);

		H.topRows((m_num_meas-1)*6) = H_temp;
		Z.topRows((m_num_meas-1)*6) = Z_temp;

		H.bottomRows(6) = h;
		Z.bottomRows(6) = z;
	}


}


// Least squares to estimate the FT sensor parameters
Eigen::VectorXd FTCalib::getCalib()
{
	Eigen::VectorXd ft_calib_params(10);

	ft_calib_params = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Z);

	return ft_calib_params;
}


Eigen::MatrixXd FTCalib::GetMeasurementMatrix(const geometry_msgs::Vector3Stamped &gravity)
{
	KDL::Vector w = KDL::Vector::Zero();
	KDL::Vector alpha = KDL::Vector::Zero();
	KDL::Vector a = KDL::Vector::Zero();

	KDL::Vector g(gravity.vector.x, gravity.vector.y, gravity.vector.z);

	Eigen::MatrixXd H;
	H = Eigen::Matrix<double, 6, 10>::Zero();

	for(unsigned int i=0; i<3; i++)
	{
		for(unsigned int j=4; j<10; j++)
		{
			if(i==j-4)
			{
				H(i,j) = 1.0;
			}
			else
			{
				H(i,j) = 0.0;
			}
		}
	}

	for(unsigned int i=3; i<6; i++)
	{
		H(i,0) = 0.0;
	}

	H(3,1) = 0.0;
	H(4,2) = 0.0;
	H(5,3) = 0.0;

	for(unsigned int i=0; i<3; i++)
	{
		H(i,0) = a(i) - g(i);
	}

	H(0,1) = -w(1)*w(1) - w(2)*w(2);
	H(0,2) = w(0)*w(1) - alpha(2);
	H(0,3) = w(0)*w(2) + alpha(1);

	H(1,1) = w(0)*w(1) + alpha(2);
	H(1,2) = -w(0)*w(0) - w(2)*w(2);
	H(1,3) = w(1)*w(2) - alpha(0);

	H(2,1) = w(0)*w(2) - alpha(1);
	H(2,2) = w(1)*w(2) + alpha(0);
	H(2,3) = -w(1)*w(1) - w(0)*w(0);

	H(3,2) = a(2) - g(2);
	H(3,3) = -a(1) + g(1);


	H(4,1) = -a(2) + g(2);
	H(4,3) = a(0) - g(0);

	H(5,1) = a(1) - g(1);
	H(5,2) = -a(0) + g(0);

	for(unsigned int i=3; i<6; i++)
	{
		for(unsigned int j=4; j<10; j++)
		{
			if(i==(j-4))
			{
				H(i,j) = 1.0;
			}
			else
			{
				H(i,j) = 0.0;
			}
		}
	}


	return H;
}

}
