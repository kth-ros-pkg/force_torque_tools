/*
 *  ft_calib.h
 *
 *  Least squares calibration of:
 *  - Bias of F/T sensor
 *  - Mass of attached gripper
 *  - Location of the center of mass of the gripper
 *
 *  Requires calibrated accelerometer readings
 *  (calibrated with respect to the robot).
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



#ifndef FTCALIB_H_
#define FTCALIB_H_
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen3/Eigen/Core>


// Least Squares calibration of bias of FT sensor and the mass and location of the COM of the gripper
namespace Calibration{
class FTCalib
{
public:

	FTCalib();
	virtual ~FTCalib();


	// adds a F/T measurement and the corresponding measurement matrix from the gravity
	// measurements of the accelerometer
	// gravity is assumed to be expressed in the F/T sensor frame
	virtual void addMeasurement(const geometry_msgs::Vector3Stamped &gravity,
			const geometry_msgs::WrenchStamped &ft_raw);


	// Least squares to estimate the F/T sensor parameters
	// The estimated parameters are :
	// [m m*cx m*cy m*cz FBx FBy FBz TBx TBy TBz]
	// m: mass of the gripper
	// [cx, cy, cz] are the coordinates of the center of mass of the gripper
	// FB : force bias
	// TB: torque bias
	// All expressed in the FT sensor frame
	virtual Eigen::VectorXd getCalib();



protected:

	Eigen::MatrixXd H; // stacked measurement matrices
	Eigen::VectorXd Z; // stacked F/T measurements
	// FT_sensor_frame_acc taken as 0

	unsigned int m_num_meas; // number of stacked measurements;

	// measurement matrix based on "On-line Rigid Object Recognition and Pose Estimation
	//  Based on Inertial Parameters", D. Kubus, T. Kroger, F. Wahl, IROS 2008
	virtual Eigen::MatrixXd GetMeasurementMatrix(const geometry_msgs::Vector3Stamped &gravity);

};
}


#endif /* INERTIALPARAMESTIMATOR_H_ */
