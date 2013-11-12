/*
 *  gravity_compensation_params.h
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


#ifndef GRAVITY_COMPENSATION_PARAMS_H_
#define GRAVITY_COMPENSATION_PARAMS_H_

#include <vector>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Core>

class GravityCompensationParams
{
public:

    GravityCompensationParams();

    virtual ~GravityCompensationParams();

    // set/get the F/T sensor bias
    // used to 'zero' the F/T sensor signal
    void setBias(const Eigen::Matrix<double, 6, 1> &bias);
    Eigen::Matrix<double, 6, 1> getBias();

    // set/get the mass of the gripper
    // used for compensating gravity
    void setGripperMass(const double &gripper_mass);
    double getGripperMass();

    // set/get the COM of the gripper
    // Used for compensating gravity.
    // Compensation assumes that the gripper COM is fixed
    // with respect to the F/T sensor
    void setGripperCOM(const tf::StampedTransform &gripper_com);
    tf::StampedTransform getGripperCOM();

private:

    // 6 element bias vector
    Eigen::Matrix<double, 6, 1> m_bias;

    // the mass of the gripper
    double m_gripper_mass;

    // (fixed) pose of the center of mass of the gripper
    // with respect to the F/T sensor
    tf::StampedTransform m_gripper_com;

};



#endif
