/*
 *  gravity_compensation_params.cpp
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
#include <gravity_compensation/gravity_compensation_params.h>


GravityCompensationParams::GravityCompensationParams()
{

}

GravityCompensationParams::~GravityCompensationParams()
{

}

void GravityCompensationParams::setBias(const Eigen::Matrix<double, 6, 1> &bias)
{
	m_bias = bias;
}

Eigen::Matrix<double, 6, 1> GravityCompensationParams::getBias()
{
    return m_bias;
}

void GravityCompensationParams::setGripperMass(const double &gripper_mass)
{
	m_gripper_mass = gripper_mass;
}

double GravityCompensationParams::getGripperMass()
{
    return m_gripper_mass;
}

void GravityCompensationParams::setGripperCOM(const tf::StampedTransform &gripper_com)
{
    m_gripper_com = gripper_com;
}

tf::StampedTransform GravityCompensationParams::getGripperCOM()
{
    return m_gripper_com;
}
