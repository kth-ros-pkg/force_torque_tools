/*
 *  gravity_compensation.h
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

#ifndef GRAVITY_COMPENSATION_H_
#define GRAVITY_COMPENSATION_H_

#include <gravity_compensation/gravity_compensation_params.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>


class GravityCompensation
{
public:

    GravityCompensation(GravityCompensationParams *g_comp_params);

    virtual ~GravityCompensation();

    // Eliminate bias from the force-torque readings
    void Zero(const geometry_msgs::WrenchStamped &ft_raw,
         geometry_msgs::WrenchStamped &ft_zeroed);

    // Compensate for the weight of the gripper.
    // Assumes that the gripper has fixed mass and COM
    // relative to F/T sensor.
    bool Compensate(const geometry_msgs::WrenchStamped &ft_zeroed,
                    const sensor_msgs::Imu &gravity,
                    geometry_msgs::WrenchStamped &ft_compensated);

private:

    GravityCompensationParams *m_g_comp_params;
    tf::TransformListener *m_tf_listener;


};


#endif
