/*
 * Copyright (c) 2010, Hozefa Indorewala <indorewala@ias.in.tum.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINTCLOUD_REGISTRATION_POINT_TYPES_H_
#define POINTCLOUD_REGISTRATION_POINT_TYPES_H_

#include <Eigen/Core>
#include <bitset>
#include <vector>
#include "pcl/ros/register_point_struct.h"
#include <pcl/point_types.h>

namespace pcl
{
  //struct PointXYZINormal;
  struct PointXYZRGBNormalScanIndex; //For use in ICP

  struct SpinImageLocal;
  // Members: float histogram[100];

}

#include <pointcloud_registration/pointcloud_registration_point_types.hpp>  // Include struct definitions


// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

//POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZINormal,
//                                   (float, x, x)
//                                   (float, y, y)
//                                   (float, z, z)
//                                   (float, intensity, intensity)
//                                   (float, normal[0], normal_x)
//                                   (float, normal[1], normal_y)
//                                   (float, normal[2], normal_z)
//                                   (float, curvature, curvature)
//);
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZIRGBNormal,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, rgb, rgb)
                                   (float, normal[0], normal_x)
                                   (float, normal[1], normal_y)
                                   (float, normal[2], normal_z)
                                   (float, curvature, curvature)
);

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::SpinImageLocal,
                                   (uint32_t[100], histogram, sil)

);
#endif  //#ifndef POINTCLOUD_REGISTRATION_POINT_TYPES_H_
