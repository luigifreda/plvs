/*
 * This file is part of PLVS.

 * Copyright (C) 2018-present Luigi Freda <luigifreda at gmail dot com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef POINT_TYPE_DEFINITIONS_H
#define POINT_TYPE_DEFINITIONS_H 

#include <type_traits>
#include "PointSurfelSegment.h"


#define USE_NORMALS 1
#define USE_POINTSURFELSEGMENT 1

#if !USE_NORMALS

    #define POINT_TYPE pcl::PointXYZRGBA
    #define POINT_TYPE_COLOR_OFFSET 4  // in terms of float 
    #define COMPUTE_NORMALS 0
    #define COMPUTE_SEGMENTS 0

#else

    #if !USE_POINTSURFELSEGMENT

        #define POINT_TYPE pcl::PointXYZRGBNormal
        #define POINT_TYPE_NORMAL_OFFSET 4  // in terms of float 
        #define POINT_TYPE_COLOR_OFFSET 8   // in terms of float 
        #define COMPUTE_NORMALS ( 1 && USE_NORMALS)
        #define COMPUTE_SEGMENTS 0        

    #else

        #define POINT_TYPE pcl::PointSurfelSegment
        #define POINT_TYPE_NORMAL_OFFSET 4  // in terms of float 
        #define POINT_TYPE_COLOR_OFFSET 8   // in terms of float 
        #define POINT_TYPE_LABEL_OFFSET (4*10)   // in terms of uchar
        #define COMPUTE_NORMALS  ( 1 && USE_NORMALS)   
        #define COMPUTE_SEGMENTS ( 1 && USE_NORMALS)

    #endif

#endif


#endif 