

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