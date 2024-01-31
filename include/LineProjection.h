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

#ifndef LINE_PROJECTION_H
#define LINE_PROJECTION_H

#include <math.h>
#include <iostream>

#include "MapLine.h"

#include "Eigen/Core"

namespace PLVS2
{


class FrameTransformsForLineProjection
{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FrameTransformsForLineProjection() = default;

    template<typename FrameType>    
    explicit FrameTransformsForLineProjection(FrameType& F)
    {
        const auto mTcw = F.GetPose();
        mRcw = mTcw.rotationMatrix();
        mtcw = mTcw.translation();
        const auto mTrw = F.GetRelativePoseTrl() * mTcw;
        mRrw = mTrw.rotationMatrix();
        mtrw = mTrw.translation();
    }
    // init left and right
    FrameTransformsForLineProjection(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, const Eigen::Matrix3f& Rrw, const Eigen::Vector3f& trw)
    {
        mRcw = Rcw;
        mtcw = tcw;
        mRrw = Rrw;
        mtrw = trw;
    }
    // init left only (right is initialized as left but it is supposed not to be used)
    FrameTransformsForLineProjection(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw)
    {
        mRcw = Rcw;
        mtcw = tcw;

        // right transforms initialized as left but not used 
        mRrw = Rcw;
        mtrw = tcw;
    }

    Eigen::Matrix3f mRcw; // left camera pose (world to left camera)
    Eigen::Vector3f mtcw; 

    Eigen::Matrix3f mRrw; // right camera pose (world to right camera)
    Eigen::Vector3f mtrw;
};


class LineProjection
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LineProjection():
        uS(0.f),vS(0.f),invSz(0.f), 
        uE(0.f),vE(0.f),invEz(0.f),
        uM(0.f),vM(0.f),
        distMiddlePoint(0.f){}
    
    
    // template<typename FrameType>
    // void ProjectLine(const MapLinePtr& pML, const FrameType& F, const FrameTransformsForLineProjection& frameTfs, bool bRight=false);
    
    template<typename FrameType>
    bool ProjectLineWithCheck(const MapLinePtr& pML, const FrameType& F, const FrameTransformsForLineProjection& frameTfs, bool bRight=false);
    
    // undistorted projections 
    float uS,vS, invSz; // start (u,v, inverse depth)
    float uE,vE, invEz; // end   (u,v, inverse depth)
    float uM,vM;        // middle
    
    float distMiddlePoint; // middle point 
    Eigen::Vector3f p3DMw;
};


// template<typename FrameType>
// inline void LineProjection::ProjectLine(const MapLinePtr& pML, const FrameType& F, const FrameTransformsForLineProjection& frameTfs, bool bRight)
// {
//     const GeometricCamera* cam = bRight ? F.mpCamera2 : F.mpCamera;
//     const auto& R = bRight ? frameTfs.mRrw : frameTfs.mRcw;
//     const auto& t = bRight ? frameTfs.mtrw : frameTfs.mtcw;

//     Eigen::Vector3f p3DSw, p3DEw;
//     pML->GetWorldEndPoints(p3DSw, p3DEw);          
//     p3DMw = 0.5*(p3DSw+p3DEw);

//     // 3D in camera coordinates
//     const Eigen::Vector3f p3DSc = R*p3DSw+t;    
//     const float &p3DScZ = p3DSc(2);    
    
//     const Eigen::Vector3f p3DEc = R*p3DEw+t;
//     const float &p3DEcZ = p3DEc(2);   
    
//     // Project in image (undistorted)
//     invSz = 1.0f/p3DScZ;
//     const Eigen::Vector2f uvS = cam->projectLinear(p3DSc); 
//     uS= uvS(0); //fx*p3DScX*invSz+cx;
//     vS= uvS(1); //fy*p3DScY*invSz+cy;
    
//     // Project in image (undistorted)
//     invEz = 1.0f/p3DEcZ;
//     const Eigen::Vector2f uvE = cam->projectLinear(p3DEc);
//     uE= uvE(0); //fx*p3DEcX*invEz+cx;
//     vE= uvE(1); //fy*p3DEcY*invEz+cy;
    
//     const Eigen::Vector3f p3DMc = 0.5*(p3DSc + p3DEc);
//     uM = 0.5*(uS + uE);
//     vM = 0.5*(vS + vE);
    
//     distMiddlePoint = p3DMc.norm();
// }


template<typename FrameType>
inline bool LineProjection::ProjectLineWithCheck(const MapLinePtr& pML, const FrameType& F, const FrameTransformsForLineProjection& frameTfs, bool bRight)
{
    const GeometricCamera* cam = bRight ? F.mpCamera2 : F.mpCamera;
    const auto& R = bRight ? frameTfs.mRrw : frameTfs.mRcw;
    const auto& t = bRight ? frameTfs.mtrw : frameTfs.mtcw;
    const bool isPinhole = cam->GetType() == GeometricCamera::CAM_PINHOLE;

    // first check middle point 
    Eigen::Vector3f p3DSw, p3DEw;
    pML->GetWorldEndPoints(p3DSw, p3DEw);      
    p3DMw = 0.5*(p3DSw+p3DEw);

    // 3D in camera coordinates
    const Eigen::Vector3f p3DMc = R*p3DMw+t; 
    const float &p3DMcZ = p3DMc(2);
        
    // Check positive depth
    if(p3DMcZ<0.0f)
        return false;  
    
    // Project in image without distortion model
    const Eigen::Vector2f uvM = cam->projectLinear(p3DMc);
    uM = uvM(0); //fx*p3DMcX*invMz+cx;
    vM = uvM(1); //fy*p3DMcY*invMz+cy;
    
    // Project in image with distortion model and check it is not outside
    if(isPinhole)
    {
        // use projection without distortion model 
        if(uM<F.mnMinX || uM>F.mnMaxX)
            return false;
        if(vM<F.mnMinY || vM>F.mnMaxY)
            return false;   
    }
    else 
    {
        // project with distortion model 
        const Eigen::Vector2f uvMraw = cam->project(p3DMc); 
        if(uvMraw(0)<F.mnMinX || uvMraw(0)>F.mnMaxX)
            return false;
        if(uvMraw(1)<F.mnMinY || uvMraw(1)>F.mnMaxY)
            return false;   
    }     
    
    distMiddlePoint = p3DMc.norm();
    
    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();
    if(distMiddlePoint<minDistance || distMiddlePoint>maxDistance )
        return false;    
    
    // 3D in camera coordinates of start point
    const Eigen::Vector3f p3DSc = R*p3DSw+t;
    const float &p3DScZ = p3DSc(2);    
    
    // Check positive depth
    if(p3DScZ<0.0f)
        return false;  

    // 3D in camera coordinates of end point 
    const Eigen::Vector3f p3DEc = R*p3DEw+t; 
    const float &p3DEcZ = p3DEc(2);   

    // Check positive depth
    if(p3DEcZ<0.0f)
        return false;    


    // Project start point in image without distortion model
    invSz = 1.0f/p3DScZ;
    const Eigen::Vector2f uvS = cam->projectLinear(p3DSc);     
    uS= uvS(0); //fx*p3DScX*invSz+cx;
    vS= uvS(1); //fy*p3DScY*invSz+cy;

    // Project in image with distortion model and check it is not outside
    if(isPinhole)
    {
        // use projection without distortion model
        if(uS<F.mnMinX || uS>F.mnMaxX)
            return false;
        if(vS<F.mnMinY || vS>F.mnMaxY)
            return false;        
    }
    else 
    {
        // project with distortion model 
        const Eigen::Vector2f uvSraw = cam->project(p3DSc); 
        if(uvSraw(0)<F.mnMinX || uvSraw(0)>F.mnMaxX)
            return false;
        if(uvSraw(1)<F.mnMinY || uvSraw(1)>F.mnMaxY)
            return false;
    } 
    
    // Project end point in image without distortion model
    invEz = 1.0f/p3DEcZ;
    const Eigen::Vector2f uvE = cam->projectLinear(p3DEc);    
    uE= uvE(0); //fx*p3DEcX*invEz+cx;
    vE= uvE(1); //fy*p3DEcY*invEz+cy;
        
    // Project in image with distortion model and check it is not outside        
    if(isPinhole)
    {
        // use projection without distortion model
        if(uE<F.mnMinX || uE>F.mnMaxX)
            return false;
        if(vE<F.mnMinY || vE>F.mnMaxY)
            return false;     
    }
    else 
    {
        // project with distortion model 
        const Eigen::Vector2f uvEraw = cam->project(p3DEc); 
        if(uvEraw(0)<F.mnMinX || uvEraw(0)>F.mnMaxX)
            return false;
        if(uvEraw(1)<F.mnMinY || uvEraw(1)>F.mnMaxY)
            return false;
    }

    return true; 
}


}// namespace PLVS2

#endif 
