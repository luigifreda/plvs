/*
 * This file is part of PLVS
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

#include <opencv2/opencv.hpp>

namespace PLVS
{

class LineProjection
{
public: 
    LineProjection():
        uS(0.f),vS(0.f),invSz(0.f), 
        uE(0.f),vE(0.f),invEz(0.f),
        uM(0.f),vM(0.f),
        distMiddlePoint(0.f){}
    
    
    template<typename FrameType>
    void ProjectLine(const cv::Mat& Rcw, const cv::Mat& tcw, const MapLinePtr& pML, const FrameType& F);
    
    template<typename FrameType>
    bool ProjectLineWithCheck(const cv::Mat& Rcw, const cv::Mat& tcw, const MapLinePtr& pML, const FrameType& F);
    
    float uS,vS, invSz; // start (u,v, inverse depth)
    float uE,vE, invEz; // end   (u,v, inverse depth)
    float uM,vM;        // middle
    
    float distMiddlePoint; // middle point 
    cv::Mat p3DMw;
};


template<typename FrameType>
inline void LineProjection::ProjectLine(const cv::Mat& Rcw, const cv::Mat& tcw, const MapLinePtr& pML, const FrameType& F)
{
    const float &fx = F.fx;
    const float &fy = F.fy;
    const float &cx = F.cx;
    const float &cy = F.cy;
    const float &bf = F.mbf;
    
    cv::Mat p3DSw, p3DEw;
    pML->GetWorldEndPoints(p3DSw, p3DEw);          
    p3DMw = 0.5*(p3DSw+p3DEw);

    // 3D in camera coordinates
    const cv::Mat p3DSc = Rcw*p3DSw+tcw;    
    const float &p3DScX = p3DSc.at<float>(0);
    const float &p3DScY = p3DSc.at<float>(1);
    const float &p3DScZ = p3DSc.at<float>(2);    
    
    const cv::Mat p3DEc = Rcw*p3DEw+tcw;
    const float &p3DEcX = p3DEc.at<float>(0);
    const float &p3DEcY = p3DEc.at<float>(1);
    const float &p3DEcZ = p3DEc.at<float>(2);   
    
    // Project in image 
    invSz = 1.0f/p3DScZ;
    uS=fx*p3DScX*invSz+cx;
    vS=fy*p3DScY*invSz+cy;
    
    // Project in image 
    invEz = 1.0f/p3DEcZ;
    uE=fx*p3DEcX*invEz+cx;
    vE=fy*p3DEcY*invEz+cy;
    
    
    const cv::Mat p3DMc = 0.5*(p3DSc + p3DEc);
    uM = 0.5*(uS + uE);
    vM = 0.5*(vS + vE);
    
    const float &p3DMcX = p3DMc.at<float>(0);
    const float &p3DMcY = p3DMc.at<float>(1);
    const float &p3DMcZ = p3DMc.at<float>(2);  
    distMiddlePoint = sqrt(p3DMcX*p3DMcX + p3DMcY*p3DMcY + p3DMcZ*p3DMcZ);
}


template<typename FrameType>
inline bool LineProjection::ProjectLineWithCheck(const cv::Mat& Rcw, const cv::Mat& tcw, const MapLinePtr& pML, const FrameType& F)
{
    const float &fx = F.fx;
    const float &fy = F.fy;
    const float &cx = F.cx;
    const float &cy = F.cy;
    //const float &bf = F.mbf;

    // first check middle point 
    cv::Mat p3DSw, p3DEw;
    pML->GetWorldEndPoints(p3DSw, p3DEw);      
    p3DMw = 0.5*(p3DSw+p3DEw);

    // 3D in camera coordinates
    const cv::Mat p3DMc = Rcw*p3DMw+tcw;
    const float &p3DMcX = p3DMc.at<float>(0);
    const float &p3DMcY = p3DMc.at<float>(1);
    const float &p3DMcZ = p3DMc.at<float>(2);
        
    // Check positive depth
    if(p3DMcZ<0.0f)
        return false;  
    
    // Project in image and check it is not outside
    const float invMz = 1.0f/p3DMcZ;
    uM = fx*p3DMcX*invMz+cx;
    vM = fy*p3DMcY*invMz+cy;
    
    if(uM<F.mnMinX || uM>F.mnMaxX)
        return false;
    if(vM<F.mnMinY || vM>F.mnMaxY)
        return false;        
    
    distMiddlePoint = sqrt(p3DMcX*p3DMcX + p3DMcY*p3DMcY + p3DMcZ*p3DMcZ);
    
    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();
    if(distMiddlePoint<minDistance || distMiddlePoint>maxDistance )
        return false;    
    
    // 3D in camera coordinates
    const cv::Mat p3DSc = Rcw*p3DSw+tcw;    
    const float &p3DScX = p3DSc.at<float>(0);
    const float &p3DScY = p3DSc.at<float>(1);
    const float &p3DScZ = p3DSc.at<float>(2);    
    
    // Project in image 
    invSz = 1.0f/p3DScZ;
    uS=fx*p3DScX*invSz+cx;
    vS=fy*p3DScY*invSz+cy;

    if(uS<F.mnMinX || uS>F.mnMaxX)
        return false;
    if(vS<F.mnMinY || vS>F.mnMaxY)
        return false;         

    // 3D in camera coordinates    
    const cv::Mat p3DEc = Rcw*p3DEw+tcw;
    const float &p3DEcX = p3DEc.at<float>(0);
    const float &p3DEcY = p3DEc.at<float>(1);
    const float &p3DEcZ = p3DEc.at<float>(2);   
    
    // Project in image 
    invEz = 1.0f/p3DEcZ;
    uE=fx*p3DEcX*invEz+cx;
    vE=fy*p3DEcY*invEz+cy;
        
    if(uE<F.mnMinX || uE>F.mnMaxX)
        return false;
    if(vE<F.mnMinY || vE>F.mnMaxY)
        return false;     

    return true; 
}


}// namespace PLVS

#endif 
