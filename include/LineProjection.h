

#ifndef LINE_PROJECTION_H
#define LINE_PROJECTION_H

#include <math.h>
#include <iostream>

#include "MapLine.h"

#include "Eigen/Core"

namespace PLVS2
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
    void ProjectLine(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, const MapLinePtr& pML, const FrameType& F);
    
    template<typename FrameType>
    bool ProjectLineWithCheck(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, const MapLinePtr& pML, const FrameType& F);
    
    float uS,vS, invSz; // start (u,v, inverse depth)
    float uE,vE, invEz; // end   (u,v, inverse depth)
    float uM,vM;        // middle
    
    float distMiddlePoint; // middle point 
    Eigen::Vector3f p3DMw;
};


template<typename FrameType>
inline void LineProjection::ProjectLine(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, const MapLinePtr& pML, const FrameType& F)
{
    const float &fx = F.fx;
    const float &fy = F.fy;
    const float &cx = F.cx;
    const float &cy = F.cy;
    const float &bf = F.mbf;
    
    Eigen::Vector3f p3DSw, p3DEw;
    pML->GetWorldEndPoints(p3DSw, p3DEw);          
    p3DMw = 0.5*(p3DSw+p3DEw);

    // 3D in camera coordinates
    const Eigen::Vector3f p3DSc = Rcw*p3DSw+tcw;    
    const float &p3DScX = p3DSc(0);
    const float &p3DScY = p3DSc(1);
    const float &p3DScZ = p3DSc(2);    
    
    const Eigen::Vector3f p3DEc = Rcw*p3DEw+tcw;
    const float &p3DEcX = p3DEc(0);
    const float &p3DEcY = p3DEc(1);
    const float &p3DEcZ = p3DEc(2);   
    
    // Project in image 
    invSz = 1.0f/p3DScZ;
    uS=fx*p3DScX*invSz+cx;
    vS=fy*p3DScY*invSz+cy;
    
    // Project in image 
    invEz = 1.0f/p3DEcZ;
    uE=fx*p3DEcX*invEz+cx;
    vE=fy*p3DEcY*invEz+cy;
    
    
    const Eigen::Vector3f p3DMc = 0.5*(p3DSc + p3DEc);
    uM = 0.5*(uS + uE);
    vM = 0.5*(vS + vE);
    
    const float &p3DMcX = p3DMc(0);
    const float &p3DMcY = p3DMc(1);
    const float &p3DMcZ = p3DMc(2);  
    distMiddlePoint = sqrt(p3DMcX*p3DMcX + p3DMcY*p3DMcY + p3DMcZ*p3DMcZ);
}


template<typename FrameType>
inline bool LineProjection::ProjectLineWithCheck(const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, const MapLinePtr& pML, const FrameType& F)
{
    const float &fx = F.fx;
    const float &fy = F.fy;
    const float &cx = F.cx;
    const float &cy = F.cy;
    //const float &bf = F.mbf;

    // first check middle point 
    Eigen::Vector3f p3DSw, p3DEw;
    pML->GetWorldEndPoints(p3DSw, p3DEw);      
    p3DMw = 0.5*(p3DSw+p3DEw);

    // 3D in camera coordinates
    const Eigen::Vector3f p3DMc = Rcw*p3DMw+tcw;
    const float &p3DMcX = p3DMc(0);
    const float &p3DMcY = p3DMc(1);
    const float &p3DMcZ = p3DMc(2);
        
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
    const Eigen::Vector3f p3DSc = Rcw*p3DSw+tcw;    
    const float &p3DScX = p3DSc(0);
    const float &p3DScY = p3DSc(1);
    const float &p3DScZ = p3DSc(2);    
    
    // Project in image 
    invSz = 1.0f/p3DScZ;
    uS=fx*p3DScX*invSz+cx;
    vS=fy*p3DScY*invSz+cy;

    if(uS<F.mnMinX || uS>F.mnMaxX)
        return false;
    if(vS<F.mnMinY || vS>F.mnMaxY)
        return false;         

    // 3D in camera coordinates    
    const Eigen::Vector3f p3DEc = Rcw*p3DEw+tcw;
    const float &p3DEcX = p3DEc(0);
    const float &p3DEcY = p3DEc(1);
    const float &p3DEcZ = p3DEc(2);   
    
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


}// namespace PLVS2

#endif 
