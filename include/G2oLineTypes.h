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

#pragma once 

#include "G2oTypes.h"
#include "g2o/types_sba_line.h"


#define USE_ANALYTIC_JACS_IMU 0

#define USE_ANALYTIC_JACS_FULL_MONO_IMU      (1 && USE_ANALYTIC_JACS_IMU)
#define USE_ANALYTIC_JACS_ONLY_POSE_MONO_IMU (1 && USE_ANALYTIC_JACS_IMU)

#define USE_ANALYTIC_JACS_FULL_STEREO_IMU      (1 && USE_ANALYTIC_JACS_IMU)
#define USE_ANALYTIC_JACS_ONLY_POSE_STEREO_IMU (1 && USE_ANALYTIC_JACS_IMU)

namespace PLVS2
{

class EdgeLineMono: public g2o::BaseBinaryEdge<2/*error size*/,Eigen::Vector3d/*meas type*/,g2o::VertexSBALine,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineMono(int cam_idx_=0): cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]); // i
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);                 // j 

        const Eigen::Vector2d projS = VPose->estimate().ProjectLinear(VLine->estimate().head(3),cam_idx); // [us;vs] start point Sw cam projection
        const Eigen::Vector2d projE = VPose->estimate().ProjectLinear(VLine->estimate().tail(3),cam_idx); // [ue;ve] end point Ew cam projection
        
        // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
        // e2 = [eS] = [ [nx,ny]*projS -d ]
        //      [eE]   [ [nx,ny]*projE -d ]        
        // constraint: 0 = eS = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
        _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
        // constraint: 0 = eE = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
        _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
    }

#if USE_ANALYTIC_JACS_FULL_MONO_IMU
    virtual void linearizeOplus();
#endif     

    bool areDepthsPositive() {
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]);         
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        return VPose->estimate().isDepthPositive(VLine->estimate().head(3),cam_idx) && 
               VPose->estimate().isDepthPositive(VLine->estimate().tail(3),cam_idx);        
    }    

    Eigen::Matrix<double,2,12> GetJacobian(){
        linearizeOplus();
        Eigen::Matrix<double,2,12> J; // 12 = 3 + 3 + 6   J = [de/d(Sw, Ew), de/d(mu)]
        J.block<2,6>(0,0) = _jacobianOplusXi; // line   de2/d(Sw, Ew) \in IR^2x6
        J.block<2,6>(0,6) = _jacobianOplusXj; // pose   de2/d(mu) \in IR^2x6
        return J;
    }

    Eigen::Matrix<double,12,12> GetHessian(){
        const Eigen::Matrix<double,2,12> J = GetJacobian();
        return J.transpose()*information()*J;
    }

    void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]); // i
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);                 // j 

        projS = VPose->estimate().ProjectLinear(VLine->estimate().head(3),cam_idx); // [us;vs] start point Sw cam projection
        projE = VPose->estimate().ProjectLinear(VLine->estimate().tail(3),cam_idx); // [ue;ve] end point Ew cam projection                
    }  

public:
    const int cam_idx;
};


class EdgeLineMonoOnlyPose : public g2o::BaseUnaryEdge<2/*error size*/,Eigen::Vector3d/*meas type*/,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineMonoOnlyPose(const Eigen::Vector3f &XSw_, const Eigen::Vector3f &XEw_, int cam_idx_=0)
                         :XSw(XSw_.cast<double>()),XEw(XEw_.cast<double>()), cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

        const Eigen::Vector2d projS = VPose->estimate().ProjectLinear(XSw,cam_idx); // [us;vs] start point Sw cam projection 
        const Eigen::Vector2d projE = VPose->estimate().ProjectLinear(XEw,cam_idx); // [ue;ve] end point Ew cam projection
        
        // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
        // e2 = [eS] = [ [nx,ny]*projS -d ]
        //      [eE]   [ [nx,ny]*projE -d ]        
        // constraint: 0 = eS = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
        _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
        // constraint: 0 = eE = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
        _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
    }    

#if USE_ANALYTIC_JACS_ONLY_POSE_MONO_IMU
    virtual void linearizeOplus();
#endif 

    bool areDepthsPositive() {
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        return VPose->estimate().isDepthPositive(XSw,cam_idx) && 
               VPose->estimate().isDepthPositive(XEw,cam_idx);        
    }    

    Eigen::Matrix<double,6,6> GetHessian(){
        linearizeOplus();
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;
    }

    void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        projS = VPose->estimate().ProjectLinear(XSw,cam_idx); // [us;vs] start point Sw cam projection 
        projE = VPose->estimate().ProjectLinear(XEw,cam_idx); // [ue;ve] end point Ew cam projection
    }  

public:
    const Eigen::Vector3d XSw, XEw;
    const int cam_idx;
};



class EdgeLineStereo : public g2o::BaseBinaryEdge<4/*error size*/,Eigen::Vector3d/*meas type*/,g2o::VertexSBALine,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineStereo(int cam_idx_=0): cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]); // i
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);                 // j 

        const ImuCamPose& imuCamPose = VPose->estimate();
        const Vector6d& mapLine = VLine->estimate();

        const Eigen::Vector3d XSc = imuCamPose.ToCam(mapLine.head(3),cam_idx); // S w.r.t. camera frame (aka P)
        const Eigen::Vector3d XEc = imuCamPose.ToCam(mapLine.tail(3),cam_idx); // E w.r.t. camera frame (aka Q)       
        const Eigen::Vector2d projS = imuCamPose.pCamera[cam_idx]->projectLinear(XSc); // [us;vs] start point Sw cam projection
        const Eigen::Vector2d projE = imuCamPose.pCamera[cam_idx]->projectLinear(XEc); // [ue;ve] end point Ew cam projection
        
        // NOTE: 
        // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
        //      [eS]   [ [nx,ny]*projS -d                                ]
        // e4 = [eE] = [ [nx,ny]*projE -d                                ]  
        //      [dS]  [ |(Sc-Bs) X (Sc-Be)|/|Bs-Be| + muWeigth * |Sc-Bs| ]
        //      [dE]  [ |(Ec-Bs) X (Ec-Be)|/|Bs-Be| + muWeigth * |Ec-Be| ]
        // where Bs = backproj(s) and Be = backproj(e) and s,e are the 2D line observation endpoints on the image plane

        // constraint: 0 = eS = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
        _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
        // constraint: 0 = eE = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
        _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
            
        // align each point of 3D map-line to 3D detected-line (by moving the camera frame and adjusting 3D map-line points)
        const Eigen::Vector3d Sc_Bs = XSc - XSc_backproj; // Sc-Bs
        const Eigen::Vector3d Ec_Be = XEc - XEc_backproj; // Ec-Be
        _error[2] = Sc_Bs.cross(XSc - XEc_backproj).norm() * lineLenghtInv + muWeigth * Sc_Bs.norm();   //  |(S-Bs) X (S-Be)|/|Bs-Be| + muWeigth * |S-Bs|
        _error[3] = (XEc - XSc_backproj).cross(Ec_Be).norm() * lineLenghtInv + muWeigth * Ec_Be.norm(); //  |(E-Bs) X (E-Be)|/|Bs-Be| + muWeigth * |E-Be|           
    }

#if USE_ANALYTIC_JACS_FULL_STEREO_IMU
    virtual void linearizeOplus();
#endif 

    Eigen::Matrix<double,4,12> GetJacobian(){
        linearizeOplus();
        Eigen::Matrix<double,4,12> J; // 12 = 3 + 3 + 6   J = [de4/d(Sw, Ew), de4/d(mu)]
        J.block<4,6>(0,0) = _jacobianOplusXi; // line   de4/d(Sw, Ew) \in IR^2x6
        J.block<4,6>(0,6) = _jacobianOplusXj; // pose   de4/d(mu) \in IR^2x6        
        return J;
    }

    Eigen::Matrix<double,12,12> GetHessian(){
        Eigen::Matrix<double,4,12> J = GetJacobian();
        return J.transpose()*information()*J;
    }

    void setBackprojections(const Eigen::Vector3f& XSc_backproj_, const Eigen::Vector3f& XEc_backproj_){
        XSc_backproj = XSc_backproj_.cast<double>();
        XEc_backproj = XEc_backproj_.cast<double>();
        const Eigen::Vector3d deltaBackproj = XSc_backproj - XEc_backproj; // deltaB
        lineLenghtInv = 1.0/deltaBackproj.norm();
        skewDeltaBackproj_over_lineLength = Skew(deltaBackproj)*lineLenghtInv; // Skew(deltaBackproj)/|deltaBackproj|
        deltaBackprojNormalized = deltaBackproj*lineLenghtInv;
    }

    // to be called after all the data have been filled up
    void init(){
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]); // i
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);                 // j 

        const ImuCamPose& imuCamPose = VPose->estimate();
        const Vector6d& mapLine = VLine->estimate();

        const Eigen::Vector3d XSc = imuCamPose.ToCam(mapLine.head(3),cam_idx); // S w.r.t. camera frame
        const Eigen::Vector3d XEc = imuCamPose.ToCam(mapLine.tail(3),cam_idx); // E w.r.t. camera frame    
        
        // check if the end-points association is correct
        const Eigen::Vector3d mapLineSE = (XSc-XEc).normalized();
        const Eigen::Vector3d& lineSE = deltaBackprojNormalized; //(XSc_backproj-XEc_backproj).normalized();    
        if(mapLineSE.dot(lineSE) < 0)
        {
            //std::cout << "BA: swapping line endpoints" << std::endl;
            std::swap(XSc_backproj,XEc_backproj);
            deltaBackprojNormalized *= -1;
            skewDeltaBackproj_over_lineLength *= -1;            
        }    
    }

    bool areDepthsPositive() {
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]);         
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        return VPose->estimate().isDepthPositive(VLine->estimate().head(3),cam_idx) && 
               VPose->estimate().isDepthPositive(VLine->estimate().tail(3),cam_idx);        
    }    

    // compute just line-alignment error (do not consider the constraints/regularizers on the line end-points)  
    double zeroMuChi2() {
        const double muWeigth_current = muWeigth;
        muWeigth = 0; 
        computeError();  
        const double res = _error.dot(information()*_error);
        muWeigth = muWeigth_current;    
        return res;
    }    
    
    // compute total squared 3D error 
    double computeSquared3DError() {
        // compute full 3D error with used muWeigth and without information matrix
        computeError(); 
        return (_error[2]*_error[2] + _error[3]*_error[3]);
    }  

    void getMapLineAndProjections(Eigen::Vector3d& mapS, Eigen::Vector3d& mapE, Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]); // i
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);                 // j 

        const ImuCamPose& imuCamPose = VPose->estimate();
        const Vector6d& mapLine = VLine->estimate();

        mapS = imuCamPose.ToCam(mapLine.head(3),cam_idx); // S w.r.t. camera frame (aka P)
        mapE = imuCamPose.ToCam(mapLine.tail(3),cam_idx); // E w.r.t. camera frame (aka Q)       
        projS = imuCamPose.pCamera[cam_idx]->projectLinear(mapS); // [us;vs] start point Sw cam projection
        projE = imuCamPose.pCamera[cam_idx]->projectLinear(mapE); // [ue;ve] end point Ew cam projection        
    }
    

public:
    const int cam_idx;

    // the following block of data must be kept consistent 
    Eigen::Vector3d XSc_backproj; // backprojected start point w.r.t. camera frame 
    Eigen::Vector3d XEc_backproj; // backprojected end point w.r.t camera frame 
    Eigen::Vector3d deltaBackprojNormalized;     // (XSc_backproj - XEc_backproj).normalized()
    Eigen::Matrix3d skewDeltaBackproj_over_lineLength; // Skew(deltaBackproj)/|deltaBackproj|
    double lineLenghtInv; // inverse of backprojected line length 

    double muWeigth; // for weighting the point-point distance |P-Bp| (or |Q-Bq| ) w.r.t. distance point-line |(P-Bp) X (P-Bq)|/|Bq-Bp|  ( or |(Q-Bp) X (Q-Bq)|/|Bq-Bp| )    
};


class EdgeLineStereoOnlyPose : public g2o::BaseUnaryEdge<4/*error size*/,Eigen::Vector3d/*meas type*/,VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeLineStereoOnlyPose(const Eigen::Vector3f &XSw_, const Eigen::Vector3f &XEw_, int cam_idx_=0)
                         :XSw(XSw_.cast<double>()),XEw(XEw_.cast<double>()), cam_idx(cam_idx_){}

    virtual bool read(std::istream& is){return false;}
    virtual bool write(std::ostream& os) const{return false;}

    void computeError(){
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        const ImuCamPose& imuCamPose = VPose->estimate();

        const Eigen::Vector3d XSc = imuCamPose.ToCam(XSw,cam_idx); // S w.r.t. camera frame (aka P)
        const Eigen::Vector3d XEc = imuCamPose.ToCam(XEw,cam_idx); // E w.r.t. camera frame (aka Q)       
        const Eigen::Vector2d projS = imuCamPose.pCamera[cam_idx]->projectLinear(XSc); // [us;vs] start point Sw cam projection
        const Eigen::Vector2d projE = imuCamPose.pCamera[cam_idx]->projectLinear(XEc); // [ue;ve] end point Ew cam projection
        
        // NOTE: 
        // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
        //      [eS]   [ [nx,ny]*projS -d          ]
        // e4 = [eE] = [ [nx,ny]*projE -d          ]  
        //      [dS]  [ |(Sc-Bs) X (Sc-Be)|/|Bs-Be|]
        //      [dE]  [ |(Ec-Bs) X (Ec-Be)|/|Bs-Be|]
        // where Bs = backproj(s) and Be = backproj(e) and s,e are the 2D line observation endpoints on the image plane

        // constraint: 0 = eS = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
        _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
        // constraint: 0 = eE = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
        _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
            
        // align each point of 3D map-line to 3D detected-line (by moving the camera frame and adjusting 3D map-line points)
        const Eigen::Vector3d Sc_Bs = XSc - XSc_backproj; // Sc-Bs
        const Eigen::Vector3d Ec_Be = XEc - XEc_backproj; // Ec-Be
        _error[2] = Sc_Bs.cross(XSc - XEc_backproj).norm() * lineLenghtInv;   //  |(S-Bs) X (S-Be)|/|Bs-Be| 
        _error[3] = (XEc - XSc_backproj).cross(Ec_Be).norm() * lineLenghtInv; //  |(E-Bs) X (E-Be)|/|Bs-Be|          
    }

#if USE_ANALYTIC_JACS_ONLY_POSE_STEREO_IMU
    virtual void linearizeOplus();
#endif 

    Eigen::Matrix<double,6,6> GetHessian(){
        linearizeOplus();
        return _jacobianOplusXi.transpose()*information()*_jacobianOplusXi;
    }

    void setBackprojections(const Eigen::Vector3f& XSc_backproj_, const Eigen::Vector3f& XEc_backproj_){
        XSc_backproj = XSc_backproj_.cast<double>();
        XEc_backproj = XEc_backproj_.cast<double>();
        const Eigen::Vector3d deltaBackproj = XSc_backproj - XEc_backproj; // deltaB
        lineLenghtInv = 1.0/deltaBackproj.norm();
        skewDeltaBackproj_over_lineLength = Skew(deltaBackproj)*lineLenghtInv; // Skew(deltaBackproj)/|deltaBackproj|
        deltaBackprojNormalized = deltaBackproj*lineLenghtInv;
    }

    // to be called after all the data have been filled up
    void init(){
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        const ImuCamPose& imuCamPose = VPose->estimate();

        const Eigen::Vector3d XSc = imuCamPose.ToCam(XSw,cam_idx); // S w.r.t. camera frame (aka P)
        const Eigen::Vector3d XEc = imuCamPose.ToCam(XEw,cam_idx); // E w.r.t. camera frame (aka Q)     
        
        // check if the end-points association is correct
        const Eigen::Vector3d mapLineSE = (XSc-XEc).normalized();
        const Eigen::Vector3d& lineSE = deltaBackprojNormalized; //(XSc_backproj-XEc_backproj).normalized();    
        if(mapLineSE.dot(lineSE) < 0)
        {
            //std::cout << "BA: swapping line endpoints" << std::endl;
            std::swap(XSc_backproj,XEc_backproj);
            deltaBackprojNormalized *= -1;
            skewDeltaBackproj_over_lineLength *= -1;            
        }    
    }

    bool areDepthsPositive() {
        const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]);         
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        return VPose->estimate().isDepthPositive(VLine->estimate().head(3),cam_idx) && 
               VPose->estimate().isDepthPositive(VLine->estimate().tail(3),cam_idx);        
    }    

    // compute total squared 3D error 
    double computeSquared3DError() {
        // compute full 3D error without information matrix
        computeError(); 
        return (_error[2]*_error[2] + _error[3]*_error[3]);
    }  

    void getMapLineAndProjections(Eigen::Vector3d& mapS, Eigen::Vector3d& mapE, Eigen::Vector2d& projS, Eigen::Vector2d& projE) {
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        const ImuCamPose& imuCamPose = VPose->estimate();

        mapS = imuCamPose.ToCam(XSw,cam_idx); // S w.r.t. camera frame (aka P)
        mapE = imuCamPose.ToCam(XEw,cam_idx); // E w.r.t. camera frame (aka Q)       
        projS = imuCamPose.pCamera[cam_idx]->projectLinear(mapS); // [us;vs] start point Sw cam projection
        projE = imuCamPose.pCamera[cam_idx]->projectLinear(mapE); // [ue;ve] end point Ew cam projection
    }

public:
    const int cam_idx;
    const Eigen::Vector3d XSw, XEw;  // start point and end point w.r.t. world frame 

    // the following block of data must be kept consistent 
    Eigen::Vector3d XSc_backproj; // backprojected start point w.r.t. camera frame 
    Eigen::Vector3d XEc_backproj; // backprojected end point w.r.t camera frame 
    Eigen::Vector3d deltaBackprojNormalized;     // (XSc_backproj - XEc_backproj).normalized()
    Eigen::Matrix3d skewDeltaBackproj_over_lineLength; // Skew(deltaBackproj)/|deltaBackproj|
    double lineLenghtInv; // inverse of backprojected line length 
};



} // namespace PLVS2
