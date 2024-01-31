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

#include "G2oLineTypes.h"
#include "ImuTypes.h"
#include "Converter.h"

namespace PLVS2
{

#if USE_ANALYTIC_JACS_FULL_MONO_IMU
void EdgeLineMono::linearizeOplus()
{
    const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]); // i
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);                 // j 

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const Eigen::Matrix3d &Rbc = VPose->estimate().Rbc[cam_idx];  
    const Eigen::Vector3d &tbc = VPose->estimate().tbc[cam_idx];      

    const Eigen::Vector3d XSc = Rcw*VLine->estimate().head(3) + tcw; // P w.r.t. camera frame (start point S)
    const Eigen::Vector3d XEc = Rcw*VLine->estimate().tail(3) + tcw; // Q w.r.t. camera frame (end point E)  

    const Eigen::Vector3d XSb = Rbc*XSc + tbc; // P w.r.t. body frame (start point S)
    const Eigen::Vector3d XEb = Rbc*XEc + tbc; // Q w.r.t. body frame (end point E)       

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    // e2 = [eS] = [ [nx,ny]*projS -d ]
    //      [eE]   [ [nx,ny]*projE -d ]

    // NOTE: [Luigi] below some notes about the derivation

    // projS = [us;vs] = cam projection of Sc
    // projE = [ue;ve] = cam projection of Ec
    const Eigen::Matrix<double,2,3> proj_jac_S = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XSc); // d[us;vs]/dSc
    const Eigen::Matrix<double,2,3> proj_jac_E = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XEc); // d[ue;ve]/dEc

    // _jacobianOplusXi = de2/d(Sw,Ew) = [ deS/d(Sw,Ew) ] = [ deS/dSc * dSc/dSw,                  0 ]  \in IR^2x6
    //                                   [ deE/d(Sw,Ew) ]   [                 0,  deE/dEc * dEc/dEw ] 
    // where deS/dSc = [nx,ny]*d[us;vs]/dSc = [nx,ny]*proj_jac_S 
    //       dSc/dSw=Rcw (since Sc=Rcw*Sw+tcw)
    //       deE/dEc = [nx,ny]*d[ue;ve]/dEc = [nx,ny]*proj_jac_E
    //       dEc/dEw=Rcw (since Ec=Rcw*Ew+tcw)

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dSc = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dEc = [nx,ny]*proj_jac_E   

    _jacobianOplusXi.block<1,3>(0,0) =  JlineS * Rcw; // deS/dSw 
    _jacobianOplusXi.block<1,3>(0,3) = Eigen::Matrix<double,1,3>::Zero(); 
    
    _jacobianOplusXi.block<1,3>(1,0) = Eigen::Matrix<double,1,3>::Zero();
    _jacobianOplusXi.block<1,3>(1,3) =  JlineE * Rcw; // deE/dEw    

    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E; 
    // derivatives with left perturbation model
    // SE3deriv=dPb/d(csi)=d(exp(csi)*Tbw*Pw)/d(csi)=d(exp(csi)*Pb)/d(csi)=[-[Pb]^, I]
    // where Tbw <-- exp(csi)*Tbw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                        [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 

    const double xs = XSb(0);
    const double ys = XSb(1);
    const double zs = XSb(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEb(0);
    const double ye = XEb(1);
    const double ze = XEb(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;                   

    // _jacobianOplusXj = de2/d(mu) = [ deS/d(mu) ] = [deS/dSc * dSc/dSb * dSb/d(mu)]  \in IR^2x6
    //                                [ deE/d(mu) ]   [deE/dEc * dEc/dEb * dEb/d(mu)] 
    _jacobianOplusXj.block<1,6>(0,0) = -JlineS * Rcb * SE3deriv_S;  // explanation in the following lines
    _jacobianOplusXj.block<1,6>(1,0) = -JlineE * Rcb * SE3deriv_E;
    // Note that VPose uses a right perturbation model with its paramenter mu:
    // Twb <-- Twb * exp(mu) (see ImuCamPose::Update())
    // This translates to a left perturbation model by inverting 
    // Tbw <-- exp(-mu) * Tbw   and noting that csi=-mu.
    // One has: dPb/d(mu)=d(exp(-mu)*Tbw)/d(mu)=-d(exp(csi)*Tbw*Pw)/d(csi)=-SE3deriv
    // This entails: 
    //   dSb/d(mu) = - SE3deriv_S
    //   dEb/d(mu) = - SE3deriv_E        
    //   deS/dSc = [nx,ny]*proj_jac_S, dSc/dSb=Rcb (since Sc=Rcb*Sb+tcw)
    //   deE/dEc = [nx,ny]*proj_jac_E, dEc/dEb=Rcb (since Ec=Rcb*Eb+tcw)    
    // Therefore:  _jacobianOplusXj = de2/d(mu) = [ -[nx,ny]*proj_jac_S * Rcb * SE3deriv_S ]
    //                                            [ -[nx,ny]*proj_jac_E * Rcb * SE3deriv_E ] 
}
#endif 

#if USE_ANALYTIC_JACS_ONLY_POSE_MONO_IMU
void EdgeLineMonoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const Eigen::Matrix3d &Rbc = VPose->estimate().Rbc[cam_idx];  
    const Eigen::Vector3d &tbc = VPose->estimate().tbc[cam_idx];

    const Eigen::Vector3d XSc = Rcw*XSw + tcw; // P w.r.t. camera frame (start point S)
    const Eigen::Vector3d XEc = Rcw*XEw + tcw; // Q w.r.t. camera frame (end point E)  

    const Eigen::Vector3d XSb = Rbc*XSc + tbc; // P w.r.t. body frame (start point S)
    const Eigen::Vector3d XEb = Rbc*XEc + tbc; // Q w.r.t. body frame (end point E)       

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    // e2 = [eS] = [ [nx,ny]*projS -d ]
    //      [eE]   [ [nx,ny]*projE -d ]

    // NOTE: [Luigi] below some notes about the derivation

    // projS = [us;vs] = cam projection of Sc
    // projE = [ue;ve] = cam projection of Ec
    const Eigen::Matrix<double,2,3> proj_jac_S = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XSc); // d[ue;ve]/dEc
    const Eigen::Matrix<double,2,3> proj_jac_E = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XEc); // d[ue;ve]/dEc 

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dSc = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dEc = [nx,ny]*proj_jac_E    

    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E; 
    // derivatives with left perturbation model
    // SE3deriv=dPb/d(csi)=d(exp(csi)*Tbw*Pw)/d(csi)=d(exp(csi)*Pb)/d(csi)=[-[Pb]^, I]
    // where Tbw <-- exp(csi)*Tbw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                        [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 

    const double xs = XSb(0);
    const double ys = XSb(1);
    const double zs = XSb(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEb(0);
    const double ye = XEb(1);
    const double ze = XEb(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;                   

    // _jacobianOplusXi = de2/d(mu) = [ deS/d(mu) ] = [deS/dSc * dSc/dSb * dSb/d(mu)]  \in IR^2x6
    //                                [ deE/d(mu) ]   [deE/dEc * dEc/dEb * dSb/d(mu)] 
    _jacobianOplusXi.block<1,6>(0,0) = -JlineS * Rcb * SE3deriv_S;  // explanation in the following lines
    _jacobianOplusXi.block<1,6>(1,0) = -JlineE * Rcb * SE3deriv_E;
    // Note that VPose uses a right perturbation model with its paramenter mu:
    // Twb <-- Twb * exp(mu) (see ImuCamPose::Update())
    // This translates to a left perturbation model by inverting 
    // Tbw <-- exp(-mu) * Tbw   and noting that csi=-mu.
    // One has: dPb/d(mu)=d(exp(-mu)*Tbw)/d(mu)=-d(exp(csi)*Tbw*Pw)/d(csi)=-SE3deriv
    // This entails: 
    //   dSb/d(mu) = - SE3deriv_S
    //   dEb/d(mu) = - SE3deriv_E        
    //   deS/dSc = [nx,ny]*proj_jac_S, dSc/dSb=Rcb (since Sc=Rcb*Sb+tcw)
    //   deE/dEc = [nx,ny]*proj_jac_E, dEc/dEb=Rcb (since Ec=Rcb*Eb+tcw)       
    // Therefore:  _jacobianOplusXi = de2/d(mu) = [ -[nx,ny]*proj_jac_S * Rcb * SE3deriv_S ]
    //                                            [ -[nx,ny]*proj_jac_E * Rcb * SE3deriv_E ] 
}
#endif 


#if USE_ANALYTIC_JACS_FULL_STEREO_IMU
void EdgeLineStereo::linearizeOplus()
{
    const g2o::VertexSBALine* VLine = static_cast<const g2o::VertexSBALine*>(_vertices[0]); // i
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);                 // j 

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const Eigen::Matrix3d &Rbc = VPose->estimate().Rbc[cam_idx];  
    const Eigen::Vector3d &tbc = VPose->estimate().tbc[cam_idx];      

    const Eigen::Vector3d XSc = Rcw*VLine->estimate().head(3) + tcw; // P w.r.t. camera frame (start point S)
    const Eigen::Vector3d XEc = Rcw*VLine->estimate().tail(3) + tcw; // Q w.r.t. camera frame (end point E)  

    const Eigen::Vector3d XSb = Rbc*XSc + tbc; // P w.r.t. body frame (start point S)
    const Eigen::Vector3d XEb = Rbc*XEc + tbc; // Q w.r.t. body frame (end point E)       

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    //      [eS]   [ [nx,ny]*projS -d                                 ]
    // e4 = [eE] = [ [nx,ny]*projE -d                                 ]  
    //      [dS]   [ |(Sc-Bs) X (Sc-Be)|/|Bs-Be| + muWeigth * |Sc-Bs| ]
    //      [dE]   [ |(Ec-Bs) X (Ec-Be)|/|Bs-Be| + muWeigth * |Ec-Be| ]
    // where Bs = backproj(s,ds) and Be = backproj(e,de), and s,e are the 2D line observation endpoints on the image plane 
    // and ds and de are their depths respectively

    // NOTE: [Luigi] below some notes about the derivation

    // projS = [us;vs] = cam projection of Sc
    // projE = [ue;ve] = cam projection of Ec
    const Eigen::Matrix<double,2,3> proj_jac_S = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XSc); // d[us;vs]/dSc
    const Eigen::Matrix<double,2,3> proj_jac_E = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XEc); // d[ue;ve]/dEc

    // _jacobianOplusXi = de4/d(Sw,Ew) = [ deS/d(Sw,Ew)   ] = [   deS/dSc * dSc/dSw,                  0  ]  \in IR^4x6
    //                                   [ deE/d(Sw,Ew)   ]   [                   0,  deE/dEc * dEc/dEw  ] 
    //                                   [ d(dS)/d(Sw,Ew) ]   [ d(dS)/dSc * dSc/dSw,                  0  ]
    //                                   [ d(dE)/d(Sw,Ew) ]   [                   0,  d(dE)/dEc * dEc/dEw ]
    // where 
    //  deS/dSc = [nx,ny]*d[us;vs]/dSc = [nx,ny]*proj_jac_S  
    //  dSc/dSw = Rcw (since Sc=Rcw*Sw+tcw)
    //  deE/dEc = [nx,ny]*d[ue;ve]/dEc = [nx,ny]*proj_jac_E
    //  dEc/dEw = Rcw (since Ec=Rcw*Ew+tcw)
    // Let's define: 
    //  deltaB = Bs-Be, Vs = (Sc-Bs) X (Sc-Be), Ve = (Ec-Bs) X (Ec-Be)
    // Following the PLVS report, it is:
    // d(dS)/dSc = - Vs^T/|Vs| * skew(deltaB)/|deltaB| + muWeigth * (Sc-Bs)^T/|Sc-Bs|
    // d(dE)/dEc = - Ve^T/|Ve| * skew(deltaB)/|deltaB| + muWeigth * (Ec-Be)^T/|Ec-Be|    

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dSc = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dEc = [nx,ny]*proj_jac_E   

    const Eigen::Vector3d Sc_Bs = XSc - XSc_backproj; // Sc-Bs
    const Eigen::Vector3d Ec_Be = XEc - XEc_backproj; // Ec-Be
    const Eigen::Vector3d Vs = Sc_Bs.cross(XSc - XEc_backproj);   //  (Sc-Bs) X (Sc-Be) 
    const Eigen::Vector3d Ve = (XEc - XSc_backproj).cross(Ec_Be); //  (Ec-Bs) X (Ec-Be)

    // JdS = d(dS)/dSc = - Vs^T/|Vs| * skew(deltaB)/|deltaB| + muWeigth * (Sc-Bs)^T/|Sc-Bs|
    const Eigen::Matrix<double,1,3> JdS = -(Vs.transpose()/Vs.norm())*skewDeltaBackproj_over_lineLength + muWeigth*Sc_Bs.transpose()/Sc_Bs.norm();
    // JdE = d(dE)/dEc = - Ve^T/|Ve| * skew(deltaB)/|deltaB| + muWeigth * (Ec-Be)^T/|Ec-Be|
    const Eigen::Matrix<double,1,3> JdE = -(Ve.transpose()/Ve.norm())*skewDeltaBackproj_over_lineLength + muWeigth*Ec_Be.transpose()/Ec_Be.norm(); 

    // _jacobianOplusXi = de4/d(Sw,Ew) \in IR^4x6
    _jacobianOplusXi.block<1,3>(0,0) =  JlineS * Rcw; // deS/dSw = deS/dSc * dSc/dSw
    _jacobianOplusXi.block<1,3>(0,3) = Eigen::Matrix<double,1,3>::Zero(); 
    
    _jacobianOplusXi.block<1,3>(1,0) = Eigen::Matrix<double,1,3>::Zero();
    _jacobianOplusXi.block<1,3>(1,3) =  JlineE * Rcw; // deE/dEw = deE/dEc * dEc/dEw 

    _jacobianOplusXi.block<1,3>(2,0) =  JdS * Rcw; // d(dS)/dSw = d(dS)/dSc * dSc/dSw
    _jacobianOplusXi.block<1,3>(2,3) = Eigen::Matrix<double,1,3>::Zero();     

    _jacobianOplusXi.block<1,3>(3,0) = Eigen::Matrix<double,1,3>::Zero();     
    _jacobianOplusXi.block<1,3>(3,3) =  JdE * Rcw; // d(dE)/dEw = d(dE)/dEc * dEc/dEw 


    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E; 
    // derivatives with left perturbation model
    // SE3deriv=dPb/d(csi)=d(exp(csi)*Tbw*Pw)/d(csi)=d(exp(csi)*Pb)/d(csi)=[-[Pb]^, I]
    // where Tbw <-- exp(csi)*Tbw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                        [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 

    const double xs = XSb(0);
    const double ys = XSb(1);
    const double zs = XSb(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEb(0);
    const double ye = XEb(1);
    const double ze = XEb(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;                   

    // _jacobianOplusXj = de4/d(mu) = [ deS/d(mu)   ] = [deS/dSc * dSc/dSb * dSb/d(mu)   ]  \in IR^4x6
    //                                [ deE/d(mu)   ]   [deE/dEc * dEc/dEb * dSb/d(mu)   ] 
    //                                [ d(dS)/d(mu) ]   [d(dS)/dSc * dSc/dSb * dSb/d(mu) ]
    //                                [ d(dE)/d(mu) ]   [d(dE)/dEc * dSc/dSb * dEb/d(mu) ]    
    // _jacobianOplusXj.block<1,6>(0,0) = -JlineS * Rcb * SE3deriv_S;  // explanation in the following lines
    // _jacobianOplusXj.block<1,6>(1,0) = -JlineE * Rcb * SE3deriv_E;
    // _jacobianOplusXj.block<1,6>(2,0) = -JdS * Rcb * SE3deriv_S;
    // _jacobianOplusXj.block<1,6>(3,0) = -JdE * Rcb * SE3deriv_E;
    const Eigen::Matrix<double,3,6> Rcb_x_SE3deriv_S = Rcb * SE3deriv_S;
    const Eigen::Matrix<double,3,6> Rcb_x_SE3deriv_E = Rcb * SE3deriv_E; 
    _jacobianOplusXj.block<1,6>(0,0) = -JlineS * Rcb_x_SE3deriv_S;  // explanation in the following lines
    _jacobianOplusXj.block<1,6>(1,0) = -JlineE * Rcb_x_SE3deriv_E;
    _jacobianOplusXj.block<1,6>(2,0) = -JdS * Rcb_x_SE3deriv_S;
    _jacobianOplusXj.block<1,6>(3,0) = -JdE * Rcb_x_SE3deriv_E;    

    // Note that VPose uses a right perturbation model with its paramenter mu:
    // Twb <-- Twb * exp(mu) (see ImuCamPose::Update())
    // This translates to a left perturbation model by inverting 
    // Tbw <-- exp(-mu) * Tbw and noting that csi=-mu.
    // One has: dPb/d(mu)=d(exp(-mu)*Tbw)/d(mu)=-d(exp(csi)*Tbw*Pw)/d(csi)=-SE3deriv
    // This entails: 
    //  dSb/d(mu) = - SE3deriv_S
    //  dEb/d(mu) = - SE3deriv_E               
    //  deS/dSc = [nx,ny]*d[us;vs]/dSc = [nx,ny]*proj_jac_S = JlineS
    //  dSc/dSb = Rcb (since Sc=Rcb*Sb+tcw)
    //  deE/dEc = [nx,ny]*d[ue;ve]/dEc = [nx,ny]*proj_jac_E = JlineE
    //  dEc/dEb = Rcb (since Ec=Rcb*Eb+tcw)    
    // d(dS)/dSc = - Vs^T/|Vs| * skew(deltaB)/|deltaB| + muWeigth * (S-Bs)^T/|S-Bs| = JdS
    // d(dE)/dEc = - Ve^T/|Ve| * skew(deltaB)/|deltaB| + muWeigth * (E-Be)^T/|E-Be| = JdE     
    // Therefore:  _jacobianOplusXj = de4/d(mu) = [ deS/d(mu)   ] = [ -JlineS * Rcb * SE3deriv_S ]
    //                                            [ deE/d(mu)   ]   [ -JlineE * Rcb * SE3deriv_E ] 
    //                                            [ d(dS)/d(mu) ]   [ -JdS * Rcb * SE3deriv_S                ]
    //                                            [ d(dE)/d(mu) ]   [ -JdE * Rcb * SE3deriv_E                ]


}
#endif 


#if USE_ANALYTIC_JACS_ONLY_POSE_STEREO_IMU
void EdgeLineStereoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);   

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];
    const Eigen::Matrix3d &Rbc = VPose->estimate().Rbc[cam_idx];  
    const Eigen::Vector3d &tbc = VPose->estimate().tbc[cam_idx];

    const Eigen::Vector3d XSc = Rcw*XSw + tcw; // P w.r.t. camera frame (start point S)
    const Eigen::Vector3d XEc = Rcw*XEw + tcw; // Q w.r.t. camera frame (end point E)  

    const Eigen::Vector3d XSb = Rbc*XSc + tbc; // P w.r.t. body frame (start point S)
    const Eigen::Vector3d XEb = Rbc*XEc + tbc; // Q w.r.t. body frame (end point E)      

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    //      [eS]   [ [nx,ny]*projS -d            ]
    // e4 = [eE] = [ [nx,ny]*projE -d            ]  
    //      [dS]   [ |(Sc-Bs) X (Sc-Be)|/|Bs-Be| ]
    //      [dE]   [ |(Ec-Bs) X (Ec-Be)|/|Bs-Be| ]
    // where Bs = backproj(s,ds) and Be = backproj(e,de) and s,e are the 2D line observation endpoints on the image plane 
    // and ds and de are their depths respectively

    // NOTE: [Luigi] below some notes about the derivation

    // projS = [us;vs] = cam projection of Sc
    // projE = [ue;ve] = cam projection of Ec
    const Eigen::Matrix<double,2,3> proj_jac_S = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XSc); // d[us;vs]/dSc
    const Eigen::Matrix<double,2,3> proj_jac_E = VPose->estimate().pCamera[cam_idx]->projectJacLinear(XEc); // d[ue;ve]/dEc

    // deS/dSc = [nx,ny]*d[us;vs]/dSc = [nx,ny]*proj_jac_S  
    // deE/dEc = [nx,ny]*d[ue;ve]/dEc = [nx,ny]*proj_jac_E
    // Let's define: 
    //  deltaB = Bs-Be, Vs = (Sc-Bs) X (Sc-Be), Ve = (Ec-Bs) X (Ec-Be)
    // Following the PLVS report, it is:
    // d(dS)/dSc = - Vs^T/|Vs| * skew(deltaB)/|deltaB| 
    // d(dE)/dEc = - Ve^T/|Ve| * skew(deltaB)/|deltaB|   

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dSc = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dEc = [nx,ny]*proj_jac_E   

    const Eigen::Vector3d Sc_Bs = XSc - XSc_backproj; // Sc-Bs
    const Eigen::Vector3d Ec_Be = XEc - XEc_backproj; // Ec-Be
    const Eigen::Vector3d Vs = Sc_Bs.cross(XSc - XEc_backproj);   //  (Sc-Bs) X (Sc-Be) 
    const Eigen::Vector3d Ve = (XEc - XSc_backproj).cross(Ec_Be); //  (Ec-Bs) X (Ec-Be)

    // JdS = d(dS)/dSc = - Vs^T/|Vs| * skew(deltaB)/|deltaB|
    const Eigen::Matrix<double,1,3> JdS = -(Vs.transpose()/Vs.norm())*skewDeltaBackproj_over_lineLength;
    // JdE = d(dE)/dEc = - Ve^T/|Ve| * skew(deltaB)/|deltaB| 
    const Eigen::Matrix<double,1,3> JdE = -(Ve.transpose()/Ve.norm())*skewDeltaBackproj_over_lineLength; 


    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E; 
    // derivatives with left perturbation model
    // SE3deriv=dPb/d(csi)=d(exp(csi)*Tbw*Pw)/d(csi)=d(exp(csi)*Pb)/d(csi)=[-[Pb]^, I]
    // where Tbw <-- exp(csi)*Tbw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                        [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 

    const double xs = XSb(0);
    const double ys = XSb(1);
    const double zs = XSb(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEb(0);
    const double ye = XEb(1);
    const double ze = XEb(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[Xb]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;                   

    // _jacobianOplusXi = de4/d(mu) = [ deS/d(mu)   ] = [deS/dSc * dSc/dSb * dSb/d(mu)   ]  \in IR^4x6
    //                                [ deE/d(mu)   ]   [deE/dEc * dEc/dEb * dSb/d(mu)   ] 
    //                                [ d(dS)/d(mu) ]   [d(dS)/dSc * dSc/dSb * dSb/d(mu) ]
    //                                [ d(dE)/d(mu) ]   [d(dE)/dEc * dSc/dSb * dEb/d(mu) ]    
    // _jacobianOplusXi.block<1,6>(0,0) = -JlineS * Rcb * SE3deriv_S;  // explanation in the following lines
    // _jacobianOplusXi.block<1,6>(1,0) = -JlineE * Rcb * SE3deriv_E;
    // _jacobianOplusXi.block<1,6>(2,0) = -JdS * Rcb * SE3deriv_S;
    // _jacobianOplusXi.block<1,6>(3,0) = -JdE * Rcb * SE3deriv_E;
    const Eigen::Matrix<double,3,6> Rcb_x_SE3deriv_S = Rcb * SE3deriv_S;
    const Eigen::Matrix<double,3,6> Rcb_x_SE3deriv_E = Rcb * SE3deriv_E;    
    _jacobianOplusXi.block<1,6>(0,0) = -JlineS * Rcb_x_SE3deriv_S;  // explanation in the following lines
    _jacobianOplusXi.block<1,6>(1,0) = -JlineE * Rcb_x_SE3deriv_E;
    _jacobianOplusXi.block<1,6>(2,0) = -JdS * Rcb_x_SE3deriv_S;
    _jacobianOplusXi.block<1,6>(3,0) = -JdE * Rcb_x_SE3deriv_E;     
    // Note that VPose uses a right perturbation model with its paramenter mu:
    // Twb <-- Twb * exp(mu) (see ImuCamPose::Update())
    // This translates to a left perturbation model by inverting 
    // Tbw <-- exp(-mu) * Tbw  and noting that csi=-mu.
    // One has: dPb/d(mu)=d(exp(-mu)*Tbw)/d(mu)=-d(exp(csi)*Tbw*Pw)/d(csi)=-SE3deriv
    // This entails: 
    //  dSb/d(mu) = - SE3deriv_S
    //  dEb/d(mu) = - SE3deriv_E               
    //  deS/dSc = [nx,ny]*d[us;vs]/dSc = [nx,ny]*proj_jac_S = JlineS 
    //  dSc/dSb = Rcb (since Sc=Rcb*Sb+tcw)
    //  deE/dEc = [nx,ny]*d[ue;ve]/dEc = [nx,ny]*proj_jac_E = JlineE 
    //  dEc/dEb = Rcb (since Ec=Rcb*Eb+tcw)    
    // d(dS)/dSc = - Vs^T/|Vs| * skew(deltaB)/|deltaB| = JdS
    // d(dE)/dEc = - Ve^T/|Ve| * skew(deltaB)/|deltaB| = JdE     
    // Therefore:  _jacobianOplusXi = de4/d(mu) = [ deS/d(mu)   ] = [ -JlineS * Rcb * SE3deriv_S ]
    //                                            [ deE/d(mu)   ]   [ -JlineE * Rcb * SE3deriv_E ] 
    //                                            [ d(dS)/d(mu) ]   [ -JdS * Rcb * SE3deriv_S                ]
    //                                            [ d(dE)/d(mu) ]   [ -JdE * Rcb * SE3deriv_E                ]

}

#endif 

}
