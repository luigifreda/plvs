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

#include "OptimizableLineTypes.h"

namespace PLVS2 {


bool EdgeSE3ProjectLineOnlyPose::read(std::istream& is){
    for (int i=0; i<3; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
            information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectLineOnlyPose::write(std::ostream& os) const {

    for (int i=0; i<3; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++){
        os << " " <<  information()(i,j);
        }
    return os.good();
}

#if USE_ANALYTIC_JACS_ONLY_POSE 
void EdgeSE3ProjectLineOnlyPose::linearizeOplus() {

    g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    const Eigen::Vector3d XSc = vi->estimate().map(XSw); // XSc = Tcw * XSw
    const Eigen::Vector3d XEc = vi->estimate().map(XEw); // XEc = Tcw * XEw

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    // e2 = [eS] = [ [nx,ny]*projS -d ]
    //      [eE]   [ [nx,ny]*projE -d ]
    // projS = [us;vs] = cam projection of XSc
    // projE = [ue;ve] = cam projection of XEc

    // NOTE: [Luigi] below some notes about the derivation 
    // derivatives with left perturbation model
    // Tcw' = exp(csi)*Tcw  (new perturbed value of Tcw)
    // Xc' = Tcw' * Xw = exp(csi) * Tcw * Xw
    // SE3deriv = dXc'/d(csi) = d(exp(csi)*Tcw*Xw)/d(csi) = d(exp(csi)*Xc)/d(csi)=[-[Xc]^, I]  (computed at csi=0)
    // where Tcw' <-- exp(csi)*Tcw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                        [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 

    // _jacobianOplusXi = de2/d(csi) = [ deS/d(csi) ] = [deS/dXSc' * dXSc'/d(csi)]  \in IR^2x6   (computed at csi=0 => de/dXc' = de/dXc )
    //                                 [ deE/d(csi) ]   [deE/dXEc' * dXEc'/d(csi)] 
    // deS/dXSc' = [nx,ny] * dprojS/dXSc = [nx,ny] * proj_jac_S = JlineS
    // deE/dXEc' = [nx,ny] * dprojE/dXEc = [nx,ny] * proj_jac_E = JlineE 
    // dXSc'/d(csi) = [-[XSc]^, I] = SE3deriv_S
    // dXEc'/d(csi) = [-[XEc]^, I] = SE3deriv_E
    // => _jacobianOplusXi = de2/d(csi) = [ JlineS * SE3deriv_S ] 
    //                                    [ JlineE * SE3deriv_E ] 

    const Eigen::Matrix<double,2,3> proj_jac_S = pCamera->projectJacLinear(XSc); // d[us;vs]/dXSc
    const Eigen::Matrix<double,2,3> proj_jac_E = pCamera->projectJacLinear(XEc); // d[ue;ve]/dXEc 

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dXSc = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dXEc = [nx,ny]*proj_jac_E    

    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E;     

    const double xs = XSc(0);
    const double ys = XSc(1);
    const double zs = XSc(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[XSc]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEc(0);
    const double ye = XEc(1);
    const double ze = XEc(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[XEc]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;       

    _jacobianOplusXi.block<1,6>(0,0) = JlineS * SE3deriv_S;  // explanation in the above lines
    _jacobianOplusXi.block<1,6>(1,0) = JlineE * SE3deriv_E;                   

}
#endif


 
bool EdgeSE3ProjectLineOnlyPoseToBody::read(std::istream& is){
    for (int i=0; i<3; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
            information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectLineOnlyPoseToBody::write(std::ostream& os) const {

    for (int i=0; i<3; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
        for (int j=i; j<3; j++){
        os << " " <<  information()(i,j);
        }
    return os.good();
}

#if USE_ANALYTIC_JACS_ONLY_POSE_TO_BODY 
void EdgeSE3ProjectLineOnlyPoseToBody::linearizeOplus() {

    g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vi->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;    
    const auto Rrl = mTrl.rotation().toRotationMatrix();

    const Eigen::Vector3d XSl = Tlw.map(XSw); // XSl = Tlw * XSw
    const Eigen::Vector3d XEl = Tlw.map(XEw); // XEl = Tlw * XEw    

    const Eigen::Vector3d XSr = Trw.map(XSw); // XSr = Trw * XSw
    const Eigen::Vector3d XEr = Trw.map(XEw); // XEr = Trw * XEw

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    // e2 = [eS] = [ [nx,ny]*projS -d ]
    //      [eE]   [ [nx,ny]*projE -d ]
    // projS = [us;vs] = cam projection of XSr
    // projE = [ue;ve] = cam projection of XEr

    // NOTE: [Luigi] below some notes about the derivation 
    // derivatives with left perturbation model
    // Tlw' = exp(csi)*Tlw  (new perturbed value of Tlw)
    // Xr' = Trl * Tlw' * Xw = Trl * exp(csi) * Tlw * Xw
    // SE3deriv = dXl'/d(csi) = d(exp(csi)*Tlw*Xw)/d(csi) = d(exp(csi)*Xl)/d(csi)=[-[Xl]^, I]  (computed at csi=0)
    // where Tlw' <-- exp(csi)*Tlw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                        [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 

    // _jacobianOplusXi = de2/d(csi) = [ deS/d(csi) ] = [deS/dXSr' * dXSr'/dXSl' * dXSl'/d(csi)]  \in IR^2x6   (computed at csi=0 => de/dXr' = de/dXr )
    //                                 [ deE/d(csi) ]   [deE/dXEr' * dXRr'/dXEl' * dXEl'/d(csi)] 
    // deS/dXSr = [nx,ny] * dprojS/dXSr = [nx,ny] * proj_jac_S = JlineS
    // deE/dXEr = [nx,ny] * dprojE/dXEr = [nx,ny] * proj_jac_E = JlineE 
    // dXSr'/dXSl' = d(Rrl*XSl'+trl)/dXSl' = Rrl  
    // dXEr'/dXEl' = d(Rrl*XEl'+trl)/dXEl' = Rrl 
    // dXSl'/d(csi) = [-[XSl]^, I] = SE3deriv_S
    // dXEl'/d(csi) = [-[XEl]^, I] = SE3deriv_E
    // => _jacobianOplusXi = de2/d(csi) = [ JlineS * Rrl * SE3deriv_S ] 
    //                                    [ JlineE * Rrl * SE3deriv_E ] 

    const Eigen::Matrix<double,2,3> proj_jac_S = pCamera->projectJacLinear(XSr); // d[us;vs]/dXSr
    const Eigen::Matrix<double,2,3> proj_jac_E = pCamera->projectJacLinear(XEr); // d[ue;ve]/dXEr 

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dXSr = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dXEr = [nx,ny]*proj_jac_E    

    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E;     

    const double xs = XSl(0);
    const double ys = XSl(1);
    const double zs = XSl(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[XSc]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEl(0);
    const double ye = XEl(1);
    const double ze = XEl(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[XEc]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;       

    _jacobianOplusXi.block<1,6>(0,0) = JlineS * Rrl * SE3deriv_S;  // explanation in the above  lines
    _jacobianOplusXi.block<1,6>(1,0) = JlineE * Rrl * SE3deriv_E;                   

}
#endif




EdgeSE3ProjectLine::EdgeSE3ProjectLine() : g2o::BaseBinaryEdge<2, Eigen::Vector3d, g2o::VertexSBALine, g2o::VertexSE3Expmap>() {
}

bool EdgeSE3ProjectLine::read(std::istream& is){
    for (int i=0; i<3; i++){
    is >> _measurement[i];
    }
    for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
        information()(j,i)=information()(i,j);
    }
    return true;
}

bool EdgeSE3ProjectLine::write(std::ostream& os) const {
    for (int i=0; i<3; i++){
    os << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
        os << " " <<  information()(i,j);
    }
    return os.good();
}

#if USE_ANALYTIC_JACS_FULL
void EdgeSE3ProjectLine::linearizeOplus() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::SE3Quat& Tcw = vj->estimate();
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    const Eigen::Vector3d XSc = Tcw.map(vi->estimate().head<3>());  // XSc = Tcw * XSw
    const Eigen::Vector3d XEc = Tcw.map(vi->estimate().tail<3>());  // XEc = Tcw * XEw

    const auto Rcw = Tcw.rotation().toRotationMatrix();

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    // e2 = [eS] = [ [nx,ny]*projS -d ]
    //      [eE]   [ [nx,ny]*projE -d ]
    // projS = [us;vs] = cam projection of XSc
    // projE = [ue;ve] = cam projection of XEc

    // NOTE: [Luigi] below some notes about the derivation 
    // derivatives with left perturbation model
    // Tcw' = exp(csi)*Tcw  (new perturbed value of Tcw)
    // Xc' = Tcw' * Xw = exp(csi) * Tcw * Xw
    // SE3deriv = dXc'/d(csi) = d(exp(csi)*Tcw*Xw)/d(csi) = d(exp(csi)*Xc)/d(csi)=[-[Xc]^, I]  (computed at csi=0)
    // where Tcw' <-- exp(csi)*Tcw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                        [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 


    // _jacobianOplusXi = de2/d(Sw,Ew) = [ deS/d(XSw,XEw) ] = [ deS/dXSc * dXSc/dXSw,                    0 ]  \in IR^2x6
    //                                   [ deE/d(XSw,XEw) ]   [                    0,  deE/dXEc * dEc/dXEw ] 
    // where deS/dXSc = [nx,ny]*d[us;vs]/dXSc = [nx,ny]*proj_jac_S 
    //       dSc/dXSw=Rcw (since XSc=Rcw*XSw+tcw)
    //       deE/dXEc = [nx,ny]*d[ue;ve]/dXEc = [nx,ny]*proj_jac_E
    //       dEc/dXEw=Rcw (since XEc=Rcw*XEw+tcw)


    const Eigen::Matrix<double,2,3> proj_jac_S = pCamera->projectJacLinear(XSc); // d[us;vs]/dXSc
    const Eigen::Matrix<double,2,3> proj_jac_E = pCamera->projectJacLinear(XEc); // d[ue;ve]/dXEc 

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dXSc = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dXEc = [nx,ny]*proj_jac_E    

    _jacobianOplusXi.block<1,3>(0,0) =  JlineS * Rcw; // deS/dXSw 
    _jacobianOplusXi.block<1,3>(0,3) = Eigen::Matrix<double,1,3>::Zero(); 
    
    _jacobianOplusXi.block<1,3>(1,0) = Eigen::Matrix<double,1,3>::Zero();
    _jacobianOplusXi.block<1,3>(1,3) =  JlineE * Rcw; // deE/dXEw    


    // _jacobianOplusXj = de2/d(csi) = [ deS/d(csi) ] = [deS/dXSc' * dXSc'/d(csi)]  \in IR^2x6   (computed at csi=0 => de/dXc' = de/dXc )
    //                                 [ deE/d(csi) ]   [deE/dXEc' * dXEc'/d(csi)] 
    // deS/dXSc' = [nx,ny] * dprojS/dXSc = [nx,ny] * proj_jac_S = JlineS
    // deE/dXEc' = [nx,ny] * dprojE/dXEc = [nx,ny] * proj_jac_E = JlineE 
    // dXSc'/d(csi) = [-[XSc]^, I] = SE3deriv_S
    // dXEc'/d(csi) = [-[XEc]^, I] = SE3deriv_E
    // => _jacobianOplusXj = de2/d(csi) = [ JlineS * SE3deriv_S ] 
    //                                    [ JlineE * SE3deriv_E ] 

    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E;     

    const double xs = XSc(0);
    const double ys = XSc(1);
    const double zs = XSc(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[XSc]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEc(0);
    const double ye = XEc(1);
    const double ze = XEc(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[XEc]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;       

    _jacobianOplusXj.block<1,6>(0,0) = JlineS * SE3deriv_S;  // explanation in the above lines
    _jacobianOplusXj.block<1,6>(1,0) = JlineE * SE3deriv_E;   
 
}
#endif




EdgeSE3ProjectLineToBody::EdgeSE3ProjectLineToBody() : g2o::BaseBinaryEdge<2, Eigen::Vector3d, g2o::VertexSBALine, g2o::VertexSE3Expmap>() {
}

bool EdgeSE3ProjectLineToBody::read(std::istream& is){
    for (int i=0; i<3; i++){
    is >> _measurement[i];
    }
    for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
        information()(j,i)=information()(i,j);
    }
    return true;
}

bool EdgeSE3ProjectLineToBody::write(std::ostream& os) const {
    for (int i=0; i<3; i++){
    os << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
        os << " " <<  information()(i,j);
    }
    return os.good();
}

#if USE_ANALYTIC_JACS_FULL_TO_BODY
void EdgeSE3ProjectLineToBody::linearizeOplus() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;       
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    const Eigen::Vector3d XSw = vi->estimate().head<3>();  
    const Eigen::Vector3d XEw = vi->estimate().tail<3>();  

    const auto Rrw = Trw.rotation().toRotationMatrix();
    const auto Rrl = mTrl.rotation().toRotationMatrix();

    const Eigen::Vector3d XSl = Tlw.map(XSw); // XSl = Tlw * XSw
    const Eigen::Vector3d XEl = Tlw.map(XEw); // XEl = Tlw * XEw    

    const Eigen::Vector3d XSr = Trw.map(XSw); // XSr = Trw * XSw
    const Eigen::Vector3d XEr = Trw.map(XEw); // XEr = Trw * XEw

    // NOTE: 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    // e2 = [eS] = [ [nx,ny]*projS -d ]
    //      [eE]   [ [nx,ny]*projE -d ]
    // projS = [us;vs] = cam projection of XSr
    // projE = [ue;ve] = cam projection of XEr

    // NOTE: [Luigi] below some notes about the derivation 
    // derivatives with left perturbation model
    // Tlw' = exp(csi)*Tlw  (new perturbed value of Tlw)
    // Xr' = Trl * Tlw' * Xw = Trl * exp(csi) * Tlw * Xw
    // SE3deriv = dXl'/d(csi) = d(exp(csi)*Tlw*Xw)/d(csi) = d(exp(csi)*Xl)/d(csi)=[-[Xl]^, I]  (computed at csi=0)
    // where Tlw' <-- exp(csi)*Tlw, csi=(phi,rho) and exp(csi)=[exp(phi), J(phi)*rho] see Barfoot's book
    //                                                         [     O^T,          1]
    // J(phi) is the left jacobian of SO(3) which can be approx to I if phi is small 


    // _jacobianOplusXi = de2/d(Sw,Ew) = [ deS/d(XSw,XEw) ] = [ deS/dXSr * dXSr/dXSw,                     0 ]  \in IR^2x6
    //                                   [ deE/d(XSw,XEw) ]   [                    0,  deE/dXEr * dXEr/dXEw ] 
    // where deS/dXSr = [nx,ny]*d[us;vs]/dXSr = [nx,ny]*proj_jac_S 
    //       dXSr/dSw=Rrw (since XSr=Rrw*XSw+tcw)
    //       deE/dXEc = [nx,ny]*d[ue;ve]/dXEc = [nx,ny]*proj_jac_E
    //       dEc/dXEw=Rrw (since XEr=Rrw*XEw+tcw)

    const Eigen::Matrix<double,2,3> proj_jac_S = pCamera->projectJacLinear(XSr); // d[us;vs]/dXSr
    const Eigen::Matrix<double,2,3> proj_jac_E = pCamera->projectJacLinear(XEr); // d[ue;ve]/dXEr 

    const Eigen::Matrix<double,1,3> JlineS = _measurement.head(2).transpose() * proj_jac_S; // deS/dXSr = [nx,ny]*proj_jac_S
    const Eigen::Matrix<double,1,3> JlineE = _measurement.head(2).transpose() * proj_jac_E; // deE/dXEr = [nx,ny]*proj_jac_E    

    _jacobianOplusXi.block<1,3>(0,0) =  JlineS * Rrw; // deS/dXSw 
    _jacobianOplusXi.block<1,3>(0,3) = Eigen::Matrix<double,1,3>::Zero(); 
    
    _jacobianOplusXi.block<1,3>(1,0) = Eigen::Matrix<double,1,3>::Zero();
    _jacobianOplusXi.block<1,3>(1,3) =  JlineE * Rrw; // deE/dXEw    


    // _jacobianOplusXj = de2/d(csi) = [ deS/d(csi) ] = [deS/dXSr' * dXSr'/dXSl' * dXSl'/d(csi)]  \in IR^2x6   (computed at csi=0 => de/dXc' = de/dXc )
    //                                 [ deE/d(csi) ]   [deE/dXEr' * dXEr'/dXEl' * dXEl'/d(csi)] 
    // deS/dXSr = [nx,ny] * dprojS/dXSr = [nx,ny] * proj_jac_S = JlineS
    // deE/dXEr = [nx,ny] * dprojE/dXEr = [nx,ny] * proj_jac_E = JlineE 
    // dXSr'/dXSl' = d(Rrl*XSl'+trl)/dXSl' = Rrl  
    // dXEr'/dXEl' = d(Rrl*XEl'+trl)/dXEl' = Rrl     
    // dXSl'/d(csi) = [-[XSl]^, I] = SE3deriv_S
    // dXEl'/d(csi) = [-[XEl]^, I] = SE3deriv_E
    // => _jacobianOplusXj = de2/d(csi) = [ JlineS * Rrl * SE3deriv_S ] 
    //                                    [ JlineE * Rrl * SE3deriv_E ] 

    Eigen::Matrix<double,3,6> SE3deriv_S, SE3deriv_E;     

    const double xs = XSl(0);
    const double ys = XSl(1);
    const double zs = XSl(2);
    SE3deriv_S << 0.0,   zs, -ys,  1.0, 0.0, 0.0,  // [-[XSl]^, I]
                  -zs,  0.0,  xs,  0.0, 1.0, 0.0,
                   ys,  -xs, 0.0,  0.0, 0.0, 1.0;

    const double xe = XEl(0);
    const double ye = XEl(1);
    const double ze = XEl(2);
    SE3deriv_E << 0.0,   ze, -ye,  1.0, 0.0, 0.0,  // [-[XEl]^, I]
                  -ze,  0.0,  xe,  0.0, 1.0, 0.0,
                   ye,  -xe, 0.0,  0.0, 0.0, 1.0;       

    _jacobianOplusXj.block<1,6>(0,0) = JlineS * Rrl * SE3deriv_S;  // explanation in the above lines
    _jacobianOplusXj.block<1,6>(1,0) = JlineE * Rrl * SE3deriv_E;   
 
}
#endif

}
