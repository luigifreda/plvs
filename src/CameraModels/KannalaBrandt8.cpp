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
/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "KannalaBrandt8.h"
#include "Frame.h"

#include <boost/serialization/export.hpp>

//BOOST_CLASS_EXPORT_IMPLEMENT(PLVS2::KannalaBrandt8)

namespace PLVS2 {
//BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

    cv::Point2f KannalaBrandt8::project(const cv::Point3f &p3D) const {
        const float x2_plus_y2 = p3D.x * p3D.x + p3D.y * p3D.y;
        const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z);
        const float psi = atan2f(p3D.y, p3D.x);

        const float theta2 = theta * theta;
        const float theta3 = theta * theta2;
        const float theta5 = theta3 * theta2;
        const float theta7 = theta5 * theta2;
        const float theta9 = theta7 * theta2;
        const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                        + mvParameters[6] * theta7 + mvParameters[7] * theta9;

        return cv::Point2f(mvParameters[0] * r * cos(psi) + mvParameters[2],
                           mvParameters[1] * r * sin(psi) + mvParameters[3]);

    }

    Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &v3D) const {
        const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
        const double theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
        const double psi = atan2f(v3D[1], v3D[0]);

        const double theta2 = theta * theta;
        const double theta3 = theta * theta2;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        const double theta9 = theta7 * theta2;
        const double r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                        + mvParameters[6] * theta7 + mvParameters[7] * theta9;

        Eigen::Vector2d res;
        res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
        res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

        return res;

    }

    Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f &v3D) const {
        const float x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
        const float theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
        const float psi = atan2f(v3D[1], v3D[0]);

        const float theta2 = theta * theta;
        const float theta3 = theta * theta2;
        const float theta5 = theta3 * theta2;
        const float theta7 = theta5 * theta2;
        const float theta9 = theta7 * theta2;
        const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                         + mvParameters[6] * theta7 + mvParameters[7] * theta9;

        Eigen::Vector2f res;
        res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
        res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

        return res;

        /*cv::Point2f cvres = this->project(cv::Point3f(v3D[0],v3D[1],v3D[2]));

        Eigen::Vector2d res;
        res[0] = cvres.x;
        res[1] = cvres.y;

        return res;*/
    }

    Eigen::Vector2f KannalaBrandt8::projectMat(const cv::Point3f &p3D) const {
        cv::Point2f point = this->project(p3D);
        return Eigen::Vector2f(point.x, point.y);
    }

    float KannalaBrandt8::uncertainty2(const Eigen::Matrix<double,2,1> &p2D) const 
    {
        /*Eigen::Matrix<double,2,1> c;
        c << mvParameters[2], mvParameters[3];
        if ((p2D-c).squaredNorm()>57600) // 240*240 (256)
            return 100.f;
        else
            return 1.0f;*/
        return 1.f;
    }

    Eigen::Vector3f KannalaBrandt8::unprojectEig(const cv::Point2f &p2D) const {
        cv::Point3f ray = this->unproject(p2D);
        return Eigen::Vector3f(ray.x, ray.y, ray.z);
    }

    cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &p2D) const {
        //Use Newthon method to solve for theta with good precision (err ~ e-6)
        cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1]);
        float scale = 1.f;
        float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
        theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

        if (theta_d > 1e-8) {
            //Compensate distortion iteratively
            float theta = theta_d;

            for (int j = 0; j < 10; j++) {
                float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 =
                        theta4 * theta4;
                float k0_theta2 = mvParameters[4] * theta2, k1_theta4 = mvParameters[5] * theta4;
                float k2_theta6 = mvParameters[6] * theta6, k3_theta8 = mvParameters[7] * theta8;
                float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                  (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
                theta = theta - theta_fix;
                if (fabsf(theta_fix) < precision)
                    break;
            }
            //scale = theta - theta_d;
            scale = std::tan(theta) / theta_d;
        }

        return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
    }

    Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(const Eigen::Vector3d &v3D) const {
        double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
        double r2 = x2 + y2;
        double r = sqrt(r2);
        double r3 = r2 * r;
        double theta = atan2(r, v3D[2]);

        double theta2 = theta * theta, theta3 = theta2 * theta;
        double theta4 = theta2 * theta2, theta5 = theta4 * theta;
        double theta6 = theta2 * theta4, theta7 = theta6 * theta;
        double theta8 = theta4 * theta4, theta9 = theta8 * theta;

        double f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
                  theta9 * mvParameters[7];
        double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
                   9 * mvParameters[7] * theta8;

        Eigen::Matrix<double, 2, 3> JacGood;
        JacGood(0, 0) = mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
        JacGood(1, 0) =
                mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

        JacGood(0, 1) =
                mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
        JacGood(1, 1) = mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

        JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);
        JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);

        return JacGood;
    }

    bool KannalaBrandt8::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                                 Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated){
        if(!tvr){
            Eigen::Matrix3f K = this->toLinearK_();
            tvr = new TwoViewReconstruction(K);
        }

        //Correct FishEye distortion
        std::vector<cv::KeyPoint> vKeysUn1 = vKeys1, vKeysUn2 = vKeys2;
        std::vector<cv::Point2f> vPts1(vKeys1.size()), vPts2(vKeys2.size());

        for(size_t i = 0; i < vKeys1.size(); i++) vPts1[i] = vKeys1[i].pt;
        for(size_t i = 0; i < vKeys2.size(); i++) vPts2[i] = vKeys2[i].pt;

        const cv::Mat D = (cv::Mat_<float>(4,1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
        const cv::Mat R = cv::Mat::eye(3,3,CV_32F);
        const cv::Mat K = this->toK();
        const cv::Mat LinearK = this->toLinearK();
        cv::fisheye::undistortPoints(vPts1,vPts1,K,D,R,LinearK);
        cv::fisheye::undistortPoints(vPts2,vPts2,K,D,R,LinearK);

        for(size_t i = 0; i < vKeys1.size(); i++) vKeysUn1[i].pt = vPts1[i];
        for(size_t i = 0; i < vKeys2.size(); i++) vKeysUn2[i].pt = vPts2[i];

        return tvr->Reconstruct(vKeysUn1,vKeysUn2,vMatches12,T21,vP3D,vbTriangulated);
    }


    cv::Mat KannalaBrandt8::toK() const {
        cv::Mat K = (cv::Mat_<float>(3, 3)
                << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
        return K;
    }
    Eigen::Matrix3f KannalaBrandt8::toK_() const {
        Eigen::Matrix3f K;
        K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f;
        return K;
    }


    bool KannalaBrandt8::epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                           const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc) {
        Eigen::Vector3f p3D;
        return this->TriangulateMatches(pCamera2,kp1,kp2,R12,t12,sigmaLevel,unc,p3D) > 0.0001f;
    }

    bool KannalaBrandt8::matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                             Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                                             const float sigmaLevel1, const float sigmaLevel2,
                                             Eigen::Vector3f& x3Dtriangulated){
        Eigen::Matrix<float,3,4> eigTcw1 = Tcw1.matrix3x4();
        Eigen::Matrix3f Rcw1 = eigTcw1.block<3,3>(0,0);
        Eigen::Matrix3f Rwc1 = Rcw1.transpose();
        Eigen::Matrix<float,3,4> eigTcw2 = Tcw2.matrix3x4();
        Eigen::Matrix3f Rcw2 = eigTcw2.block<3,3>(0,0);
        Eigen::Matrix3f Rwc2 = Rcw2.transpose();

        cv::Point3f ray1c = this->unproject(kp1.pt);
        cv::Point3f ray2c = pOther->unproject(kp2.pt);

        Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
        Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);

        //Check parallax between rays
        Eigen::Vector3f ray1 = Rwc1 * r1;
        Eigen::Vector3f ray2 = Rwc2 * r2;

        const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

        //If parallax is lower than 0.9998, reject this match
        if(cosParallaxRays > 0.9998){
            return false;
        }

        //Parallax is good, so we try to triangulate
        cv::Point2f p11,p22;

        p11.x = ray1c.x;
        p11.y = ray1c.y;

        p22.x = ray2c.x;
        p22.y = ray2c.y;

        Eigen::Vector3f x3D;

        Triangulate(p11,p22,eigTcw1,eigTcw2,x3D);

        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3D)+Tcw1.translation()(2);
        if(z1<=0){  //Point is not in front of the first camera
            return false;
        }


        float z2 = Rcw2.row(2).dot(x3D)+Tcw2.translation()(2);
        if(z2<=0){ //Point is not in front of the first camera
            return false;
        }

        //Check reprojection error in first keyframe
        //  -Transform point into camera reference system
        Eigen::Vector3f x3D1 = Rcw1 * x3D + Tcw1.translation();
        Eigen::Vector2f uv1 = this->project(x3D1);

        float errX1 = uv1(0) - kp1.pt.x;
        float errY1 = uv1(1) - kp1.pt.y;

        if((errX1*errX1+errY1*errY1)>5.991*sigmaLevel1){   //Reprojection error is high
            return false;
        }

        //Check reprojection error in second keyframe;
        //  -Transform point into camera reference system
        Eigen::Vector3f x3D2 = Rcw2 * x3D + Tcw2.translation(); // avoid using q
        Eigen::Vector2f uv2 = pOther->project(x3D2);

        float errX2 = uv2(0) - kp2.pt.x;
        float errY2 = uv2(1) - kp2.pt.y;

        if((errX2*errX2+errY2*errY2)>5.991*sigmaLevel2){   //Reprojection error is high
            return false;
        }

        //Since parallax is big enough and reprojection errors are low, this pair of points
        //can be considered as a match
        x3Dtriangulated = x3D;

        return true;
    }

    float KannalaBrandt8::TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc, Eigen::Vector3f& p3D) {

        Eigen::Vector3f r1 = this->unprojectEig(kp1.pt);
        Eigen::Vector3f r2 = pCamera2->unprojectEig(kp2.pt);

        //Check parallax
        Eigen::Vector3f r21 = R12 * r2;

        const float cosParallaxRays = r1.dot(r21)/(r1.norm() *r21.norm());

        if(cosParallaxRays > 0.9998){
            return -1;
        }

        //Parallax is good, so we try to triangulate
        cv::Point2f p11,p22;

        p11.x = r1[0];
        p11.y = r1[1];

        p22.x = r2[0];
        p22.y = r2[1];

        Eigen::Vector3f x3D;
        Eigen::Matrix<float,3,4> Tcw1;
        Tcw1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

        Eigen::Matrix<float,3,4> Tcw2;

        Eigen::Matrix3f R21 = R12.transpose();
        Tcw2 << R21, -R21 * t12;


        Triangulate(p11,p22,Tcw1,Tcw2,x3D);
        // cv::Mat x3Dt = x3D.t();

        float z1 = x3D(2);
        if(z1 <= 0){
            return -2;
        }

        float z2 = R21.row(2).dot(x3D)+Tcw2(2,3);
        if(z2<=0){
            return -3;
        }

        //Check reprojection error
        Eigen::Vector2f uv1 = this->project(x3D);

        float errX1 = uv1(0) - kp1.pt.x;
        float errY1 = uv1(1) - kp1.pt.y;

        if((errX1*errX1+errY1*errY1)>5.991 * sigmaLevel){   //Reprojection error is high
            return -4;
        }

        Eigen::Vector3f x3D2 = R21 * x3D + Tcw2.col(3);
        Eigen::Vector2f uv2 = pCamera2->project(x3D2);

        float errX2 = uv2(0) - kp2.pt.x;
        float errY2 = uv2(1) - kp2.pt.y;

        if((errX2*errX2+errY2*errY2)>5.991 * unc){   //Reprojection error is high
            return -5;
        }

        p3D = x3D;

        return z1;
    }

    // we assume the keylines are undistorted
    bool KannalaBrandt8::TriangulateLineMatches(GeometricCamera* pCamera2, const cv::line_descriptor_c::KeyLine& kl1, const cv::line_descriptor_c::KeyLine& kl2,  const float sigma1, const float sigma2, const CameraPairTriangulationInput& camPairdata, LineTriangulationOutput& outData)
    {
        outData.isValid = false; 

        const auto& K1 = camPairdata.K1; 
        const auto& K2 = camPairdata.K2;
        const auto& minZ = camPairdata.minZ;
        const auto& maxZ = camPairdata.maxZ;

        const float invfx1 = 1.0f/K1(0,0);
        const float invfy1 = 1.0f/K1(1,1);
        const float& cx1 = K1(0,2);
        const float& cy1 = K1(1,2);

#define KEYLINES_ARE_DISTORTED 0

#if KEYLINES_ARE_DISTORTED
        // unproject on the plane Z=1 
        const Eigen::Vector3f Bp1 = this->unprojectEig(cv::Point2f(kl1.startPointX,kl1.startPointY));
        const Eigen::Vector3f Bq1 = this->unprojectEig(cv::Point2f(kl1.endPointX,kl1.endPointY));

        // undistort the left keyline endpoints
        const Eigen::Vector3f p1 = K1*Bp1;
        const Eigen::Vector3f q1 = K1*Bq1;
#else 
        // assume the keylines are already undistorted
        const Eigen::Vector3f p1(kl1.startPointX , kl1.startPointY, 1.0);
        const Eigen::Vector3f q1(kl1.endPointX ,   kl1.endPointY, 1.0);         
#endif 
        Eigen::Vector3f l1 = p1.cross(q1);
        const float l1Norm = sqrt( Utils::Pow2(l1[0]) + Utils::Pow2(l1[1]) );
        l1 = l1/l1Norm; // in this way we have l1 = (nx, ny, -d) with (nx^2 + ny^2) = 1
                           
#if KEYLINES_ARE_DISTORTED
        // undistort the right keyline endpoints
        const Eigen::Vector3f p2 = K2*pCamera2->unprojectEig(cv::Point2f(kl2.startPointX,kl2.startPointY));
        const Eigen::Vector3f q2 = K2*pCamera2->unprojectEig(cv::Point2f(kl2.endPointX,kl2.endPointY));    
#else 
        const Eigen::Vector3f p2(kl2.startPointX, kl2.startPointY, 1.0);
        const Eigen::Vector3f q2(kl2.endPointX, kl2.endPointY, 1.0);        
#endif 
        Eigen::Vector3f l2 = p2.cross(q2);
        const float l2Norm = sqrt( Utils::Pow2(l2[0]) + Utils::Pow2(l2[1]) );
        l2 = l2/l2Norm; // in this way we have l2 = (nx, ny, -d) with (nx^2 + ny^2) = 1             

        // Check if we can triangulate, i.e. check if the normals of the two planes corresponding to lines are not parallel
        bool bCanTriangulateLines = true;
        Eigen::Vector3f n1 = K1.transpose()*l1; n1 /= n1.norm();
        Eigen::Vector3f n2 = K2.transpose()*l2; n2 /= n2.norm();

        Eigen::Vector3f n2_1 = camPairdata.R12*n2; // n2 in camera 1 coordinates
        const float normalsDotProduct= fabs( n1.dot(n2_1) );
        const float sigma = std::max( sigma1, sigma2 );
        const float dotProductThreshold = 1e-5 * sigma; //Frame::kLineNormalsDotProdThreshold * sigma; // this is a percentage over unitary modulus ( we modulate threshold by sigma)
        //const float dotProductThreshold = Frame::kLineNormalsDotProdThreshold * sigma; // this is a percentage over unitary modulus ( we modulate threshold by sigma)
        if( fabs( normalsDotProduct - 1.f ) < dotProductThreshold) 
        {
            bCanTriangulateLines = false; // normals are almost parallel => cannot triangulate lines
            //std::cout << "  - cannot triangulate n1: " << n1.transpose() << ", n2: " << n2.transpose() <<", normalsDotProduct: " << fabs( normalsDotProduct - 1.f ) << std::endl;                     
        }
        
        if(bCanTriangulateLines)
        {
            // compute the intersections of rays backprojected from camera 1 with the 3D plane corresponding to l2  
            // (check PLVS report)                                        
            const float num = -l2.dot(camPairdata.e2);       
            const float denP = (l2.dot(camPairdata.H21*p1)); // distance point-line in pixels 
            const float denQ = (l2.dot(camPairdata.H21*q1)); // distance point-line in pixels
            
            constexpr float denTh = 5;            
            if( ( fabs(denP) < denTh ) || (fabs(denQ) < denTh) ) return false; 
                
            outData.depthS = num/denP; // depthP1
            outData.depthE = num/denQ; // depthQ1

            if( (outData.depthS >= minZ ) && (outData.depthE >= minZ) && (outData.depthS <= maxZ) && (outData.depthE <= maxZ) )
            {
    #if KEYLINES_ARE_DISTORTED
                outData.p3DS = outData.depthS * Bp1; 
                outData.p3DE = outData.depthE * Bq1;
    #else   
                outData.p3DS = Eigen::Vector3f( (p1[0]-cx1)*invfx1*outData.depthS, (p1[1]-cy1)*invfy1*outData.depthS, outData.depthS ); // depthP1 * K1.inv()*p1;   
                outData.p3DE = Eigen::Vector3f( (q1[0]-cx1)*invfx1*outData.depthE, (q1[1]-cy1)*invfy1*outData.depthE, outData.depthE ); // depthQ1 * K1.inv()*q1;              
    #endif 
                const Eigen::Vector3f camRay = outData.p3DS.normalized();
                Eigen::Vector3f lineES = outData.p3DS-outData.p3DE;  
                const double lineLength = lineES.norm();
                if(lineLength >= Frame::skMinLineLength3D)
                {        
                    lineES /= lineLength;
                    const float cosViewAngle = fabs((float)camRay.dot(lineES));
                    if(cosViewAngle<=Frame::kCosViewZAngleMax)
                    {
                        outData.isValid = true;
                    }
                }
            }
        }
        return outData.isValid;
    }


    std::ostream & operator<<(std::ostream &os, const KannalaBrandt8 &kb) {
        os << kb.mvParameters[0] << " " << kb.mvParameters[1] << " " << kb.mvParameters[2] << " " << kb.mvParameters[3] << " "
           << kb.mvParameters[4] << " " << kb.mvParameters[5] << " " << kb.mvParameters[6] << " " << kb.mvParameters[7];
        return os;
    }

    std::istream & operator>>(std::istream &is, KannalaBrandt8 &kb) {
        float nextParam;
        for(size_t i = 0; i < 8; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            kb.mvParameters[i] = nextParam;

        }
        return is;
    }

    void KannalaBrandt8::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float,3,4> &Tcw1,
                                     const Eigen::Matrix<float,3,4> &Tcw2, Eigen::Vector3f &x3D)
    {
        Eigen::Matrix<float,4,4> A;
        A.row(0) = p1.x*Tcw1.row(2)-Tcw1.row(0);
        A.row(1) = p1.y*Tcw1.row(2)-Tcw1.row(1);
        A.row(2) = p2.x*Tcw2.row(2)-Tcw2.row(0);
        A.row(3) = p2.y*Tcw2.row(2)-Tcw2.row(1);

        Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);
        Eigen::Vector4f x3Dh = svd.matrixV().col(3);
        x3D = x3Dh.head(3)/x3Dh(3);
    }

    bool KannalaBrandt8::IsEqual(GeometricCamera* pCam)
    {
        if(pCam->GetType() != GeometricCamera::CAM_FISHEYE)
            return false;

        KannalaBrandt8* pKBCam = (KannalaBrandt8*) pCam;

        if(abs(precision - pKBCam->GetPrecision()) > 1e-6)
            return false;

        if(size() != pKBCam->size())
            return false;

        bool is_same_camera = true;
        for(size_t i=0; i<size(); ++i)
        {
            if(abs(mvParameters[i] - pKBCam->getParameter(i)) > 1e-6)
            {
                is_same_camera = false;
                break;
            }
        }
        return is_same_camera;
    }

}

BOOST_CLASS_EXPORT(PLVS2::KannalaBrandt8);    
