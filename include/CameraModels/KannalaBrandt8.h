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

#ifndef CAMERAMODELS_KANNALABRANDT8_H
#define CAMERAMODELS_KANNALABRANDT8_H


#include <assert.h>
#include <vector>
#include <opencv2/core/core.hpp>

#include "BoostArchiver.h"
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/nvp.hpp>

#include "GeometricCamera.h"

#include "TwoViewReconstruction.h"
#include "Utils.h"

#include <line_descriptor_custom.hpp>

namespace PLVS2 {
    class KannalaBrandt8 : public GeometricCamera {

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        //ar & boost::serialization::base_object<GeometricCamera>(*this);
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GeometricCamera);
        ar & const_cast<float&>(precision);
        ar & mvLappingArea;
    }

    public:
        KannalaBrandt8(const float linearFovScale=0.7) : GeometricCamera(linearFovScale), precision(1e-6) {
            mvParameters.resize(8);
            mnId=nNextId++;
            mnType = CAM_FISHEYE;
        }
        KannalaBrandt8(const std::vector<float> vParameters, const float linearFovScale=0.7) 
            : GeometricCamera(vParameters, linearFovScale), precision(1e-6), mvLappingArea(2,0) ,tvr(nullptr) {
            MSG_ASSERT(mvParameters.size() == 8,"KannalaBrandt8::KannalaBrandt8: mvParameters.size() != 8");
            mnId=nNextId++;
            mnType = CAM_FISHEYE;
        }
#if 0
        KannalaBrandt8(const std::vector<float> _vParameters, const float _linearFovScale, const float _precision) 
            : GeometricCamera(_vParameters, _linearFovScale), precision(_precision), mvLappingArea(2,0) {
            MSG_ASSERT(mvParameters.size() == 8,"KannalaBrandt8::KannalaBrandt8: mvParameters.size() != 8");
            mnId=nNextId++;
            mnType = CAM_FISHEYE;
        }
#endif         
        KannalaBrandt8(KannalaBrandt8* pKannala) : GeometricCamera(pKannala->mvParameters, pKannala->mfLinearFovScale), precision(pKannala->precision), mvLappingArea(2,0) ,tvr(nullptr) {
            MSG_ASSERT(mvParameters.size() == 8,"KannalaBrandt8::KannalaBrandt8: mvParameters.size() != 8");
            mnId=nNextId++;
            mnType = CAM_FISHEYE;
        }

        // project with distortion model 
        cv::Point2f project(const cv::Point3f &p3D) const;
        Eigen::Vector2d project(const Eigen::Vector3d & v3D) const;
        Eigen::Vector2f project(const Eigen::Vector3f & v3D) const;
        Eigen::Vector2f projectMat(const cv::Point3f& p3D) const;

        cv::Mat getDistortionParams() const { return (cv::Mat_<float>(4,1) << getParameter(4), getParameter(5), getParameter(6), getParameter(7)); }

        float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) const;

        // unproject with distortion model
        Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const;
        cv::Point3f unproject(const cv::Point2f &p2D) const;

        Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D) const;


        bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                     Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

        cv::Mat toK() const;
        Eigen::Matrix3f toK_() const;

        bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc);

        float TriangulateMatches(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,  const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc, Eigen::Vector3f& p3D);
        bool TriangulateLineMatches(GeometricCamera* pCamera2, const cv::line_descriptor_c::KeyLine& kl1, const cv::line_descriptor_c::KeyLine& kl2,  const float sigma1, const float sigma2, const CameraPairTriangulationInput& camPairdata, LineTriangulationOutput& outData);

        std::vector<int> mvLappingArea;

        bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                 Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                                 const float sigmaLevel1, const float sigmaLevel2,
                                 Eigen::Vector3f& x3Dtriangulated);

        friend std::ostream& operator<<(std::ostream& os, const KannalaBrandt8& kb);
        friend std::istream& operator>>(std::istream& is, KannalaBrandt8& kb);

        float GetPrecision(){ return precision;}

        bool IsEqual(GeometricCamera* pCam);
    private:
        const float precision;

        //Parameters vector corresponds to
        //[fx, fy, cx, cy, k0, k1, k2, k3]

        TwoViewReconstruction* tvr;

        void Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float,3,4> &Tcw1,
                         const Eigen::Matrix<float,3,4> &Tcw2, Eigen::Vector3f &x3D);
    };
}

//BOOST_CLASS_EXPORT_KEY(ORBSLAM2::KannalaBrandt8)

#endif //CAMERAMODELS_KANNALABRANDT8_H
