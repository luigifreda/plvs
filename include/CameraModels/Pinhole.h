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

#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

#include <assert.h>
#include <vector>
#include <opencv2/core/core.hpp>

#include "BoostArchiver.h"
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/nvp.hpp>

#include "GeometricCamera.h"

#include "TwoViewReconstruction.h"

namespace PLVS2 {
    class Pinhole : public GeometricCamera {

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        //ar & boost::serialization::base_object<GeometricCamera>(*this);
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GeometricCamera);
    }

    public:
        Pinhole(const float linearFovScale=1.0) : GeometricCamera(linearFovScale) {
            mvParameters.resize(4);
            mnId=nNextId++;
            mnType = CAM_PINHOLE;
        }
        Pinhole(const std::vector<float> _vParameters, const float linearFovScale=1.0) : GeometricCamera(_vParameters, linearFovScale), tvr(nullptr) {
            assert(mvParameters.size() == 4);
            mnId=nNextId++;
            mnType = CAM_PINHOLE;
        }

        Pinhole(Pinhole* pPinhole) : GeometricCamera(pPinhole->mvParameters, pPinhole->mfLinearFovScale), tvr(nullptr) {
            assert(mvParameters.size() == 4);
            mnId=nNextId++;
            mnType = CAM_PINHOLE;
        }

        ~Pinhole(){
            if(tvr) delete tvr;
        }

        // project without distortion model 
        cv::Point2f project(const cv::Point3f &p3D) const;
        Eigen::Vector2d project(const Eigen::Vector3d & v3D) const;
        Eigen::Vector2f project(const Eigen::Vector3f & v3D) const;
        Eigen::Vector2f projectMat(const cv::Point3f& p3D) const;

        float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) const;

        // unproject without distortion model
        Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const;
        cv::Point3f unproject(const cv::Point2f &p2D) const;

        Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D) const;


        bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                             Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

        cv::Mat toK() const;
        Eigen::Matrix3f toK_() const;

        bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc);

        bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                 Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                                 const float sigmaLevel1, const float sigmaLevel2,
                                 Eigen::Vector3f& x3Dtriangulated) { return false;}

        friend std::ostream& operator<<(std::ostream& os, const Pinhole& ph);
        friend std::istream& operator>>(std::istream& os, Pinhole& ph);

        bool IsEqual(GeometricCamera* pCam);
    private:
        //Parameters vector corresponds to
        //      [fx, fy, cx, cy]
        TwoViewReconstruction* tvr;
    };
}

//BOOST_CLASS_EXPORT_KEY(ORBSLAM2::Pinhole)

#endif //CAMERAMODELS_PINHOLE_H
