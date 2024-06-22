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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "BoostArchiver.h"
#include <boost/serialization/assume_abstract.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include "Converter.h"
#include "GeometricTools.h"

namespace PLVS2 {
    class GeometricCamera {

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & mnId;
            ar & mnType;
            ar & mvParameters;
            ar & mvLinearParameters;
            ar & mfLinearFovScale;
        }

    public:
        GeometricCamera(const float linearFovScale=1.0) : mfLinearFovScale(linearFovScale) 
        {
            mvLinearParameters.resize(4);
        }
        GeometricCamera(const std::vector<float> &vParameters, const float linearFovScale=1.0) 
          : mvParameters(vParameters), mfLinearFovScale(linearFovScale) 
        {
            setLinearParameters(vParameters, linearFovScale);
        }
        ~GeometricCamera() {}

        virtual cv::Point2f project(const cv::Point3f &p3D) const = 0;
        virtual Eigen::Vector2d project(const Eigen::Vector3d & v3D) const = 0;
        virtual Eigen::Vector2f project(const Eigen::Vector3f & v3D) const = 0;
        virtual Eigen::Vector2f projectMat(const cv::Point3f& p3D) const = 0;

        virtual cv::Mat getDistortionParams() const { return cv::Mat(); }

        // project without distortion model 
        Eigen::Vector2d projectLinear(const Eigen::Vector3d & v3D) const;
        Eigen::Vector2f projectLinear(const Eigen::Vector3f & v3D) const;
        Eigen::Vector3d projectLinear3(const Eigen::Vector3d & v3D) const;        

        virtual float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) const = 0;

        virtual Eigen::Vector3f unprojectEig(const cv::Point2f &p2D) const = 0;
        Eigen::Vector3f unprojectEig(const cv::Point2f &p2D, const float d) const {return unprojectEig(p2D)*d;}        
        virtual cv::Point3f unproject(const cv::Point2f &p2D) const = 0;

        Eigen::Vector3f unprojectEigLinear(const float& u, const float& v) const;
        Eigen::Vector3f unprojectEigLinear(const cv::Point2f &p2D) const {return unprojectEigLinear(p2D.x, p2D.y);}
        Eigen::Vector3f unprojectEigLinear(const Eigen::Vector3d & p) const {return unprojectEigLinear(p[0], p[1]);}  // assuming p normalized with p[2]=1       
        Eigen::Vector3f unprojectEigLinear(const cv::Point2f &p2D, const float d) const {return unprojectEigLinear(p2D.x, p2D.y)*d;}    
        Eigen::Vector3f unprojectEigLinear(const Eigen::Vector3d & p, const float d) const {return unprojectEigLinear(p[0], p[1])*d;}  // assuming p normalized with p[2]=1                     

        virtual Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D) const = 0;

        // projection Jacobian without distortion model
        Eigen::Matrix<double,2,3> projectJacLinear(const Eigen::Vector3d& v3D) const;        

        virtual bool ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                                             Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated) = 0;

        virtual cv::Mat toK() const = 0;
        virtual Eigen::Matrix3f toK_() const = 0;
        cv::Mat toLinearK() const;
        Eigen::Matrix3f toLinearK_() const;        

        virtual bool epipolarConstrain(GeometricCamera* otherCamera, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel, const float unc) = 0;

        float getParameter(const int i) const {return mvParameters[i];}
        void setParameter(const float p, const size_t i){mvParameters[i] = p;}
        void setParameters(const std::vector<float>& vParameters, const float linearFovScale=1.0)
        {
            assert(this->size()>0);
            assert(vParameters.size() == this->size());
            mvParameters = vParameters;
            setLinearParameters(mvParameters, linearFovScale);
        }

        void updateLinearParameters() {setLinearParameters(mvParameters, getLinearFovScale());}
        void updateLinearParameters(const float linearFovScale) {setLinearParameters(mvParameters, linearFovScale);}
        
        float getLinearFovScale() const {return mfLinearFovScale;}
        float getLinearParameter(const int i) const {return mvLinearParameters[i];}
        void setLinearParameter(const float p, const size_t i){mvLinearParameters[i] = p;}
        void setLinearParameters(const std::vector<float>& vParameters, const float linearFovScale=1.0)
        {
            assert(vParameters.size() >= 4); // at least fx,fy,cx,cy
            mvLinearParameters.resize(4);
            mfLinearFovScale = linearFovScale; 
            mvLinearParameters[0] = linearFovScale * vParameters[0]; // fx 
            mvLinearParameters[1] = linearFovScale * vParameters[1]; // fy
            mvLinearParameters[2] = vParameters[2]; // cx
            mvLinearParameters[3] = vParameters[3]; // cy
        }

        size_t size() const {return mvParameters.size();}

        virtual bool matchAndtriangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, GeometricCamera* pOther,
                                 Sophus::SE3f& Tcw1, Sophus::SE3f& Tcw2,
                                 const float sigmaLevel1, const float sigmaLevel2,
                                 Eigen::Vector3f& x3Dtriangulated) = 0;

        unsigned int GetId() const { return mnId; }

        unsigned int GetType() const { return mnType; }

        const static unsigned int CAM_PINHOLE = 0;
        const static unsigned int CAM_FISHEYE = 1;

        static long unsigned int nNextId;

    protected:
        std::vector<float> mvParameters;
        std::vector<float> mvLinearParameters;  // Corresponding undistorted PinHole camera model with modified FOV (see setLinearParameters())
                                                // NOTE: With Pinhole camera model, we set mfLinearFovScale=1.0 and mvLinearParameters is equal to mvParameters     

        unsigned int mnId;

        unsigned int mnType;

        float mfLinearFovScale = 1.0f; // This acts on the linear projection model, which represents a corresponding undistorted PinHole camera model
    };

    inline cv::Mat GeometricCamera::toLinearK() const {
        cv::Mat K = (cv::Mat_<float>(3, 3)
            << mvLinearParameters[0], 0.f, mvLinearParameters[2], 0.f, mvLinearParameters[1], mvLinearParameters[3], 0.f, 0.f, 1.f);
        return K;
    }

    inline Eigen::Matrix3f GeometricCamera::toLinearK_() const {
        Eigen::Matrix3f K;
        K << mvLinearParameters[0], 0.f, mvLinearParameters[2], 0.f, mvLinearParameters[1], mvLinearParameters[3], 0.f, 0.f, 1.f;
        return K;
    }

    inline Eigen::Vector2d GeometricCamera::projectLinear(const Eigen::Vector3d & v3D) const
    {
        Eigen::Vector2d res;
        res[0] = mvLinearParameters[0] * v3D[0] / v3D[2] + mvLinearParameters[2];
        res[1] = mvLinearParameters[1] * v3D[1] / v3D[2] + mvLinearParameters[3];
        return res;
    }

    inline Eigen::Vector2f GeometricCamera::projectLinear(const Eigen::Vector3f & v3D) const
    {
        Eigen::Vector2f res;
        res[0] = mvLinearParameters[0] * v3D[0] / v3D[2] + mvLinearParameters[2];
        res[1] = mvLinearParameters[1] * v3D[1] / v3D[2] + mvLinearParameters[3];
        return res;
    }

    inline Eigen::Vector3d GeometricCamera::projectLinear3(const Eigen::Vector3d & v3D) const
    {
        Eigen::Vector3d res;
        res[0] = mvLinearParameters[0] * v3D[0] / v3D[2] + mvLinearParameters[2];
        res[1] = mvLinearParameters[1] * v3D[1] / v3D[2] + mvLinearParameters[3];
        res[2] = 1;
        return res;
    }    

    inline Eigen::Vector3f GeometricCamera::unprojectEigLinear(const float& u, const float& v) const
    {
        Eigen::Vector3f res;
        res[0] = (u - mvLinearParameters[2]) / mvLinearParameters[0];
        res[1] = (v - mvLinearParameters[3]) / mvLinearParameters[1];
        res[2] = 1;
        return res;
    }

    inline Eigen::Matrix<double, 2, 3> GeometricCamera::projectJacLinear(const Eigen::Vector3d &v3D) const {
        Eigen::Matrix<double, 2, 3> Jac;
        const double v3Dz2 = (v3D[2] * v3D[2]);
        Jac(0, 0) = mvLinearParameters[0] / v3D[2];
        Jac(0, 1) = 0.f;
        Jac(0, 2) = -mvLinearParameters[0] * v3D[0] / v3Dz2;
        Jac(1, 0) = 0.f;
        Jac(1, 1) = mvLinearParameters[1] / v3D[2];
        Jac(1, 2) = -mvLinearParameters[1] * v3D[1] / v3Dz2;
        return Jac;
    }    
}


#endif //CAMERAMODELS_GEOMETRICCAMERA_H
