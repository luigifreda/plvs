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


#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#ifdef USE_G2O_NEW
#include "Thirdparty/g2o_new/install/include/g2o/core/eigen_types.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sba/types_six_dof_expmap.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sim3/types_seven_dof_expmap.h"
#else
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h" 
#endif

#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

namespace PLVS2
{

class Converter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const Sophus::SE3f &T);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static Eigen::Isometry3d toIsometry3d(const cv::Mat &cvT);
    static Eigen::Isometry3d toIsometry3d(const Sophus::SE3f &T);    

    // TODO templetize these functions
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix<float,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix<float,3,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,6,1> &m);
    static cv::Mat toCvMat(const Eigen::Matrix<float,3,1> &m);
    static cv::Mat toCvMat(const Eigen::Matrix<float,3,3> &m);

    static cv::Mat toCvMat(const Eigen::MatrixXf &m);
    static cv::Mat toCvMat(const Eigen::MatrixXd &m);

    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    static cv::Mat toCvSE3(const Eigen::Matrix<float,3,3> &R, const Eigen::Matrix<float,3,1> &t);

    template <typename T> 
    static cv::Mat toCvSE3d(const Eigen::Matrix<T,3,3> &R, const Eigen::Matrix<T,3,1> &t);

    static cv::Mat toCvSim3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t, const double s); 
    static cv::Mat toCvSim3(const Eigen::Matrix<float,3,3> &R, const Eigen::Matrix<float,3,1> &t, const float s);  
    
    static cv::Mat toCvSim3(const Sophus::Sim3f& Sim3);
    static cv::Mat toCvSE3(const Sophus::Sim3f& Sim3);
    static cv::Mat toCvSE3(const Sophus::SE3f& T);

    static cv::Mat toCvSE3d(const Sophus::SE3f& T);

    static cv::Mat tocvSkewMatrix(const cv::Mat &v);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<float,3,1> toVector3f(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);

    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static Eigen::Matrix<double,4,4> toMatrix4d(const cv::Mat &cvMat4);
    static Eigen::Matrix<float,3,3> toMatrix3f(const cv::Mat &cvMat3);
    static Eigen::Matrix<float,4,4> toMatrix4f(const cv::Mat &cvMat4);

    static Eigen::Matrix<double,6,1> toVector6d(const cv::Mat &cvVector3D1, const cv::Mat &cvVector3D2);
    static Eigen::Matrix<double,6,1> toVector6d(const Eigen::Vector3f &v3D1, const Eigen::Vector3f &v3D2);

    static std::vector<float> toQuaternion(const cv::Mat &M);

    static bool isRotationMatrix(const cv::Mat &R);
    static std::vector<float> toEuler(const cv::Mat &R);

    //TODO: Sophus migration, to be deleted in the future
    static Sophus::SE3<float> toSophus(const cv::Mat& T);
    static Sophus::Sim3f toSophus(const g2o::Sim3& S);
};

}// namespace PLVS2

#endif // CONVERTER_H
