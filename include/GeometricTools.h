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


#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

#include <opencv2/core/core.hpp>
#include <Thirdparty/Sophus/sophus/se3.hpp>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace PLVS2
{

class KeyFrame;


struct CameraPairTriangulationInput
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3f K1, K2; 
    
    Eigen::Matrix3f R12, R21;
    Eigen::Vector3f t12, t21;

    Eigen::Matrix3f H21;
    Eigen::Vector3f e2;    

    float minZ = -1; 
    float maxZ = -1; 
};

struct LineTriangulationOutput
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3f p3DS; // start point in camera 1 (left) coordinates
    Eigen::Vector3f p3DE; // end point in camera 1 (left) coordinates
    float depthS = -1;    // start point depth in camera 1 (left)
    float depthE = -1;    // end point depth in camera 1 (left)
    bool isValid = false; 
};


class GeometricTools
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Validate a finite 3D segment against an undistorted image detection.
    // Distance to the infinite supporting line alone cannot reject unrelated
    // detections that occupy disjoint intervals along that line.
    static bool CheckLineSegmentProjection(const Eigen::Vector3f& start,
                                          const Eigen::Vector3f& end,
                                          const Eigen::Matrix3f& K,
                                          const Eigen::Vector2f& observedStart,
                                          const Eigen::Vector2f& observedEnd,
                                          float sigma2)
    {
        if(!start.allFinite() || !end.allFinite() || start.z() <= 1e-6f || end.z() <= 1e-6f ||
           !K.allFinite() || !observedStart.allFinite() || !observedEnd.allFinite() ||
           !std::isfinite(sigma2) || sigma2 <= 0.f)
            return false;
        const Eigen::Vector3f hs = K * start;
        const Eigen::Vector3f he = K * end;
        const Eigen::Vector2f projectedStart = hs.head<2>() / hs.z();
        const Eigen::Vector2f projectedEnd = he.head<2>() / he.z();
        const Eigen::Vector2f observedDirection = observedEnd - observedStart;
        const float length = observedDirection.norm();
        if(!projectedStart.allFinite() || !projectedEnd.allFinite() || length <= 1e-6f)
            return false;
        const Eigen::Vector2f axis = observedDirection / length;
        const Eigen::Vector2f normal(-axis.y(), axis.x());
        const float errorStart = normal.dot(projectedStart - observedStart);
        const float errorEnd = normal.dot(projectedEnd - observedStart);
        if(errorStart * errorStart > 3.84f * sigma2 || errorEnd * errorEnd > 3.84f * sigma2)
            return false;
        const float s = axis.dot(projectedStart - observedStart);
        const float e = axis.dot(projectedEnd - observedStart);
        const float projectedLength = std::fabs(e - s);
        const float overlap = std::min(length, std::max(s, e)) - std::max(0.f, std::min(s, e));
        // Allow partial visibility and reversed endpoints, but require actual
        // shared support (at least 10% of the shorter segment).
        return projectedLength > 1e-6f && overlap >= 0.1f * std::min(length, projectedLength);
    }
    
    // Compute the Fundamental matrix between KF1 and KF2
    static Eigen::Matrix3f ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    //Triangulate point with KF1 and KF2
    static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D);

    template<int rows, int cols>
    static bool CheckMatrices(const cv::Mat &cvMat, const Eigen::Matrix<float,rows,cols> &eigMat) {
        const float epsilon = 1e-3;
        // std::cout << cvMat.cols - cols << cvMat.rows - rows << std::endl;
        if(rows != cvMat.rows || cols != cvMat.cols) {
            std::cout << "wrong cvmat size\n";
            return false;
        }
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((cvMat.at<float>(i,j) > (eigMat(i,j) + epsilon)) ||
                    (cvMat.at<float>(i,j) < (eigMat(i,j) - epsilon))){
                    std::cout << "cv mat:\n" << cvMat << std::endl;
                    std::cout << "eig mat:\n" << eigMat << std::endl;
                    return false;
                }
        return true;
    }

    template<typename T, int rows, int cols>
    static bool CheckMatrices( const Eigen::Matrix<T,rows,cols> &eigMat1, const Eigen::Matrix<T,rows,cols> &eigMat2) {
        const float epsilon = 1e-3;
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((eigMat1(i,j) > (eigMat2(i,j) + epsilon)) ||
                    (eigMat1(i,j) < (eigMat2(i,j) - epsilon))){
                    std::cout << "eig mat 1:\n" << eigMat1 << std::endl;
                    std::cout << "eig mat 2:\n" << eigMat2 << std::endl;
                    return false;
                }
        return true;
    }

};

}// namespace PLVS2

#endif // GEOMETRIC_TOOLS_H
