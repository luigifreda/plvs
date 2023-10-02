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

#ifndef SIM3_UTILS_H
#define SIM3_UTILS_H

#include <math.h>
#include <iostream>
#include <limits>
#include <Eigen/Core>

namespace PLVS2
{


class Sim3Utils
{
public:
    
    // outliers are assumed to be zero vectors
    static cv::Vec3d CalculateMean(const cv::Mat_<cv::Vec3d> &points, int numInliers)
    {
        cv::Mat_<cv::Vec3d> result;
        cv::reduce(points, result, 0, cv::REDUCE_SUM);
        return result(0, 0)/numInliers;
    }

    /* Find the transformation  {scale,R,t} that minimize
     * 
     *   || points2 - (scale*R*points1 + t) ||
     * 
     * See :
       "Least-Squares Estimation of Transformation Parameters Between
       Two Point Patterns."  Shinji Umeyama.  IEEE Transactions on
       Pattern Analysis and Machine Intelligence.  Vol. 13, No. 4,
       April 1991.
     */
    static double FindSimTransform(const cv::Mat_<cv::Vec3d> &points1, const cv::Mat_<cv::Vec3d>& points2, cv::Mat_<double>& R,  cv::Mat_<double>& t, double& scale, std::vector<bool>& mask )
    {
        const int N = points1.rows;
        
        int numInliers = 0;        
        for(size_t ii=0;ii<N;ii++)
        {
            if(!mask[ii])
            {
                // reset outliers (so that we can used CalculateMean without reallocating the vectors)
                points1.row(ii) = cv::Vec3d(0,0,0);
                points2.row(ii) = cv::Vec3d(0,0,0);                
            }
            else
            {
                numInliers++;
            }
        }       

        /* Calculate centroids. */
        const cv::Vec3d m1 = CalculateMean(points1, numInliers); 
        const cv::Vec3d m2 = CalculateMean(points2, numInliers); 

        /* Calculate covariance matrix for input points. Also calculate RMS deviation from centroid
         * which is used for scale calculation.
         */
        cv::Mat_<double> C(3, 3, 0.0);
        double p1Rms = 0, p2Rms = 0;
        for(int ptIdx = 0; ptIdx < N; ptIdx++) 
        {
            if(!mask[ptIdx]) continue; 
            
            const cv::Vec3d p1 = points1(ptIdx, 0) - m1; // x1 -m1
            const cv::Vec3d p2 = points2(ptIdx, 0) - m2; // x2 -m2
            p1Rms += p1.dot(p1); // variance 1
            p2Rms += p2.dot(p2); // variance 2
            for (int i = 0; i < 3; i++) 
            {
                for (int j = 0; j < 3; j++)
                {
                    C(i, j) += p2[i] * p1[j];
                }
            }
        }
        
        cv::Mat_<double> U, D, Vt;
        cv::SVD::compute(C, D, U, Vt);

        //std::cout << "D : " << D << std::endl; 
        
        R = U * Vt; // this is equivalent to R = U*S*Vt where S = diag(1,1,1)

        if (cv::determinant(R) < 0) 
        {
            R -= U.col(2) * (Vt.row(2) * 2.0); // this is equivalent to R = U*S*Vt where S = diag(1,1,-1)
            D.at<double>(2) *= -1;
        }

        double traceDS = D.at<double>(0) + D.at<double>(1) + D.at<double>(2);
        scale = traceDS/p1Rms; // this is the mathematical solution presented in the paper 
        //scale = sqrt(p2Rms/p1Rms);   // this is another solution to the scale estimation (ratio amongst standard deviations)
        
        t = (cv::Mat_<double>(3,1) << m2(0), m2(1), m2(2) ) - scale*(R * (cv::Mat_<double>(3,1) << m1(0), m1(1), m1(2) ) );
                
        // compute error 
        double error = 0; 
        std::vector<std::pair<float,int> >  vErrIdx(N);

        for (int ptIdx = 0; ptIdx < N; ptIdx++) 
        {
            if(mask[ptIdx])
            {            
                const cv::Vec3d& p1 = points1(ptIdx, 0); 
                const cv::Vec3d& p2 = points2(ptIdx, 0);   
                const double erri = cv::norm( ( cv::Mat_<double>(3,1) << p2(0), p2(1), p2(2) ) - (R*(scale*( cv::Mat_<double>(3,1) << p1(0), p1(1), p1(2) ))+t) );//, cv::NORM_L2SQR);
                error += erri;
                vErrIdx[ptIdx] = std::pair<float,int>(erri,ptIdx);
            }
            else
            {
                vErrIdx[ptIdx] = std::pair<float,int>(std::numeric_limits<float>::max(),ptIdx);
            }
        }
        error /= numInliers;
        
        // mark outliers 
        //int numOutliers = 0; 
        std::sort(vErrIdx.begin(),vErrIdx.end());
        double median = vErrIdx[vErrIdx.size()/2].first;
        const float thDist = 2.f*1.48f*median;    
        for(int i=vErrIdx.size()-1;i>=0;i--)
        {
            if(vErrIdx[i].first<thDist)
                break;
            else
            {
                //numOutliers++;
                const int idx = vErrIdx[i].second;
                mask[idx] = false;     
            }
        }        
        //std::cout << "sim3 num outliers: " << numOutliers << std::endl; 
        
        return error; 
        
    }  

};

}// namespace PLVS2

#endif 
