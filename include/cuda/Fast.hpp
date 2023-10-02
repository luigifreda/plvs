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
#ifndef __FAST_HPP__
#define __FAST_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <cuda_runtime.h>
#include <cuda/Cuda.hpp>

namespace PLVS
{
namespace cuda
{
using namespace std;
using namespace cv;
using namespace cv::cuda;

const float FEATURE_SIZE = 7.0;

class GpuFast
{
    short2 * kpLoc;
    float * kpScore;
    unsigned int * counter_ptr;
    unsigned int highThreshold;
    unsigned int lowThreshold;
    unsigned int maxKeypoints;
    unsigned int count;
    cv::cuda::GpuMat scoreMat;
    cudaStream_t stream;
    Stream cvStream;
public:
    GpuFast(int highThreshold, int lowThreshold, int maxKeypoints = 10000);
    ~GpuFast();

    void detect(InputArray, std::vector<KeyPoint>&);

    void detectAsync(InputArray);
    void joinDetectAsync(std::vector<KeyPoint>&);
};

class IC_Angle
{
    unsigned int maxKeypoints;
    KeyPoint * keypoints;
    cudaStream_t stream;
    Stream _cvStream;
public:
    IC_Angle(unsigned int maxKeypoints = 10000);
    ~IC_Angle();
    void launch_async(InputArray _image, KeyPoint * _keypoints, int npoints, int half_k, int minBorderX, int minBorderY, int octave, int size);
    void join(KeyPoint * _keypoints, int npoints);

    Stream& cvStream()
    {
        return _cvStream;
    }
    static void loadUMax(const int* u_max, int count);
};
}
}
#endif
