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
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "opencv2/core/cuda/common.hpp"
#include "opencv2/core/cuda/utility.hpp"
#include "opencv2/core/cuda/reduce.hpp"
#include "opencv2/core/cuda/functional.hpp"
#include <cuda/helper_cuda.h>
#include <cuda/Orb.hpp>
#include <cuda/Utils.hpp>

using namespace cv;
using namespace cv::cuda;
using namespace cv::cuda::device;

namespace PLVS2
{
namespace cuda
{

__constant__ unsigned char c_pattern[sizeof (Point) * 512];

void GpuOrb::loadPattern(const Point * _pattern)
{
    checkCudaErrors(cudaMemcpyToSymbol(c_pattern, _pattern, sizeof (Point) * 512));
}

#define GET_VALUE(idx) \
    image(loc.y + __float2int_rn(pattern[idx].x * b + pattern[idx].y * a), \
          loc.x + __float2int_rn(pattern[idx].x * a - pattern[idx].y * b))

__global__ void calcOrb_kernel(const PtrStepb image, KeyPoint * keypoints, const int npoints, PtrStepb descriptors)
{
    int id = blockIdx.x;
    int tid = threadIdx.x;
    if (id >= npoints) return;

    const KeyPoint &kpt = keypoints[id];
    short2 loc = make_short2(kpt.pt.x, kpt.pt.y);
    const Point * pattern = ((Point *) c_pattern) + 16 * tid;

    uchar * desc = descriptors.ptr(id);
    const float factorPI = (float) (CV_PI / 180.f);
    float angle = (float) kpt.angle * factorPI;
    float a = (float) cosf(angle), b = (float) sinf(angle);

    int t0, t1, val;
    t0 = GET_VALUE(0);
    t1 = GET_VALUE(1);
    val = t0 < t1;
    t0 = GET_VALUE(2);
    t1 = GET_VALUE(3);
    val |= (t0 < t1) << 1;
    t0 = GET_VALUE(4);
    t1 = GET_VALUE(5);
    val |= (t0 < t1) << 2;
    t0 = GET_VALUE(6);
    t1 = GET_VALUE(7);
    val |= (t0 < t1) << 3;
    t0 = GET_VALUE(8);
    t1 = GET_VALUE(9);
    val |= (t0 < t1) << 4;
    t0 = GET_VALUE(10);
    t1 = GET_VALUE(11);
    val |= (t0 < t1) << 5;
    t0 = GET_VALUE(12);
    t1 = GET_VALUE(13);
    val |= (t0 < t1) << 6;
    t0 = GET_VALUE(14);
    t1 = GET_VALUE(15);
    val |= (t0 < t1) << 7;

    desc[tid] = (uchar) val;
}

#undef GET_VALUE

GpuOrb::GpuOrb(int maxKeypoints) : maxKeypoints(maxKeypoints), descriptors(maxKeypoints, 32, CV_8UC1)
{
    checkCudaErrors(cudaStreamCreate(&stream));
    cvStream = StreamAccessor::wrapStream(stream);
    checkCudaErrors(cudaMalloc(&keypoints, sizeof (KeyPoint) * maxKeypoints));
}

GpuOrb::~GpuOrb()
{
    cvStream.~Stream();
    checkCudaErrors(cudaFree(keypoints));
    checkCudaErrors(cudaStreamDestroy(stream));
}

void GpuOrb::launch_async(InputArray _image, const KeyPoint * _keypoints, const int npoints)
{
    if (npoints == 0)
    {
        POP_RANGE;
        return;
    }
    const GpuMat image = _image.getGpuMat();

    checkCudaErrors(cudaMemcpyAsync(keypoints, _keypoints, sizeof (KeyPoint) * npoints, cudaMemcpyHostToDevice, stream));
    desc = descriptors.rowRange(0, npoints);
    desc.setTo(Scalar::all(0), cvStream);

    dim3 dimBlock(32);
    dim3 dimGrid(npoints);
    calcOrb_kernel << <dimGrid, dimBlock, 0, stream>>>(image, keypoints, npoints, desc);
    checkCudaErrors(cudaGetLastError());
}

void GpuOrb::join(Mat & _descriptors)
{
    desc.download(_descriptors, cvStream);
    checkCudaErrors(cudaStreamSynchronize(stream));
}
}
}
