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
#ifndef __ALLOCATOR_HPP__
#define __ALLOCATOR_HPP__

#include <opencv2/core/cuda.hpp>

namespace PLVS
{
namespace cuda
{
extern cv::cuda::GpuMat::Allocator * gpu_mat_allocator;

class Allocator : public cv::cuda::GpuMat::Allocator
{
    const int allocatorPitchBase = 128;
    size_t getPitch(size_t widthSize);

public:

    bool allocate(cv::cuda::GpuMat* mat, int rows, int cols, size_t elemSize);
    void free(cv::cuda::GpuMat* mat);
};
}
}

#endif
