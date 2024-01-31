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

#include <cassert>
#include <cuda/helper_cuda.h>
#include <cuda/Allocator.hpp>

namespace PLVS2
{
namespace cuda
{

size_t Allocator::getPitch(size_t widthSize)
{
    return 128 + widthSize - widthSize % 128;
}

bool Allocator::allocate(cv::cuda::GpuMat* mat, int rows, int cols, size_t elemSize)
{
    if (rows > 1 && cols > 1)
    {
        mat->step = getPitch(elemSize * cols);
        checkCudaErrors(cudaMallocManaged(&mat->data, mat->step * rows));
    }
    else
    {
        // Single row or single column must be continuous
        checkCudaErrors(cudaMallocManaged(&mat->data, elemSize * cols * rows));
        mat->step = elemSize * cols;
    }

    mat->refcount = (int*) new int();

    return true;
}

void Allocator::free(cv::cuda::GpuMat* mat)
{
    checkCudaErrors(cudaFree(mat->datastart));
    delete mat->refcount;
}

cv::cuda::GpuMat::Allocator * gpu_mat_allocator;

}
}


namespace
{
using namespace PLVS2;

void __attribute__ ((constructor)) init()
{
    // Setup GPU Memory Management
    cuda::gpu_mat_allocator = new cuda::Allocator();
    // cv::cuda::GpuMat::setDefaultAllocator(cuda::gpu_mat_allocator);
}
}
