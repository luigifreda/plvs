#pragma once
#ifndef __ALLOCATOR_HPP__
#define __ALLOCATOR_HPP__

#include <opencv2/core/cuda.hpp>

namespace PLVS2
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
