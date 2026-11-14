/*
 * Error-checking helper for the ROCm build of PLVS.
 *
 * The device sources include <cuda/helper_cuda.h> and use checkCudaErrors() to
 * abort on a failed runtime call. The vendored CUDA samples header only defines
 * that macro once the CUDA toolkit has defined __DRIVER_TYPES_H__, so it gives
 * nothing here. On a ROCm build this directory is prepended to the include path
 * and the sources get this file instead; on a CUDA build the directory is not on
 * the include path and the vendored header is used unchanged.
 */

#pragma once

#include <cstdio>
#include <cstdlib>

#include <hip/hip_runtime.h>

namespace PLVS2
{

namespace cuda
{

inline void abortOnHipError(hipError_t status, const char* call, const char* file, int line)
{
    if (status != hipSuccess)
    {
        std::fprintf(stderr, "%s:%d: %s returned %s (%d)\n", file, line, call,
                     hipGetErrorString(status), static_cast<int>(status));
        std::exit(EXIT_FAILURE);
    }
}

} // namespace cuda

} // namespace PLVS2

#define checkCudaErrors(call) \
    ::PLVS2::cuda::abortOnHipError((call), #call, __FILE__, __LINE__)
