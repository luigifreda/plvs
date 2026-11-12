/*
 * CUDA to HIP compatibility header for libsgm
 * This header provides aliases so CUDA code compiles unchanged on HIP.
 */

#pragma once

#if defined(USE_HIP) || defined(__HIP_PLATFORM_AMD__)

#include <hip/hip_runtime.h>

// Runtime API - streams and events
#define cudaStream_t                    hipStream_t
#define cudaEvent_t                     hipEvent_t
#define cudaStreamCreate                hipStreamCreate
#define cudaStreamDestroy               hipStreamDestroy
#define cudaStreamSynchronize           hipStreamSynchronize
#define cudaStreamWaitEvent             hipStreamWaitEvent
#define cudaEventCreate                 hipEventCreate
#define cudaEventDestroy                hipEventDestroy
#define cudaEventRecord                 hipEventRecord

// Error handling
#define cudaError_t                     hipError_t
#define cudaError                       hipError_t
#define cudaSuccess                     hipSuccess
#define cudaGetLastError                hipGetLastError
#define cudaGetErrorString              hipGetErrorString

// Memory
#define cudaMalloc                      hipMalloc
#define cudaFree                        hipFree
#define cudaMemcpy                      hipMemcpy
#define cudaMemset                      hipMemset
#define cudaMemcpyHostToDevice          hipMemcpyHostToDevice
#define cudaMemcpyDeviceToHost          hipMemcpyDeviceToHost
#define cudaMemcpyDeviceToDevice        hipMemcpyDeviceToDevice

// The .cu sources gate the warp-synchronous shuffle variants
// (__shfl_*_sync, which require an explicit lane mask) behind
// CUDA_VERSION >= 9000 and fall back to the maskless __shfl_* forms
// otherwise. On HIP the masked variants demand a 64-bit mask on wave64
// hardware (a compile-time assert rejects the 32-bit 0xffffffff literals
// these sources pass), and the maskless forms already carry the logical
// width argument the algorithm needs, so select the pre-9000 branch by
// leaving CUDA_VERSION at 0.
#ifndef CUDA_VERSION
#define CUDA_VERSION 0
#endif

// Helper macro for error checking
#ifndef CudaSafeCall
#define CudaSafeCall(err) do { \
    hipError_t _err = (err); \
    if (_err != hipSuccess) { \
        printf("HIP error at %s:%d: %s\n", __FILE__, __LINE__, hipGetErrorString(_err)); \
    } \
} while(0)
#endif

#else

#include <cuda_runtime.h>

#ifndef CudaSafeCall
#define CudaSafeCall(err) do { \
    cudaError_t _err = (err); \
    if (_err != cudaSuccess) { \
        printf("CUDA error at %s:%d: %s\n", __FILE__, __LINE__, cudaGetErrorString(_err)); \
    } \
} while(0)
#endif

#endif
