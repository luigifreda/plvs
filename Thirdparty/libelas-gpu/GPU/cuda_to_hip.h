/*
 * CUDA to HIP compatibility header for libelas-gpu
 * This header provides aliases so CUDA code compiles unchanged on HIP.
 */

#pragma once

#if defined(USE_HIP) || defined(__HIP_PLATFORM_AMD__)

#include <hip/hip_runtime.h>

// Runtime API
#define cudaMalloc                      hipMalloc
#define cudaFree                        hipFree
#define cudaMemcpy                      hipMemcpy
#define cudaMemcpyToSymbol              hipMemcpyToSymbol
#define cudaMemcpyHostToDevice          hipMemcpyHostToDevice
#define cudaMemcpyDeviceToHost          hipMemcpyDeviceToHost

// Error types
#define cudaError_t                     hipError_t
#define cudaSuccess                     hipSuccess
#define cudaGetLastError                hipGetLastError
#define cudaGetErrorString              hipGetErrorString

// Device sync
#define cudaDeviceSynchronize           hipDeviceSynchronize

#else

#include <cuda_runtime.h>

#endif
