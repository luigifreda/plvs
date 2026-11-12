/*
 * CUDA to HIP compatibility header for PLVS
 * This header provides aliases so CUDA code compiles unchanged on HIP.
 * On CUDA builds, it includes cuda_runtime.h and is otherwise a no-op.
 */

#pragma once

#if defined(USE_HIP) || defined(__HIP_PLATFORM_AMD__)

// The PLVS device sources include OpenCV's CUDA module headers
// (opencv2/core/cuda/{common,utility,reduce,functional}.hpp). On a ROCm build
// of OpenCV those headers keep the CUDA runtime spellings and rely on OpenCV's
// own installed shim to alias them to HIP. Pull that shim in so the texture,
// channel-format, stream and runtime symbols OpenCV uses resolve identically
// here as they do inside the OpenCV build.
#include <opencv2/core/cuda/cuda_to_hip.h>

// Managed-memory aliases that the PLVS allocator uses but OpenCV does not, so
// they are absent from OpenCV's shim.
#define cudaMallocManaged               hipMallocManaged
#define cudaStreamAttachMemAsync        hipStreamAttachMemAsync
#define cudaMemAttachGlobal             hipMemAttachGlobal

#else

#include <cuda_runtime.h>

#endif
