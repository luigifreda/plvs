#include <cuda/helper_cuda.h>
#include <cuda/Cuda.hpp>

namespace PLVS
{

namespace cuda
{

void deviceSynchronize()
{
    checkCudaErrors(cudaDeviceSynchronize());
}

}

}
