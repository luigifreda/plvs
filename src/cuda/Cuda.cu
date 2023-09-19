#include <cuda/helper_cuda.h>
#include <cuda/Cuda.hpp>

namespace PLVS2
{

namespace cuda
{

void deviceSynchronize()
{
    checkCudaErrors(cudaDeviceSynchronize());
}

}

}
