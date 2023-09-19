#include "elas.h"
#ifdef USE_CUDA
#include "elas_gpu.h"
#endif

#include "ElasInterface.h"

namespace libelas {

ElasInterface::ElasInterface(Elas::Parameters param)
{
#ifdef USE_CUDA
    pImpl = new ElasGPU(param);
#else
    pImpl = new Elas(param);
#endif
}

ElasInterface::~ElasInterface()
{
    delete pImpl;
}
    
void ElasInterface::process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims)
{
    pImpl->process(I1, I2, D1, D2, dims);
}

}


