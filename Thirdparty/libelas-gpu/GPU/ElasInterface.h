#ifndef __ELAS_INTERFACE_H__
#define __ELAS_INTERFACE_H__

#include "elas.h"

namespace libelas {

class ElasInterface
{

public:
    
    // Constructor, input: parameters
    // Pass this to the super constructor
    ElasInterface(Elas::Parameters param);
    
    ~ElasInterface();
    
    // matching function
    // inputs: pointers to left (I1) and right (I2) intensity image (uint8, input)
    //         pointers to left (D1) and right (D2) disparity image (float, output)
    //         dims[0] = width of I1 and I2
    //         dims[1] = height of I1 and I2
    //         dims[2] = bytes per line (often equal to width, but allowed to differ)
    //         note: D1 and D2 must be allocated before (bytes per line = width)
    //               if subsampling is not active their size is width x height,
    //               otherwise width/2 x height/2 (rounded towards zero)
    void process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims);
    
    const Elas::Parameters& getParameters() { return pImpl->param; }

private: 

    Elas* pImpl; 
    
};

}

#endif //__ELAS_INTERFACE_H__
