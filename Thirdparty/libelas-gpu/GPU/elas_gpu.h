#ifndef __ELAS_GPU_H__
#define __ELAS_GPU_H__

// Enable profiling
//#define PROFILE

#include <algorithm>
#include <math.h>
#include <vector>
#include <cuda.h>
#include <stdint.h>
#include <functional>  

#include "elas.h"
#include "descriptor.h"
#include "triangle.h"
#include "matrix.h"

namespace libelas {

/**
 * Our ElasGPU class with all cuda implementations
 * Note where we extend the Elas class so we are calling
 * On all non-gpu functions there if they are not implemented
 */
class ElasGPU : public Elas {

public:     
    
    static bool skCudaInit;
    
public:

  // Constructor, input: parameters
  // Pass this to the super constructor
  ElasGPU(Parameters param);

// This was originally "private"
// Was converted to allow sub-classes to call this
// This assumes the user knows what they are doing
public:

  void computeDisparity(std::vector<support_pt> p_support,std::vector<triangle> tri,int32_t* disparity_grid,int32_t *grid_dims,
                        uint8_t* I1_desc,uint8_t* I2_desc,bool right_image,float* D);

  void adaptiveMean (float* D);

};

}

#endif //__ELAS_GPU_H__
