// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef DISTVOXEL_H_
#define DISTVOXEL_H_

#include <limits>
#include <stdint.h>
#include <Eigen/Core>

#include <open_chisel/FixedPointFloat.h>

#define USE_KFID_INTEGRATION 0


namespace chisel
{

    class EIGEN_ALIGN16 DistVoxel
    {
        public:
        	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        	
#if USE_KFID_INTEGRATION 
        static const int kMaxKfidConfidence = 5; 
#endif        
        	
            DistVoxel();
            virtual ~DistVoxel();

            inline float GetSDF() const
            {
                return sdf;
            }
            
            inline uint32_t GetKfid() const
            {
                return kfid;
            }            

            inline void SetSDF(const float& distance)
            {        
                sdf = distance;    
            }
            
            inline void SetKfid( const uint32_t& id)
            {
#if USE_KFID_INTEGRATION         
                if( kfid < 0 )
                {
                    kfid = id; 
                }
                else
                {
                    if( kfid == id) 
                    {
                        kfidConfidence = std::max( int(kfidConfidence) + 1, kMaxKfidConfidence); // clamping 
                    }
                    else
                    {
                        kfidConfidence = std::min( int(kfidConfidence) - 1, 0);
                        if(kfidConfidence == 0) kfid = id;
                    }
                }
#else
                kfid = id;                
#endif                  
            }

            inline float GetWeight() const { return weight; }
            inline void SetWeight(const float& w) { weight = w; }

            inline void Integrate(const float& distUpdate, const float& weightUpdate)
            {
                const float oldSDF = GetSDF();
                const float oldWeight = GetWeight();
                float newDist = (oldWeight * oldSDF + weightUpdate * distUpdate) / (weightUpdate + oldWeight);
                SetSDF(newDist);
                SetWeight(oldWeight + weightUpdate);

            }

            inline void Carve()
            {
                //Reset();
                Integrate(0.0, 1.5);      
            }

            inline void Reset()
            {
                sdf = 99999;
                weight = 0;
#if USE_KFID_INTEGRATION
                kfid = -1;                
                kfidConfidence = 0;
#else
                kfid = 0;
#endif                
            }

        protected:
           float sdf;
           float weight;  // N.B.: is 0 for unknown voxels!
           
#if USE_KFID_INTEGRATION
           int32_t kfid;           
           int kfidConfidence; 
#else
           uint32_t kfid;
#endif           
                   
    };

} // namespace chisel 

#endif // DISTVOXEL_H_ 
