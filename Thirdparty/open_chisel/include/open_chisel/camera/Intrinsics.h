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

#ifndef INTRINSICS_H_
#define INTRINSICS_H_

#include <memory>
#include <open_chisel/geometry/Geometry.h>

namespace chisel
{

    class Intrinsics
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Intrinsics();
            virtual ~Intrinsics();

            //inline float GetFx() const { return matrix(0, 0); }
            inline float GetFx() const { return fx; }
            inline void SetFx(float fx_in) { matrix(0, 0) = fx_in; fx = fx_in; }
            
            inline float GetFy() const { return fy; }
            inline void SetFy(float fy_in) { matrix(1, 1) = fy_in; fy = fy_in; }
            
            inline float GetCx() const { return cx; }
            inline void SetCx(float cx_in) { matrix(0, 2) = cx_in; cx = cx_in; }
            
            inline float GetCy() const { return cy; }
            inline void SetCy(float cy_in) { matrix(1, 2) = cy_in; cy = cy_in; }

            inline const Mat3x3& GetMatrix() const { return matrix; }
            inline Mat3x3& GetMutableMatrix() { return matrix; }
            inline void SetMatrix(const Mat3x3& m) 
            { 
                matrix = m; 
                fx = matrix(0, 0);
                fy = matrix(1, 1);
                cx = matrix(0, 2);
                cy = matrix(1, 2);
            }

        protected:
            Mat3x3 matrix;
            float fx;
            float fy;
            float cx;
            float cy; 
    };
    typedef std::shared_ptr<Intrinsics> IntrinsicsPtr;
    typedef std::shared_ptr<const Intrinsics> IntrinsicsConstPtr;

} // namespace chisel 

#endif // INTRINSICS_H_ 
