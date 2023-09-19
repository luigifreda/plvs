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

#ifndef QUADRATICTRUNCATOR_H_
#define QUADRATICTRUNCATOR_H_

namespace chisel
{

    class QuadraticTruncator : public Truncator
    {
        public:
            QuadraticTruncator() = default;

            QuadraticTruncator(float quadratic, float linear, float constant, float scale) :
                quadraticTerm(quadratic), linearTerm(linear), constantTerm(constant), scalingFactor(scale)
            {

            }

            virtual ~QuadraticTruncator()
            {

            }


            inline float GetTruncationDistance(float reading) const
            {
                //return std::abs(GetQuadraticTerm() * pow(reading, 2) + GetLinearTerm() * reading + GetConstantTerm()) * scalingFactor;
                //return std::abs(GetQuadraticTerm() * reading*reading + GetLinearTerm() * reading + GetConstantTerm()) * scalingFactor;
                return (GetQuadraticTerm() * reading*reading + GetLinearTerm() * reading + GetConstantTerm()) * scalingFactor; 
            }

            inline float GetQuadraticTerm() const { return quadraticTerm; }
            inline float GetLinearTerm() const { return linearTerm; }
            inline float GetConstantTerm() const { return constantTerm; }
            inline float GetScalingFactor() const { return scalingFactor; }
            
            inline void SetQuadraticTerm(float value) { quadraticTerm = value; }
            inline void SetLinearTerm(float value)  { linearTerm = value; }
            inline void SetConstantTerm(float value)  { constantTerm = value; }
            inline void SetScalingFactor(float value)  { scalingFactor = value; }

        protected:
            float quadraticTerm;
            float linearTerm;
            float constantTerm;
            float scalingFactor;

    };
    typedef std::shared_ptr<QuadraticTruncator> QuadraticTruncatorPtr;
    typedef std::shared_ptr<const QuadraticTruncator> QuadraticTruncatorConstPtr;

} // namespace chisel 

#endif // QUADRATICTRUNCATOR_H_ 
