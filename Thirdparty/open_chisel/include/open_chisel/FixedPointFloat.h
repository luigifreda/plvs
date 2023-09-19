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


#ifndef FIXEDPOINTFLOAT_H_
#define FIXEDPOINTFLOAT_H_

#include <algorithm>

namespace chisel
{
    const float MinFloatValue = -1000;
    const float MaxFloatValue = 1000;
    const float MaxUFloat = 1000;
    typedef uint16_t FixedFloat16;
    typedef uint16_t UFixedFloat16;

    inline float ClampFloat(const float& input)
    {
        return std::max(std::min(input, MaxFloatValue), MinFloatValue);
    }

    inline float ClampUnsignedFloat(const float input)
    {
        return  std::max(std::min(input, MaxUFloat), 0.0f);
    }

    inline FixedFloat16 FloatToFixedFloat16(const float& input)
    {
        return static_cast<FixedFloat16>(((ClampFloat(input) - MinFloatValue) / (MaxFloatValue - MinFloatValue)) * std::numeric_limits<FixedFloat16>::max());
    }

    inline float FixedFloat16ToFloat(const FixedFloat16& input)
    {
        const float t = static_cast<float>(input) / std::numeric_limits<FixedFloat16>::max();
        return MinFloatValue + t * (MaxFloatValue - MinFloatValue);
    }

    inline UFixedFloat16 FloatToUFixedFloat16(const float& input)
    {
        return static_cast<UFixedFloat16>((ClampUnsignedFloat(input) / MaxUFloat) * std::numeric_limits<UFixedFloat16>::max());
    }

    inline float UFixedFloat16ToFloat(const UFixedFloat16& input)
    {
        const float t = static_cast<float>(input) / std::numeric_limits<UFixedFloat16>::max();
        return   t * (MaxUFloat);
    }
}



#endif // FIXEDPOINTFLOAT_H_ 
