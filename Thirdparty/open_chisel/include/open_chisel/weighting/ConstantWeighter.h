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

#ifndef CONSTANTWEIGHTER_H_
#define CONSTANTWEIGHTER_H_

#include "Weighter.h"

namespace chisel
{

    class ConstantWeighter : public Weighter
    {
        public:
            ConstantWeighter() = default;
            ConstantWeighter(float w)
            {
                weight = w;
            }
            virtual ~ConstantWeighter()
            {

            }

            virtual float GetWeight(float surfaceDist, float truncationDist) const
            {
                return weight / (2.0f * truncationDist);
            }

        protected:
            float weight;

    };
    typedef std::shared_ptr<ConstantWeighter> ConstantWeighterPtr;
    typedef std::shared_ptr<const ConstantWeighter> ConstantWeighterConstPtr;

} // namespace chisel 

#endif // CONSTANTWEIGHTER_H_ 
