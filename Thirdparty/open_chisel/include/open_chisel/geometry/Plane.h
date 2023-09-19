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

#ifndef PLANE_H_
#define PLANE_H_

#include <memory>
#include "Geometry.h"

namespace chisel
{

    class Plane
    {
        public:

            enum class IntersectionType
            {
                Inside,
                Outside,
                Intersects
            };
            
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            Plane();
            Plane(const Vec4& params);
            Plane(const Vec3& normal, float distance);
            Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3);
            Plane(float a, float b, float c, float d);
            virtual ~Plane();

            float GetSignedDistance(const Vec3& point) const;

            inline IntersectionType ClassifyPoint(const Vec3& point) const
            {
                float d = GetSignedDistance(point);

                if(d < 0)
                {
                    return IntersectionType::Inside;
                }
                else if(d > 0)
                {
                    return IntersectionType::Outside;
                }
                return IntersectionType::Intersects;
            }

            Vec3 normal;
            float distance;

    };
    typedef std::shared_ptr<Plane> PlanePtr;
    typedef std::shared_ptr<const Plane> PlaneConstPtr;

} // namespace chisel 

#endif // PLANE_H_ 
