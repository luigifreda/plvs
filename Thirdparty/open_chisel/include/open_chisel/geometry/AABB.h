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

#ifndef AABB_H_
#define AABB_H_

#include <memory>
#include <Eigen/Core>

#include "Geometry.h"
#include "Plane.h"

namespace chisel
{

    class AABB
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            AABB();
            AABB(const Vec3& min, const Vec3& max);
            virtual ~AABB();

            inline bool Contains(const Vec3& pos) const
            {
                return pos(0) >= min(0) && pos(1) >= min(1) && pos(2) >= min(2) &&
                       pos(0) <= max(0) && pos(1) <= max(1) && pos(2) <= max(2);
            }

            inline bool Intersects(const AABB& other) const
            {
                if(min.x() > other.max.x()) return false;
                if(min.y() > other.max.y()) return false;
                if(min.z() > other.max.z()) return false;
                if(max.x() < other.min.x()) return false;
                if(max.y() < other.min.y()) return false;
                if(max.z() < other.min.z()) return false;
                return true;
            }

            inline Vec3 GetCenter() const
            {
                return (max + min) * 0.5f;
            }

            inline Vec3 GetExtents() const
            {
                return (max - min) * 0.5f;
            }

            Plane::IntersectionType Intersects(const Plane& other) const;

            Vec3 min;
            Vec3 max;
    };
    typedef std::shared_ptr<AABB> AABBPtr;
    typedef std::shared_ptr<const AABB> AABBConstPtr;

} // namespace chisel 

#endif // AABB_H_ 
