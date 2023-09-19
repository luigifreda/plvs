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

#include <open_chisel/geometry/AABB.h>

namespace chisel
{

    AABB::AABB()
    {

    }

    AABB::AABB(const Vec3& _min, const Vec3& _max) :
            min(_min), max(_max)
    {

    }

    Plane::IntersectionType AABB::Intersects(const Plane& plane) const
    {
        //check all corner side of plane
        Vec3 ext = GetExtents();

        Vec3List corners;
        corners.resize(8);
        corners[0] = max;
        corners[1] = min;
        corners[2] = min + Vec3(ext(0), 0, 0);
        corners[3] = min + Vec3(0, ext(1), 0);
        corners[4] = min + Vec3(0, 0, ext(2));
        corners[5] = min + Vec3(ext(0), 0, ext(2));
        corners[6] = min + Vec3(ext(0), ext(1), 0);
        corners[7] = min + Vec3(0, ext(1), ext(2));

        float lastdistance = plane.normal.dot(corners[0]) + plane.distance;

        for (int i = 1; i < 8; i++)
        {
            float distance = plane.normal.dot(corners[i]) + plane.distance;

            if ((distance <= 0.0f && lastdistance > 0.0f) || (distance >= 0.0f && lastdistance < 0.0f))
                return Plane::IntersectionType::Intersects;

            lastdistance = distance;
        }

        if (lastdistance > 0.0f)
            return Plane::IntersectionType::Outside;

        return Plane::IntersectionType::Inside;

    }

    AABB::~AABB()
    {

    }

} // namespace chisel 
