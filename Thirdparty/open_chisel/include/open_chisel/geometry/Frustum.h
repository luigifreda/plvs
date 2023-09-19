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

#ifndef FRUSTUM_H_
#define FRUSTUM_H_

#include <memory>
#include "Geometry.h"
#include "AABB.h"

namespace chisel
{

    class Frustum
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Frustum();
            virtual ~Frustum();

            bool Intersects(const AABB& box) const;
            bool Contains(const Vec3& point) const;
            void ComputeBoundingBox(AABB* box) const;
            void SetFromParams(const Transform& view, float near, float far, float fx, float fy, float cx, float cy, float imgWidth, float imgHeight);
            void SetFromVectors(const Vec3& forward, const Vec3& pos, const Vec3& right, const Vec3& up, float near, float far, float fov, float aspect);
            void SetFromOpenGLViewProjection(const Mat4x4& view, const Mat4x4& proj);

            const Plane& GetBottomPlane() const { return bottom; }
            const Plane& GetTopPlane() const { return top; }
            const Plane& GetLeftPlane() const { return left; }
            const Plane& GetRightPlane() const { return right; }
            const Plane& GetNearPlane() const { return near; }
            const Plane& GetFarPlane() const { return far; }

            const Vec3* GetLines() const { return lines; }
            const Vec3* GetCorners()  const { return corners; }

        protected:
            Vec3 corners[8];
            Vec3 lines[24];
            Plane top;
            Plane left;
            Plane right;
            Plane bottom;
            Plane near;
            Plane far;
    };
    typedef std::shared_ptr<Frustum> FrustumPtr;
    typedef std::shared_ptr<const Frustum> FrustumConstPtr;

} // namespace chisel 

#endif // FRUSTUM_H_ 
