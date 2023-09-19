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

#ifndef PINHOLECAMERA_H_
#define PINHOLECAMERA_H_

#include <memory>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/camera/Intrinsics.h>

namespace chisel
{

    class PinholeCamera
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            PinholeCamera();
            virtual ~PinholeCamera();

            inline const Intrinsics& GetIntrinsics() const { return intrinsics; }
            inline Intrinsics& GetMutableIntrinsics() { return intrinsics; }
            inline void SetIntrinsics(const Intrinsics& value) { intrinsics = value; }

            inline int GetWidth() const { return width; }
            inline int GetHeight() const { return height; }
            inline void SetWidth(int value) { width = value; }
            inline void SetHeight(int value) { height = value; }
            inline float GetNearPlane() const { return nearPlane; }
            inline float GetFarPlane() const { return farPlane; }
            inline void SetNearPlane(float value) { nearPlane = value; }
            inline void SetFarPlane(float value) { farPlane = value; }

            void SetupFrustum(const Transform& view, Frustum* frustum) const;

            Vec3 ProjectPoint(const Vec3& point) const;
            Vec3 UnprojectPoint(const Vec3& point) const;

            bool IsPointOnImage(const Vec3& point) const;

        protected:
            Intrinsics intrinsics;
            int width;
            int height;
            float nearPlane;
            float farPlane;

    };
    typedef std::shared_ptr<PinholeCamera> PinholeCameraPtr;
    typedef std::shared_ptr<const PinholeCamera> PinholeCameraConstPtr;

} // namespace chisel 

#endif // PINHOLECAMERA_H_ 
