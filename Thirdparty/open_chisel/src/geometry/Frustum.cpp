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

#include <open_chisel/geometry/Frustum.h>

#include <assert.h>
#include <algorithm>

namespace chisel
{

    Frustum::Frustum()
    {
        // TODO Auto-generated constructor stub

    }

    Frustum::~Frustum()
    {
        // TODO Auto-generated destructor stub
    }

    bool Frustum::Intersects(const AABB& box) const
    {
        const Plane* planes[] = { &far, &near, &top, &bottom, &left, &right };

        for (const Plane* plane : planes)
        {
            Vec3 axisVert;
            const Vec3& normal = plane->normal;

            // x-axis
            // Which AABB vertex is furthest down (plane normals direction) the x axis.
            if (normal(0) < 0.0f)
                axisVert(0) = box.min(0);
            else
                axisVert(0) = box.max(0);

            // y-axis
            // Which AABB vertex is furthest down (plane normals direction) the y axis.
            if (normal(1) < 0.0f)
                axisVert(1) = box.min(1);
            else
                axisVert(1) = box.max(1);

            // z-axis
            // Which AABB vertex is furthest down (plane normals direction) the z axis.
            if (normal(2) < 0.0f)
                axisVert(2) = box.min(2);
            else
                axisVert(2) = box.max(2);

            // Now we get the signed distance from the AABB vertex that's furthest down
            // the frustum planes normal, and if the signed distance is negative, then
            // the entire bounding box is behind the frustum plane.
            if (axisVert.dot(normal) + plane->distance > 0.0f)
                return true;
        }

        return false;
    }

    bool Frustum::Contains(const Vec3& point) const
    {
        // Compute distance to all the planes of the frustum.
        const Plane* planes[] = { &far, &near, &top, &bottom, &left, &right };

        int i = 0;
        for (const Plane* plane : planes)
        {
            if (plane->ClassifyPoint(point) == Plane::IntersectionType::Outside)
            {
                return false;
            }
            i++;
        }

        // Frustrum is convex intersection of positive parts of 6 planes.
        // If it is inside all the planes, it is inside the frustum.
        return true;
    }

    void Frustum::ComputeBoundingBox(AABB* box) const
    {
        assert(box != nullptr);
        float bigNum = std::numeric_limits<float>::max();

        Vec3 tempMin(bigNum, bigNum, bigNum);
        Vec3 tempMax(-bigNum, -bigNum, -bigNum);
        for (int i = 0; i < 8; i++)
        {
          const Vec3& corner = corners[i];
          tempMin(0) = std::min<float>(tempMin(0), corner(0));
          tempMin(1) = std::min<float>(tempMin(1), corner(1));
          tempMin(2) = std::min<float>(tempMin(2), corner(2));

          tempMax(0) = std::max<float>(tempMax(0), corner(0));
          tempMax(1) = std::max<float>(tempMax(1), corner(1));
          tempMax(2) = std::max<float>(tempMax(2), corner(2));
        }

        box->min = tempMin;
        box->max = tempMax;
    }

    void Frustum::SetFromOpenGLViewProjection(const Mat4x4& view, const Mat4x4& proj)
    {
        Vec3 right = view.transpose().block(0, 0, 3, 1).cast<float>();
        Vec3 up = view.transpose().block(0, 1, 3, 1).cast<float>();
        Vec3 d = -view.transpose().block(0, 2, 3, 1).cast<float>();
        Vec3 p = view.block(0, 3, 3, 1).cast<float>();
        const float& aa = proj.data()[0];
        const float& bb = proj.data()[5];
        const float& cc = proj.data()[10];
        const float& dd = proj.data()[14];

        float aspect = bb / aa;
        float fov = 2.0f * atan(1.0f / bb);
        float kk = (cc - 1.0f) / (cc + 1.0f);
        float n = (dd * (1.0f - kk)) / (2.0f * kk);
        float f = kk * n;
        SetFromVectors(d, p, right, up, n, f, fov, aspect);
    }

    void Frustum::SetFromParams(const Transform& view, float nearDist, float farDist, float fx, float fy, float /*cx*/, float cy, float imgWidth, float imgHeight)
    {
        Mat3x3 r = view.linear();
        Vec3 right = r.col(0);
        Vec3 up = -r.col(1);
        Vec3 d = r.col(2);
        Vec3 p = view.translation();
        float aspect = (fx * imgWidth) / (fy * imgHeight);
        float fov = atan2(cy, fy) + atan2(imgHeight - cy, fy);
        SetFromVectors(d, p, right, up, nearDist, farDist, fov, aspect);
    }

    void Frustum::SetFromVectors(const Vec3& forward, const Vec3& pos, const Vec3& rightVec, const Vec3& up, float nearDist, float farDist, float fov, float aspect)
    {
        const float angleTangent = tan(fov / 2);
        const float heightFar = angleTangent * farDist;
        const float widthFar = heightFar * aspect;
        const float heightNear = angleTangent * nearDist;
        const float widthNear = heightNear * aspect;
        const Vec3 farCenter = pos + forward * farDist;
        const Vec3 farTopLeft = farCenter + (up * heightFar) - (rightVec * widthFar);
        const Vec3 farTopRight = farCenter + (up * heightFar) + (rightVec * widthFar);
        const Vec3 farBotLeft = farCenter - (up * heightFar) - (rightVec * widthFar);
        const Vec3 farBotRight = farCenter - (up * heightFar) + (rightVec * widthFar);

        const Vec3 nearCenter = pos + forward * nearDist;
        const Vec3 nearTopLeft = nearCenter + (up * heightNear) - (rightVec * widthNear);
        const Vec3 nearTopRight = nearCenter + (up * heightNear) + (rightVec * widthNear);
        const Vec3 nearBotLeft = nearCenter - (up * heightNear) - (rightVec * widthNear);
        const Vec3 nearBotRight = nearCenter - (up * heightNear) + (rightVec * widthNear);

        near = Plane(nearBotLeft, nearTopLeft, nearBotRight);
        far = Plane(farTopRight, farTopLeft, farBotRight);
        left = Plane(farTopLeft, nearTopLeft, farBotLeft);
        right = Plane(nearTopRight, farTopRight, nearBotRight);
        top = Plane(nearTopLeft, farTopLeft, nearTopRight);
        bottom = Plane(nearBotRight, farBotLeft, nearBotLeft);

        corners[0] = farTopLeft.cast<float>();
        corners[1] = farTopRight.cast<float>();
        corners[2] = farBotLeft.cast<float>();
        corners[3] = farBotRight.cast<float>();
        corners[4] = nearBotRight.cast<float>();
        corners[5] = nearTopLeft.cast<float>();
        corners[6] = nearTopRight.cast<float>();
        corners[7] = nearBotLeft.cast<float>();

        // Far face lines.
        lines[0] = corners[0];
        lines[1] = corners[1];
        lines[2] = corners[3];
        lines[3] = corners[2];
        lines[4] = corners[1];
        lines[5] = corners[3];
        lines[6] = corners[2];
        lines[7] = corners[0];

        // Near face lines.
        lines[8] = corners[4];
        lines[9] = corners[7];
        lines[10] = corners[6];
        lines[11] = corners[5];
        lines[12] = corners[5];
        lines[13] = corners[7];
        lines[14] = corners[6];
        lines[15] = corners[4];

        // Connecting lines.
        lines[16] = corners[0];
        lines[17] = corners[5];
        lines[18] = corners[1];
        lines[19] = corners[6];
        lines[20] = corners[2];
        lines[21] = corners[7];
        lines[22] = corners[3];
        lines[23] = corners[4];
    }


} // namespace chisel 
