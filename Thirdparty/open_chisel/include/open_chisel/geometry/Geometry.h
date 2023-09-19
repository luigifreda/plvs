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


#ifndef CHISEL_GEOMETRY_H_
#define CHISEL_GEOMETRY_H_

#include <vector>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace chisel
{
    typedef Eigen::Vector2i Point2;
    typedef Eigen::Vector3i Point3;
    typedef Eigen::Vector2f Vec2;
    typedef Eigen::Vector3f Vec3;
    typedef Eigen::Vector4f Vec4;
    typedef Eigen::Matrix3f Mat3x3;
    typedef Eigen::Matrix4f Mat4x4;
    typedef Eigen::Affine3f Transform;
    typedef Eigen::Quaternionf Quaternion;

    typedef std::vector<Point2, Eigen::aligned_allocator<Point2> > Point2List;
    typedef std::vector<Point3, Eigen::aligned_allocator<Point3> > Point3List;
    typedef std::vector<Vec2, Eigen::aligned_allocator<Vec2> > Vec2List;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Vec3List;
    typedef std::vector<Vec4, Eigen::aligned_allocator<Vec4> > Vec4List;
    typedef std::vector<Mat3x3, Eigen::aligned_allocator<Mat3x3> > Mat3x3List;
    typedef std::vector<Mat4x4, Eigen::aligned_allocator<Mat4x4> > Mat4List;
    typedef std::vector<Transform, Eigen::aligned_allocator<Transform> > TransformList;
    typedef std::vector<Quaternion, Eigen::aligned_allocator<Quaternion> > QuaternionList;
    
    typedef std::vector<uint32_t, Eigen::aligned_allocator<uint32_t> > KfidList;
    
    struct TransformRt
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                
        Mat3x3 R;
        Vec3 t; 
        
    };
    typedef std::unordered_map<uint32_t, TransformRt, std::hash<uint32_t>, std::equal_to<uint32_t>, Eigen::aligned_allocator<std::pair<const uint32_t, TransformRt> > > MapKfidRt;    
    
}
#endif // GEOMETRY_H_ 
