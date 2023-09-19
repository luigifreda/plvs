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

#ifndef MESH_H_
#define MESH_H_

#include <memory>
#include <vector>
#include <open_chisel/geometry/Geometry.h>

namespace chisel
{
    typedef size_t VertIndex;
    typedef std::vector<VertIndex, Eigen::aligned_allocator<VertIndex> > VertIndexList;
    
    class Mesh
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Mesh();
            virtual ~Mesh();

            inline bool HasVertices() const { return !vertices.empty(); }
            inline bool HasNormals() const { return !normals.empty(); }
            inline bool HasColors() const { return !colors.empty(); }
            inline bool HasIndices() const { return !indices.empty(); }
            inline bool HasKfids() const { return !kfids.empty(); }            

            inline void Clear()
            {
                vertices.clear();
                normals.clear();
                colors.clear();
                indices.clear();
                kfids.clear();
            }
            
            void Transform(const Mat3x3& R, const Vec3& t);

            Vec3List vertices;
            VertIndexList indices;
            Vec3List normals;
            Vec3List colors;
            KfidList kfids;

    };
    typedef std::shared_ptr<Mesh> MeshPtr;
    typedef std::shared_ptr<const Mesh> MeshConstPtr;

} // namespace chisel 

#endif // MESH_H_ 
