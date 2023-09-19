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

#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <memory>
#include <vector>
#include <open_chisel/geometry/Geometry.h>

namespace chisel
{

    class PointCloud
    {
        public:
            PointCloud();
            virtual ~PointCloud();

            inline bool HasColor() const
            {
                return !colors.empty();
            }
            
            inline bool HasNormals() const
            {
                return !normals.empty();
            }            

            inline const Vec3List& GetPoints() const { return points; }
            inline Vec3List& GetMutablePoints() { return points; }
            
            inline const Vec3List& GetColors() const { return colors; }
            inline Vec3List& GetMutableColors() { return colors; }

            inline const Vec3List& GetNormals() const { return normals; }
            inline Vec3List& GetMutableNormals() { return normals; }
            
            inline const KfidList& GetKfids() const { return kfids; }
            inline KfidList& GetMutableKfids() { return kfids; }            
            
            inline void AddPoint(const Vec3& point)
            {
                points.push_back(point);
            }

            inline void AddColor(const Vec3& color)
            {
                colors.push_back(color);
            }
            
            inline void AddNormal(const Vec3& normal)
            {
                normals.push_back(normal);
            }                   

            inline void AddPointAndColor(const Vec3& point, const Vec3& color)
            {
                AddPoint(point);
                AddColor(color);
            }
            
            inline void AddPointColorNormal(const Vec3& point, const Vec3& color, const Vec3& normal)
            {
                AddPoint(point);
                AddColor(color);
                AddNormal(normal);                
            }            
            
            inline void Clear()
            {
                points.clear();
                colors.clear();
                normals.clear();
                kfids.clear();
            }

        protected:
            Vec3List points;
            Vec3List colors;
            Vec3List normals;
            KfidList kfids;
            
    };
    typedef std::shared_ptr<PointCloud> PointCloudPtr;
    typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;

} // namespace chisel 

#endif // POINTCLOUD_H_ 
