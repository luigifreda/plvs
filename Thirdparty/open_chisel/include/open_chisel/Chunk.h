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

#ifndef CHUNK_H_
#define CHUNK_H_

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <open_chisel/geometry/AABB.h>
#include "DistVoxel.h"
#include "ColorVoxel.h"

namespace chisel
{
    typedef std::vector<DistVoxel, Eigen::aligned_allocator<DistVoxel> > DistVoxelList;
    typedef std::vector<ColorVoxel, Eigen::aligned_allocator<ColorVoxel> > ColorVoxelList;
    
    typedef Eigen::Vector3i ChunkID;
    typedef int VoxelID;

    struct ChunkStatistics
    {
            size_t numKnownInside;
            size_t numKnownOutside;
            size_t numUnknown;
            float totalWeight;
    };

    class Chunk
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Chunk();
            Chunk(const ChunkID id, const Eigen::Vector3i& numVoxels, float resolution, bool useColor);
            virtual ~Chunk();

            void AllocateDistVoxels();
            void AllocateColorVoxels();

            inline const ChunkID& GetID() const { return ID; }
            inline ChunkID& GetIDMutable() { return ID; }
            inline void SetID(const ChunkID& id) { ID = id; }

            inline bool HasColors() const { return !colors.empty(); }
            inline bool HasVoxels() const { return !voxels.empty(); }
            inline const DistVoxelList& GetVoxels() const { return voxels; }
            inline size_t GetVoxelsSize() const { return voxels.size(); }

            inline const Eigen::Vector3i& GetNumVoxels() const { return numVoxels; }
            inline float GetVoxelResolutionMeters() const { return voxelResolutionMeters; }

            //inline const DistVoxel& GetDistVoxel(const VoxelID& voxelID) const { return voxels.at(voxelID); }
            //inline DistVoxel& GetDistVoxelMutable(const VoxelID& voxelID) { return voxels.at(voxelID); }
            inline const DistVoxel& GetDistVoxel(const VoxelID& voxelID) const { return voxels[voxelID]; }
            inline DistVoxel& GetDistVoxelMutable(const VoxelID& voxelID) { return voxels[voxelID]; }
            
            //inline const ColorVoxel& GetColorVoxel(const VoxelID& voxelID) const { return colors.at(voxelID); }
            //inline ColorVoxel& GetColorVoxelMutable(const VoxelID& voxelID) { return colors.at(voxelID); }
            inline const ColorVoxel& GetColorVoxel(const VoxelID& voxelID) const { return colors[voxelID]; }
            inline ColorVoxel& GetColorVoxelMutable(const VoxelID& voxelID) { return colors[voxelID]; }

            Point3 GetVoxelCoords(const Vec3& worldCoords) const;

            inline VoxelID GetVoxelID(const Point3& coords) const
            {
                return GetVoxelID(coords.x(), coords.y(), coords.z());
            }

            inline VoxelID GetVoxelID(const int& x, const int& y, const int& z) const
            {
                return (z * numVoxels(1) + y) * numVoxels(0) + x;
            }

            inline const DistVoxel& GetDistVoxel(const int& x, const int& y, const int& z) const
            {
                return GetDistVoxel(GetVoxelID(x, y, z));
            }

            inline DistVoxel& GetDistVoxelMutable(const int& x, const int& y, const int& z)
            {
                return GetDistVoxelMutable(GetVoxelID(x, y, z));
            }

            inline const ColorVoxel& GetColorVoxel(const int& x, const int& y, const int& z) const
            {
                return GetColorVoxel(GetVoxelID(x, y, z));
            }

            inline ColorVoxel& GetColorVoxelMutable(const int& x, const int& y, const int& z)
            {
                return GetColorVoxelMutable(GetVoxelID(x, y, z));
            }

            inline bool IsCoordValid(VoxelID idx) const
            {
                return idx >= 0 && idx < voxels.size();
            }

            inline bool IsCoordValid(const int& x, const int& y, const int& z) const
            {
                return (x >= 0 && x < numVoxels(0) && y >= 0 && y < numVoxels(1) && z >= 0 && z < numVoxels(2));
            }


            inline size_t GetTotalNumVoxels() const
            {
                return numVoxels(0) * numVoxels(1) * numVoxels(2);
            }

            inline const ColorVoxelList& GetColorVoxels() const
            {
                    return colors;
            }

            void ComputeStatistics(ChunkStatistics* stats);

            AABB ComputeBoundingBox();

            inline const Vec3& GetOrigin() { return origin; }

            Vec3 GetColorAt(const Vec3& pos);

            VoxelID GetVoxelID(const Vec3& worldPos) const;
            VoxelID GetLocalVoxelIDFromGlobal(const Point3& worldPoint) const;
            Point3 GetLocalCoordsFromGlobal(const Point3& worldPoint) const;

        protected:
            ChunkID ID;
            Eigen::Vector3i numVoxels;
            float voxelResolutionMeters;
            float invVoxelResolutionMeters;            
            Vec3 chunkSize;            
            
            DistVoxelList voxels;
            ColorVoxelList colors;
            
            Vec3 origin;
            Point3 originPoint3; 

    };

    typedef std::shared_ptr<Chunk> ChunkPtr;
    typedef std::shared_ptr<const Chunk> ChunkConstPtr;
    typedef std::vector<ChunkID, Eigen::aligned_allocator<ChunkID> > ChunkIDList;

} // namespace chisel 

#endif // CHUNK_H_ 
