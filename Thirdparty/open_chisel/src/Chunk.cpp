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

#include <open_chisel/Chunk.h>

namespace chisel
{

    Chunk::Chunk() :
            voxelResolutionMeters(0)
    {
        // TODO Auto-generated constructor stub

    }

    Chunk::Chunk(const ChunkID id, const Eigen::Vector3i& nv, float r, bool useColor) :
            ID(id), numVoxels(nv), voxelResolutionMeters(r)
    {
        
        invVoxelResolutionMeters = 1.0f / (voxelResolutionMeters);
        chunkSize = numVoxels.cast<float>() * voxelResolutionMeters;
        
        AllocateDistVoxels();

        if(useColor)
        {
            AllocateColorVoxels();
        }

        origin = Vec3(numVoxels(0) * ID(0) * voxelResolutionMeters, numVoxels(1) * ID(1) * voxelResolutionMeters, numVoxels(2) * ID(2) * voxelResolutionMeters);
        
        originPoint3 = Point3(ID.x() * numVoxels.x(), ID.y() * numVoxels.y(), ID.z() * numVoxels.z());
    }

    Chunk::~Chunk()
    {

    }

    void Chunk::AllocateDistVoxels()
    {
        const int totalNum = GetTotalNumVoxels();
        //voxels.clear();
        voxels.resize(totalNum, DistVoxel());
    }

    void Chunk::AllocateColorVoxels()
    {
        const int totalNum = GetTotalNumVoxels();
        //colors.clear();
        colors.resize(totalNum, ColorVoxel());
    }

    AABB Chunk::ComputeBoundingBox()
    {
        const Vec3& pos = origin;
        //const Vec3 size = numVoxels.cast<float>() * voxelResolutionMeters;
        const Vec3& size = chunkSize;
        return AABB(pos, pos + size);
    }

    Point3 Chunk::GetVoxelCoords(const Vec3& worldCoords) const
    {
        //const float roundingFactorX = 1.0f / (voxelResolutionMeters);
        //const float roundingFactorY = 1.0f / (voxelResolutionMeters);
        //const float roundingFactorZ = 1.0f / (voxelResolutionMeters);

        return Point3( static_cast<int>(std::floor(worldCoords(0) * invVoxelResolutionMeters)),
                       static_cast<int>(std::floor(worldCoords(1) * invVoxelResolutionMeters)),
                       static_cast<int>(std::floor(worldCoords(2) * invVoxelResolutionMeters)));
    }

    VoxelID Chunk::GetVoxelID(const Vec3& relativePos) const
    {
        return GetVoxelID(GetVoxelCoords(relativePos));
    }

    VoxelID Chunk::GetLocalVoxelIDFromGlobal(const Point3& worldPoint) const
    {
        return GetVoxelID(GetLocalCoordsFromGlobal(worldPoint));
    }

    Point3 Chunk::GetLocalCoordsFromGlobal(const Point3& worldPoint) const
    {
        //return (worldPoint - Point3(ID.x() * numVoxels.x(), ID.y() * numVoxels.y(), ID.z() * numVoxels.z()));
        return (worldPoint - originPoint3);        
    }


    void Chunk::ComputeStatistics(ChunkStatistics* stats)
    {
        assert(stats != nullptr);

        for (const DistVoxel& vox : voxels)
        {
            const float weight = vox.GetWeight();
            if (weight > 0)
            {
                const float sdf = vox.GetSDF();
                if (sdf < 0)
                {
                    stats->numKnownInside++;
                }
                else
                {
                    stats->numKnownOutside++;
                }
            }
            else
            {
                stats->numUnknown++;
            }

            stats->totalWeight += weight;

        }
    }

    Vec3 Chunk::GetColorAt(const Vec3& pos)
    {
        if(ComputeBoundingBox().Contains(pos))
        {
            const Vec3 chunkPos = (pos - origin) * invVoxelResolutionMeters;
            const int chunkX = static_cast<int>(chunkPos(0));
            const int chunkY = static_cast<int>(chunkPos(1));
            const int chunkZ = static_cast<int>(chunkPos(2));

            if(IsCoordValid(chunkX, chunkY, chunkZ))
            {
                const ColorVoxel& color = GetColorVoxel(chunkX, chunkY, chunkZ);
                const float invMaxVal = 1.f/static_cast<float>(std::numeric_limits<uint8_t>::max());
                return Vec3(static_cast<float>(color.GetRed()) * invMaxVal, static_cast<float>(color.GetGreen()) * invMaxVal, static_cast<float>(color.GetBlue()) * invMaxVal);
            }
        }

        return Vec3::Zero();
    }


} // namespace chisel 
