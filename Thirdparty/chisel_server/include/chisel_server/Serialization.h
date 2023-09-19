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


#ifndef SERIALIZATION_H_
#define SERIALIZATION_H_

#include <open_chisel/Chunk.h>
//#include <chisel_msgs/ChunkMessage.h>

namespace chisel_ros
{
    void FillChunkMessage(chisel::ChunkConstPtr chunk, chisel_msgs::ChunkMessage* message)
    {
        chisel::ChunkHasher hasher;
        assert(message != nullptr);
        message->header.stamp = ros::Time::now();

        chisel::ChunkID id = chunk->GetID();
        message->ID_x = id.x();
        message->ID_y = id.y();
        message->ID_z = id.z();
        message->spatial_hash = hasher(id);

        message->resolution_meters = chunk->GetVoxelResolutionMeters();

        Eigen::Vector3i size = chunk->GetNumVoxels();
        message->num_voxels_x = size.x();
        message->num_voxels_y = size.y();
        message->num_voxels_z = size.z();

        message->distance_data.reserve(chunk->GetTotalNumVoxels());

        if(chunk->HasColors())
        {
            message->color_data.reserve(chunk->GetTotalNumVoxels());
        }

        const std::vector<chisel::DistVoxel>& voxels = chunk->GetVoxels();

        for (const chisel::DistVoxel& voxel : voxels)
        {
            float sdf = voxel.GetSDF();
            float weight = voxel.GetWeight();
            message->distance_data.push_back
            (
                    *(reinterpret_cast<uint32_t*>(&sdf))
                  | *(reinterpret_cast<uint32_t*>(&weight)) << sizeof(uint32_t)
            );
        }

        const std::vector<chisel::ColorVoxel>& colors = chunk->GetColorVoxels();

        for (const chisel::ColorVoxel& voxel : colors)
        {
            message->color_data.push_back
            (
                      static_cast<uint32_t>(voxel.GetRed())
                    | static_cast<uint32_t>(voxel.GetBlue())   << sizeof(uint8_t)
                    | static_cast<uint32_t>(voxel.GetGreen())  << 2 * sizeof(uint8_t)
                    | static_cast<uint32_t>(voxel.GetBlue())   << 3 * sizeof(uint8_t)
                    | static_cast<uint32_t>(voxel.GetWeight()) << 4 * sizeof(uint8_t)
            );
        }

    }
}


#endif // SERIALIZATION_H_ 
