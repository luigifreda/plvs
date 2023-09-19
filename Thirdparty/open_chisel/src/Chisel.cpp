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

#include <open_chisel/Chisel.h>

#include <open_chisel/io/PLY.h>

#include <open_chisel/geometry/Raycast.h>

#include <iostream>
#include <vector>
#include <unordered_map>

namespace chisel
{

Chisel::Chisel()
{
    // TODO Auto-generated constructor stub
}

Chisel::Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor) 
: chunkManager(chunkSize, voxelResolution, useColor)
{
    //std::cout << "Chisel::Chisel() - centroids size : " << chunkManager.GetCentroids().size() << std::endl;  
}

Chisel::~Chisel()
{
    // TODO Auto-generated destructor stub
}

void Chisel::Reset()
{
    chunkManager.Reset();
    meshesToUpdate.clear();
}

void Chisel::UpdateMeshes()
{
#if 0    
    chunkManager.RecomputeMeshes(meshesToUpdate);
#else    
    chunkManager.RecomputeMeshesParallel(meshesToUpdate);    
#endif    
    meshesToUpdate.clear();
}

void Chisel::GarbageCollect(const ChunkIDList& chunks)
{
    std::cout << chunkManager.GetChunks().size() << " chunks " << chunkManager.GetAllMeshes().size() << " meshes before collect." << std::endl;
    //for (const ChunkID& chunkID : chunks)
    for (const auto& chunkID : chunks)
    {
        chunkManager.RemoveChunk(chunkID);
        meshesToUpdate.erase(chunkID);
    }
    std::cout << chunkManager.GetChunks().size() << " chunks " << chunkManager.GetAllMeshes().size() << " meshes after collect." << std::endl;
}

bool Chisel::SaveAllMeshesToPLY(const std::string& filename)
{
    printf("Saving all meshes to PLY file...\n");

    chisel::MeshPtr fullMesh(new chisel::Mesh());

    size_t v = 0;
    for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
    {
        for (const Vec3& vert : it.second->vertices)
        {
            fullMesh->vertices.push_back(vert);
            fullMesh->indices.push_back(v);
            v++;
        }

        for (const Vec3& color : it.second->colors)
        {
            fullMesh->colors.push_back(color);
        }

        for (const Vec3& normal : it.second->normals)
        {
            fullMesh->normals.push_back(normal);
        }
    }

    printf("Full mesh has %lu verts\n", v);
    bool success = SaveMeshPLYASCII(filename, fullMesh);

    if (!success)
    {
        printf("Saving failed!\n");
    }

    return success;
}


void Chisel::IntegratePointCloud(const ProjectionIntegrator& integrator,
                                 const PointCloud& cloud, // w.r.t camera frame 
                                 const Transform& cameraPose,
                                 float maxDist)
{

    const float roundToVoxel = 1.0f / chunkManager.GetResolution();
    const Vec3 halfVoxel = Vec3(0.5, 0.5, 0.5) * chunkManager.GetResolution();
    //std::unordered_map<ChunkID, bool, ChunkHasher> updated;
    ChunkSet updatedChunks;
    //std::unordered_map<ChunkID, bool, ChunkHasher> newChunks;
    ChunkSet newChunks;
    const Vec3 startCamera = cameraPose.translation();
    const Transform inversePose = cameraPose.inverse();
    const Point3 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
    const Point3 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    const size_t numPoints = cloud.GetPoints().size();
    const bool carving = integrator.IsCarvingEnabled();
    const float carvingDist = integrator.GetCarvingDist();

    for (size_t i = 0; i < numPoints; i++)
    {
        Point3List raycastVoxels;
        //const Vec3& point = cloud.GetPoints().at(i);
        const Vec3& point = cloud.GetPoints()[i];
        const Vec3& color = cloud.GetColors()[i]; // added for color integration 
        const Vec3 worldPoint = cameraPose * point;
        const float depth = point.z();

        if (depth < 0.01f) continue;

        Vec3 dir = (worldPoint - startCamera).normalized();
        float truncation = integrator.GetTruncator()->GetTruncationDistance(depth);
        const Vec3 scaledWorldPoint = worldPoint * roundToVoxel;
        const Vec3 scaledDirXTruncation = dir * truncation * roundToVoxel;
        //Vec3 start = carving ? static_cast<Vec3> (startCamera * roundToVoxel) : static_cast<Vec3> ( (worldPoint - dir * truncation) * roundToVoxel);
        Vec3 start = carving ? static_cast<Vec3> (startCamera * roundToVoxel) : static_cast<Vec3> ( scaledWorldPoint - scaledDirXTruncation );
        //Vec3 end = (worldPoint + dir * truncation) * roundToVoxel;
        Vec3 end = scaledWorldPoint + scaledDirXTruncation;

        raycastVoxels.clear();
        Raycast(start, end, minVal, maxVal, &raycastVoxels);
        if (raycastVoxels.size() > 500)
        {
            ///std::cerr << "Error, too many racyast voxels!!!" << std::endl;
            /// < throw std::out_of_range("Too many raycast voxels");
            std::cerr << "WARNING: too many racyast voxels!!!" << std::endl;
        }

        //std::cout << "chunk: " << chunk->GetID().transpose() << " from "
        //        << start.transpose() << " to " << end.transpose() << " : " << raycastVoxels.size() << std::endl;
        for (const Point3& voxelCoords : raycastVoxels)
        {
            Vec3 center = chunkManager.GetCentroid(voxelCoords);
            bool wasNew = false;
            ChunkPtr chunk = chunkManager.GetOrCreateChunkAt(center, &wasNew);
            if (wasNew)
            {
                newChunks[chunk->GetID()] = true;
                updatedChunks[chunk->GetID()] = false;
            }

            VoxelID id = chunk->GetLocalVoxelIDFromGlobal(voxelCoords);
            if (!chunk->IsCoordValid(id))
            {
                continue;
            }

            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            ColorVoxel& voxel = chunk->GetColorVoxelMutable(id); // added for color integration 

            const float u = depth - (inversePose * (center)).z();
            const float weight = integrator.GetWeighter()->GetWeight(u, truncation);

            if (fabs(u) < truncation)
            {
                distVoxel.Integrate(u, weight);
                voxel.Integrate((uint8_t) (color.x() * 255.0f), (uint8_t) (color.y() * 255.0f), (uint8_t) (color.z() * 255.0f), 1); // added for color integration 
                updatedChunks[chunk->GetID()] = true;
            }
            else if (!wasNew && carving && (u > truncation + carvingDist) )
            {
                if (distVoxel.GetWeight() > 0 && distVoxel.GetSDF() < 0.0f)
                {
                    distVoxel.Carve();
                    updatedChunks[chunk->GetID()] = true;
                }
            }
        }
    }

    for (const auto& updatedChunk : updatedChunks)
    {
        if (updatedChunk.second)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        meshesToUpdate[updatedChunk.first + ChunkID(dx, dy, dz)] = true;
                    }
                }
            }
        }
    }
    ChunkIDList garbageChunks;
    for (const auto& newChunk : newChunks)
    {
        if (!updatedChunks[newChunk.first])        
        {
            garbageChunks.push_back(newChunk.first);
        }
    }

    GarbageCollect(garbageChunks);
}


void Chisel::IntegrateWorldPointCloudWithNormals(const ProjectionIntegrator& integrator,
                                 const PointCloud& cloud, // w.r.t camera frame 
                                 const Transform& cameraPose, // Twc
                                 float maxDist)
{

    const float roundToVoxel = 1.0f / chunkManager.GetResolution();
    const Vec3 halfVoxel = Vec3(0.5, 0.5, 0.5) * chunkManager.GetResolution();
    //std::unordered_map<ChunkID, bool, ChunkHasher> updated;
    ChunkSet updated;
    //std::unordered_map<ChunkID, bool, ChunkHasher> newChunks;
    ChunkSet newChunks;
    
    // < here we use normals to identify the "sdf truncation segment"
    //const Vec3 startCamera = cameraPose.translation();
    
    const Transform inversePose = cameraPose.inverse();
    const Point3 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
    const Point3 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    const size_t numPoints = cloud.GetPoints().size();
    
    // <  no carving needed here 
    //const bool carving = integrator.IsCarvingEnabled();
    //const float carvingDist = integrator.GetCarvingDist();
    
    //const float delta = 0.3 * chunkManager.GetResolution();
    //const Vec3 vDelta()
    
    const float truncation = 4 * chunkManager.GetResolution();    

    for (size_t i = 0; i < numPoints; i++)
    {
        Point3List raycastVoxels;
        //const Vec3& point = cloud.GetPoints().at(i);
        const Vec3& point = cloud.GetPoints()[i];
        const Vec3& color = cloud.GetColors()[i];  
        const Vec3& normal = cloud.GetNormals()[i];       
        const uint32_t kfid = cloud.GetKfids()[i];        
        const Vec3 worldPoint = cameraPose * point;
        //const float depth = point.z();

        //if (depth < 0.001f) continue;

        //Vec3 dir = (worldPoint - startCamera).normalized();
        Vec3 dir = normal.normalized();
        //std::cout << "dir: " << dir << std::endl; 
        
        //const float truncation = integrator.GetTruncator()->GetTruncationDistance(depth);
        //const float truncation = 4 * chunkManager.GetResolution();
        
        const Vec3 scaledWorldPoint = worldPoint * roundToVoxel;
        const Vec3 scaledDirXTruncation = dir * truncation * roundToVoxel;
        //Vec3 start = carving ? static_cast<Vec3> (startCamera * roundToVoxel) : static_cast<Vec3> ( (worldPoint - dir * truncation) * roundToVoxel);
        Vec3 start = scaledWorldPoint - scaledDirXTruncation;
        //Vec3 end = (worldPoint + dir * truncation) * roundToVoxel;
        Vec3 end = scaledWorldPoint + scaledDirXTruncation;

        raycastVoxels.clear();
        
        Raycast(start, end, minVal, maxVal, &raycastVoxels);
                
        if (raycastVoxels.size() > 500)
        {
            ///std::cerr << "Error, too many racyast voxels!!!" << std::endl;
            /// < throw std::out_of_range("Too many raycast voxels");
            std::cerr << "WARNING: too many racyast voxels!!!" << std::endl;
        }

        //std::cout << "chunk: " << chunk->GetID().transpose() << " from "
        //        << start.transpose() << " to " << end.transpose() << " : " << raycastVoxels.size() << std::endl;
        for (const Point3& voxelCoords : raycastVoxels)
        {
            Vec3 center = chunkManager.GetCentroid(voxelCoords);
            bool wasNew = false;
            ChunkPtr chunk = chunkManager.GetOrCreateChunkAt(center, &wasNew);
            if (wasNew)
            {
                newChunks[chunk->GetID()] = true;
                updated[chunk->GetID()] = false;
            }

            VoxelID id = chunk->GetLocalVoxelIDFromGlobal(voxelCoords);
            if (!chunk->IsCoordValid(id))
            {
                continue;
            }

            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(id); // added for color integration 

            //const float u = depth - (inversePose * (center)).z();  // inversePose = Tcw
            const float u = (center - worldPoint).dot(dir);
            const float weight = integrator.GetWeighter()->GetWeight(u, truncation);
            //const float weight = 1.0;

            if (fabs(u) < truncation)
            {
                distVoxel.Integrate(u, weight);
                distVoxel.SetKfid(kfid);
                
                colorVoxel.Integrate((uint8_t) (color.x() * 255.0f), (uint8_t) (color.y() * 255.0f), (uint8_t) (color.z() * 255.0f), 1); // added for color integration 
                updated[chunk->GetID()] = true;
            }
            /// < no carving 
            /*else if (!wasNew && carving && (u > truncation + carvingDist) )
            {
                if (distVoxel.GetWeight() > 0 && distVoxel.GetSDF() < 0.0f)
                {
                    distVoxel.Carve();
                    updated[chunk->GetID()] = true;
                }
            }*/
        }
    }

    for (const auto& updatedChunk : updated)
    {
        if (updatedChunk.second)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        meshesToUpdate[updatedChunk.first + ChunkID(dx, dy, dz)] = true;
                    }
                }
            }
        }
    }
    ChunkIDList garbageChunks;
    for (const auto& newChunk : newChunks)
    {
        if (!updated[newChunk.first])
        {
            garbageChunks.push_back(newChunk.first);
        }
    }

    GarbageCollect(garbageChunks);
}


template <class DataType> void Chisel::IntegratePointCloudWidthDepth(const ProjectionIntegrator& integrator,
                                                                     const PointCloud& cloud,
                                                                     const Transform& cameraPose,
                                                                     const std::shared_ptr<const DepthImage<DataType> >& depthImage,
                                                                     const PinholeCamera& camera,
                                                                     float maxDist)
{
    //std::unordered_map<ChunkID, bool, ChunkHasher> updated;
    ChunkSet updatedChunks;
    //std::unordered_map<ChunkID, bool, ChunkHasher> newChunks;
    ChunkSet newChunks;    

    /// < carving with depth image
    int num_carved_chunks = 0; 
    if (integrator.IsCarvingEnabled())
    {
        Frustum frustum;
        camera.SetupFrustum(cameraPose, &frustum);

        ChunkIDList chunksIntersecting;
        chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

        const Mat3x3 camRcw = cameraPose.linear().transpose();
        const Vec3 camtcw = cameraPose.translation();
        
        //for (const ChunkID& chunkID : chunksIntersecting)
        for (const auto& chunkID : chunksIntersecting)            
        {
            //if ( chunkManager.HasMesh(chunkID))
            ChunkMap::iterator itc = chunkManager.FindChunk(chunkID);
            if (itc != chunkManager.ChunksEnd() )
            {
                //ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                ChunkPtr& chunk = itc->second;

                bool needsUpdate = integrator.CarveWithDepth(depthImage, camera, camRcw, camtcw, chunk.get());
                if (needsUpdate)
                {
                    num_carved_chunks++;
#if 0                    
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            for (int dz = -1; dz <= 1; dz++)
                            {
                                meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                            }
                        }
                    }
#else           
                    meshesToUpdate[chunkID] = true;
#endif                    

                }
            }
        }
        std::cout << "carved in "<< num_carved_chunks << " chunks " << std::endl; 
    }

    /// < point cloud part 
    const float resolution = chunkManager.GetResolution();
    const float roundToVoxel = 1.0f / resolution;
    const Vec3 halfVoxel = Vec3(0.5, 0.5, 0.5) * chunkManager.GetResolution();
    
    const float diag = 2.0f * sqrt(3.0f) * resolution;
    //float diag = sqrt(3.0f) * resolution;

    const Vec3 startCamera = cameraPose.translation();
    const Transform inversePose = cameraPose.inverse();
    const Point3 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
    const Point3 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    const size_t numPoints = cloud.GetPoints().size();
    
    const bool carving = false; // integrator.IsCarvingEnabled(); /// < disable carving since we already did it above 
    const float carvingDist = integrator.GetCarvingDist();
    
    //const Vec3 startCameraXroundToVoxel = startCamera * roundToVoxel;

    //std::cout << "PointCloud #points: " << numPoints << ", #colors: " << cloud.GetColors().size() << ", #kfids: " << cloud.GetKfids().size() << std::endl; 
    
    for (size_t i = 0; i < numPoints; i++)
    {
        Point3List raycastVoxels;
        
        //const Vec3& point = cloud.GetPoints().at(i);
        const Vec3& point = cloud.GetPoints()[i];
        const Vec3& color = cloud.GetColors()[i]; // added for color integration 
        const uint32_t kfid = cloud.GetKfids()[i];
        
        Vec3 worldPoint = cameraPose * point;
        const float depth = point.z();

        if (depth < 0.01f) continue;

        const Vec3 dir = (worldPoint - startCamera).normalized();
        //const float truncation = std::min(integrator.GetTruncator()->GetTruncationDistance(depth),diag); /// < truncation is here capped! 
        const float truncation = std::max(integrator.GetTruncator()->GetTruncationDistance(depth),diag); /// < truncation is here floored!  
        //Vec3 start = carving ? startCameraXroundToVoxel: static_cast<Vec3> ((worldPoint - dir * truncation) * roundToVoxel);
        
        const Vec3 scaledWorldPoint = worldPoint * roundToVoxel;
        const Vec3 scaledDirXTruncation = dir * truncation * roundToVoxel;
        //Vec3 start = carving ? static_cast<Vec3> (startCamera * roundToVoxel) : static_cast<Vec3> ( (worldPoint - dir * truncation) * roundToVoxel);
        //const Vec3 start = ((worldPoint - dir * truncation) * roundToVoxel);        
        Vec3 start = scaledWorldPoint - scaledDirXTruncation;
        //Vec3 end = (worldPoint + dir * truncation) * roundToVoxel;
        Vec3 end = scaledWorldPoint + scaledDirXTruncation;        
                
        raycastVoxels.clear();
        Raycast(start, end, minVal, maxVal, &raycastVoxels); // rasterization
        if (raycastVoxels.size() > 500)
        {
            ///std::cerr << "Error, too many racyast voxels!!!" << std::endl;
            /// < throw std::out_of_range("Too many raycast voxels");
            std::cerr << "WARNING: too many racyast voxels!!! - truncation distance: " << truncation << std::endl;
        }

        //std::cout << "chunk: " << chunk->GetID().transpose() << " from "
        //        << start.transpose() << " to " << end.transpose() << " : " << raycastVoxels.size() << std::endl;
        
        //for (const Point3& voxelCoords : raycastVoxels)
        for (const auto& voxelCoords : raycastVoxels)
        {
            const Vec3 center = chunkManager.GetCentroid(voxelCoords);
            bool wasNew = false;
            ChunkPtr chunk = chunkManager.GetOrCreateChunkAt(center, &wasNew);
            if (wasNew)
            {
                newChunks[chunk->GetID()] = true;
                updatedChunks[chunk->GetID()] = false;
            }

            VoxelID id = chunk->GetLocalVoxelIDFromGlobal(voxelCoords);
            if (!chunk->IsCoordValid(id))
            {
                continue;
            }

            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(id); // color integration 

            //const float u = depth - (inversePose * (center)).z(); /// < original version 
            
            const Vec3 center_c  = (inversePose * (center));  
            const float length = center_c.norm(); 
            const float u = length*(depth/center_c.z() - 1);
            
            const float weight = integrator.GetWeighter()->GetWeight(u, truncation);

            if (fabs(u) < truncation)
            {
                distVoxel.Integrate(u, weight);
                distVoxel.SetKfid(kfid);
                //voxel.Integrate((uint8_t) (color.x() * 255.0f), (uint8_t) (color.y() * 255.0f), (uint8_t) (color.z() * 255.0f), 1); // added for color integration 
                colorVoxel.IntegrateSimple((uint8_t) (color.x() * 255.0f), (uint8_t) (color.y() * 255.0f), (uint8_t) (color.z() * 255.0f), 1); // added for color integration 
                updatedChunks[chunk->GetID()] = true;
            }
#if 0            
            else if (!wasNew && carving && u > truncation + carvingDist)
            {
                if (distVoxel.GetWeight() > 0 && distVoxel.GetSDF() < 0.0f)
                {
                    distVoxel.Carve();
                    updatedChunks[chunk->GetID()] = true;
                }
            }
#endif            
        }
    }

    //for (const std::pair < ChunkID, bool>& updatedChunk : updatedChunks)
    for (const auto& updatedChunk : updatedChunks)
    {
#if 1        
        if (updatedChunk.second)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        meshesToUpdate[updatedChunk.first + ChunkID(dx, dy, dz)] = true;
                    }
                }
            }
        }
#else
        if (updatedChunk.second) meshesToUpdate[updatedChunk.first] = true;
#endif        
    }
    
    ChunkIDList garbageChunks;
    for (const auto& newChunk : newChunks)
    {
        if (!updatedChunks[newChunk.first])
        {
            garbageChunks.push_back(newChunk.first);
        }
    }

    GarbageCollect(garbageChunks);

}


void Chisel::Deform(MapKfidRt& deformationMap)   
{
    chunkManager.Deform(deformationMap, meshesToUpdate);
}




template void Chisel::IntegratePointCloudWidthDepth(const ProjectionIntegrator& integrator,
                                                    const PointCloud& cloud,
                                                    const Transform& extrinsic,
                                                    const std::shared_ptr<const DepthImage<float> >& depthImage,
                                                    const PinholeCamera& camera,
                                                    float maxDist);

template void Chisel::IntegratePointCloudWidthDepth(const ProjectionIntegrator& integrator,
                                                    const PointCloud& cloud,
                                                    const Transform& extrinsic,
                                                    const std::shared_ptr<const DepthImage<double> >& depthImage,
                                                    const PinholeCamera& camera,
                                                    float maxDist);


} // namespace chisel 
