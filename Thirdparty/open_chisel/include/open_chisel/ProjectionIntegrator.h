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

#ifndef PROJECTIONINTEGRATOR_H_
#define PROJECTIONINTEGRATOR_H_

#include <iostream>

#include <open_chisel/pointcloud/PointCloud.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/Chunk.h>

#include <open_chisel/truncation/Truncator.h>
#include <open_chisel/weighting/Weighter.h>

namespace chisel
{

class ProjectionIntegrator
{
public:
    ProjectionIntegrator();
    ProjectionIntegrator(const TruncatorPtr& t,
                         const WeighterPtr& w,
                         float carvingDist,
                         bool enableCarving,
                         const Vec3List& centroids);

    virtual ~ProjectionIntegrator();

    bool Integrate(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk, const std::vector<size_t>& idx) const;
    bool IntegratePointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk, const std::vector<size_t>& idx) const;
    bool IntegrateColorPointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk, const std::vector<size_t>& idx) const;

    template<class DataType> bool Integrate(const std::shared_ptr<const DepthImage<DataType> >& depthImage,
                                            const PinholeCamera& camera,
                                            const Transform& cameraPose, Chunk* chunk) const
    {
        assert(chunk != nullptr);

        Eigen::Vector3i numVoxels = chunk->GetNumVoxels();
        float resolution = chunk->GetVoxelResolutionMeters();
        Vec3 origin = chunk->GetOrigin();
        float diag = 2.0 * sqrt(3.0f) * resolution;
        Vec3 voxelCenter;
        bool updated = false;
        for (size_t i = 0, iEnd=centroids.size(); i < iEnd; i++)
        {
            voxelCenter = centroids[i] + origin;
            Vec3 voxelCenterInCamera = cameraPose.linear().transpose() * (voxelCenter - cameraPose.translation());
            Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);

            if (!camera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
                continue;

            float voxelDist = voxelCenterInCamera.z();
            float depth = depthImage->DepthAt((int) cameraPos(1), (int) cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

            if (std::isnan(depth))
            {
                continue;
            }

            float truncation = truncator->GetTruncationDistance(depth);
            float surfaceDist = depth - voxelDist;

            if (fabs(surfaceDist) < truncation + diag)
            {
                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                voxel.Integrate(surfaceDist, 1.0f);
                updated = true;
            }
            else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
            {
                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
                {
                    voxel.Carve();
                    updated = true;
                }
            }

        }
        return updated;
    }

    template<class DataType, class ColorType> bool IntegrateColor(const std::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& depthCamera, const Transform& depthCameraPose, const std::shared_ptr<const ColorImage<ColorType> >& colorImage, const PinholeCamera& colorCamera, const Transform& colorCameraPose, Chunk* chunk) const
    {
        assert(chunk != nullptr);

        float resolution = chunk->GetVoxelResolutionMeters();
        Vec3 origin = chunk->GetOrigin();
        float resolutionDiagonal = 2.0 * sqrt(3.0f) * resolution;
        bool updated = false;
        //std::vector<size_t> indexes;
        //indexes.resize(centroids.size());
        //for (size_t i = 0; i < centroids.size(); i++)
        //{
        //    indexes[i] = i;
        //}

        for (size_t i = 0, iEnd=centroids.size(); i < iEnd; i++)
            //parallel_for(indexes.begin(), indexes.end(), [&](const size_t& i)
        {
            Color<ColorType> color;
            Vec3 voxelCenter = centroids[i] + origin;
            Vec3 voxelCenterInCamera = depthCameraPose.linear().transpose() * (voxelCenter - depthCameraPose.translation());
            Vec3 cameraPos = depthCamera.ProjectPoint(voxelCenterInCamera);

            if (!depthCamera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
            {
                continue;
            }

            float voxelDist = voxelCenterInCamera.z();
            float depth = depthImage->DepthAt((int) cameraPos(1), (int) cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

            if (std::isnan(depth))
            {
                continue;
            }

            float truncation = truncator->GetTruncationDistance(depth);
            float surfaceDist = depth - voxelDist;

            if (std::abs(surfaceDist) < truncation + resolutionDiagonal)
            {
                Vec3 voxelCenterInColorCamera = colorCameraPose.linear().transpose() * (voxelCenter - colorCameraPose.translation());
                Vec3 colorCameraPos = colorCamera.ProjectPoint(voxelCenterInColorCamera);
                if (colorCamera.IsPointOnImage(colorCameraPos))
                {
                    ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(i);

                    if (colorVoxel.GetWeight() < 5)
                    {
                        int r = static_cast<int> (colorCameraPos(1));
                        int c = static_cast<int> (colorCameraPos(0));
                        colorImage->At(r, c, &color);
                        colorVoxel.Integrate(color.red, color.green, color.blue, 1);
                    }
                }

                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                voxel.Integrate(surfaceDist, weighter->GetWeight(surfaceDist, truncation));

                updated = true;
            }
            else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
            {
                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
                {
                    voxel.Carve();
                    updated = true;
                }
            }


        }
        //);

        return updated;
    }
    
    // depth camera and color camera have the same pin-hole model (images are assumed to be registered)
    template<class DataType, class ColorType> bool IntegrateColorWithOneCameraModelBGR(const std::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& depthCamera, const Transform& depthCameraPose, const std::shared_ptr<const ColorImage<ColorType> >& colorImage, const PinholeCamera& colorCamera, const Transform& colorCameraPose, Chunk* chunk) const
    {
        assert(chunk != nullptr);

        const float resolution = chunk->GetVoxelResolutionMeters();
        const Vec3& origin = chunk->GetOrigin();
        const float resolutionDiagonal = 2.0 * sqrt(3.0f) * resolution;
        bool updated = false;
        //std::vector<size_t> indexes;
        //indexes.resize(centroids.size());
        //for (size_t i = 0; i < centroids.size(); i++)
        //{
        //    indexes[i] = i;
        //}

        const Mat3x3 rotation = depthCameraPose.linear().transpose();
        const Vec3 traslation = depthCameraPose.translation();
        for (size_t i = 0, iEnd=centroids.size(); i < iEnd; i++)
            //parallel_for(indexes.begin(), indexes.end(), [&](const size_t& i)
        {
            Color<ColorType> color;
            const Vec3& voxelCenter = centroids[i] + origin;
            const Vec3 voxelCenterInCamera =  rotation * (voxelCenter - traslation);
            const Vec3 cameraPos = depthCamera.ProjectPoint(voxelCenterInCamera);

            if (!depthCamera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
            {
                continue;
            }

            float voxelDist = voxelCenterInCamera.z();
            float depth = depthImage->DepthAt((int) cameraPos(1), (int) cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

            if (std::isnan(depth))
            {
                continue;
            }

            float truncation = truncator->GetTruncationDistance(depth);
            float surfaceDist = depth - voxelDist;

            if (std::abs(surfaceDist) < truncation + resolutionDiagonal)
            {
//                Vec3 voxelCenterInColorCamera = colorCameraPose.linear().transpose() * (voxelCenter - colorCameraPose.translation());
//                Vec3 colorCameraPos = colorCamera.ProjectPoint(voxelCenterInColorCamera);
                const Vec3& colorCameraPos = cameraPos; /// < we assume color camera has the same model as depth camera
//                if (colorCamera.IsPointOnImage(colorCameraPos))
                {
                    ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(i);

                    if (colorVoxel.GetWeight() < 5)
                    {
                        int r = static_cast<int> (colorCameraPos(1));
                        int c = static_cast<int> (colorCameraPos(0));
                        colorImage->AtBGR(r, c, &color);
                        colorVoxel.IntegrateSimple(color.red, color.green, color.blue, 1);
                    }
                }

                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                voxel.Integrate(surfaceDist, weighter->GetWeight(surfaceDist, truncation));

                updated = true;
            }
            else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
            {
                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
                {
                    //voxel.Carve();
                    voxel.Reset();
                    updated = true;
                }
            }


        }
        //);

        return updated;
    }
    
    template<class DataType> bool CarveWithDepth(const std::shared_ptr<const DepthImage<DataType> >& depthImage,
                                            const PinholeCamera& camera,
                                            //const Transform& cameraPose, 
                                            const Mat3x3& camRcw, const Vec3& camtcw, 
                                            Chunk* chunk) const
    {
        assert(chunk != nullptr);

        //const Eigen::Vector3i& numVoxels = chunk->GetNumVoxels();
        const float resolution = chunk->GetVoxelResolutionMeters();
        const Vec3& origin = chunk->GetOrigin();
        const float diag = 2.0 * sqrt(3.0f) * resolution;
        Vec3 voxelCenter;
        bool updated = false;
        
        for (size_t i = 0, iEnd = centroids.size(); i < iEnd; i++)
        {
            DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
            const float voxelWeight = voxel.GetWeight();
            if(voxelWeight <= 1e-15) continue; // if voxel is unknown go ahead 
            
            voxelCenter = centroids[i] + origin;
            
            //const Vec3 voxelCenterInCamera = cameraPose.linear().transpose() * (voxelCenter - cameraPose.translation());
            const Vec3 voxelCenterInCamera = camRcw * (voxelCenter - camtcw);
            const Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);

            if ( voxelCenterInCamera.z() < 0 || ( !camera.IsPointOnImage(cameraPos) ) )
                continue;

            const float voxelDist = voxelCenterInCamera.z();
            const float& depth = depthImage->DepthAt((int) cameraPos(1), (int) cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

            if (std::isnan(depth))
            {
                continue;
            }

            //const float truncation = truncator->GetTruncationDistance(depth);
            const float truncation = std::max(truncator->GetTruncationDistance(depth),diag); /// < truncation is here floored!              
            const float surfaceDist = depth - voxelDist;

//            if (fabs(surfaceDist) < truncation + diag)
//            {
//                DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
//                voxel.Integrate(surfaceDist, 1.0f);
//                updated = true;
//            }
//            else 
            //if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
            if (surfaceDist > truncation + carvingDist)
            {
                //DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
                //if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
                if ( voxel.GetSDF() < 1e-5 )
                {
                    //voxel.Carve();
                    voxel.Reset();
                    updated = true;
                }
            }

        }
        return updated;
    }
    

    inline const TruncatorPtr& GetTruncator() const
    {
        return truncator;
    }

    inline void SetTruncator(const TruncatorPtr& value)
    {
        truncator = value;
    }

    inline const WeighterPtr& GetWeighter() const
    {
        return weighter;
    }

    inline void SetWeighter(const WeighterPtr& value)
    {
        weighter = value;
    }

    inline float GetCarvingDist() const
    {
        return carvingDist;
    }

    inline bool IsCarvingEnabled() const
    {
        return enableVoxelCarving;
    }

    inline void SetCarvingDist(float dist)
    {
        carvingDist = dist;
    }

    inline void SetCarvingEnabled(bool enabled)
    {
        enableVoxelCarving = enabled;
    }

    inline void SetCentroids(const Vec3List& c)
    {
        centroids = c;
    }

protected:
    TruncatorPtr truncator;
    WeighterPtr weighter;
    float carvingDist;
    bool enableVoxelCarving;
    Vec3List centroids;
};

} // namespace chisel 

#endif // PROJECTIONINTEGRATOR_H_ 
