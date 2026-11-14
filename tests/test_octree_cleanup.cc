#include "Map.h"
#include "PointCloudMapOctreePointCloud.h"

#include <iostream>
#include <memory>
#include <stdexcept>

using namespace PLVS2;

static void check(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

struct TestMap : PointCloudMapOctreePointCloud<pcl::PointSurfelSegment>
{
    using Base = PointCloudMapOctreePointCloud<pcl::PointSurfelSegment>;
    using Base::Base;
    using Base::octree_;
    using Base::lastTimestamp_;
};

static pcl::PointSurfelSegment point(float x)
{
    pcl::PointSurfelSegment p;
    p.x = x;
    p.y = p.z = 0.5f;
    p.normal_x = 1.0f;
    p.normal_y = p.normal_z = 0.0f;
    return p;
}

int main(int argc, char**)
{
    try
    {
        Map sparseMap;
        auto params = std::make_shared<PointCloudMapParameters>();
        params->resolution = 1.0;
        params->nPointCounterThreshold = 3;
        TestMap map(&sparseMap, params);
        map.octree_.defineBoundingBox(0, 0, 0, 8, 8, 8);
        map.lastTimestamp_ = TestMap::kDeltaTimeForCleaningUnstablePointsUs + 2;

        TestMap::PointCloudT::Ptr oldCloud(new TestMap::PointCloudT());
        oldCloud->header.stamp = 1;
        oldCloud->push_back(point(0.5f));
        for (int i = 0; i < 3; ++i)
            oldCloud->push_back(point(1.5f));
        oldCloud->push_back(point(2.5f));
        map.InsertCloud(oldCloud);

        // Reproduce the state caused by centroid rounding across a voxel face.
        // Set it explicitly so the regression is independent of FMA/compiler flags.
        auto* leaf = map.octree_.findLeafAtPointPublic(point(0.5f));
        check(leaf != nullptr, "Missing unstable leaf");
        leaf->getCentroid().x = 1.0f;

        TestMap::PointCloudT::Ptr recentCloud(new TestMap::PointCloudT());
        recentCloud->header.stamp = map.lastTimestamp_ - 1;
        recentCloud->push_back(point(3.5f));
        map.InsertCloud(recentCloud);

        for (int pass = 0; pass < 2; ++pass)
        {
            const int count = argc > 1 ? map.UpdateMapSegm() : map.UpdateMapNoSegm();
            check(count == 1, "Stable output leaf was lost");
            check(map.GetMap()->points.front().x == 1.5f, "Output contains the wrong leaf");
            check(!map.octree_.isVoxelOccupiedAtPoint(point(0.5f)), "Drifted unstable leaf survived");
            check(map.octree_.isVoxelOccupiedAtPoint(point(1.5f)), "Stable neighbor was deleted");
            check(!map.octree_.isVoxelOccupiedAtPoint(point(2.5f)), "Second old unstable leaf survived");
            check(map.octree_.isVoxelOccupiedAtPoint(point(3.5f)), "Recent unstable leaf was deleted");
            check(map.octree_.getLeafCount() == 2, "Unexpected remaining leaf count");
        }
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
