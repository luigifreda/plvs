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

#include <chisel_server/ChiselServer.h>
#include <chisel_server/Conversions.h>
//#include <chisel_ros/Serialization.h>
//#include <visualization_msgs/Marker.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace chisel_server
{

// return microseconds timestamp

static boost::uint64_t getTimestamp()
{
    using namespace boost::posix_time;
    static ptime epoch(boost::gregorian::date(1970, 1, 1));
    ptime now = boost::posix_time::microsec_clock::local_time();
    return (now - epoch).total_microseconds();
}

ChiselServerParams::ChiselServerParams()
{
    /// < default settings 

    chunkSizeX = 16;
    chunkSizeY = 16;
    chunkSizeZ = 16;

    voxelResolution = 0.015;
    
    // sigma model from the paper "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking"
    // sigma(z) = 0.0012 + 0.0019*(z-0.4)^2
    truncationDistQuad = 0.0019;
    truncationDistLinear = -0.00152; // -2*0.4*0.0019
    truncationDistConst = 0.001504; // 0.4^2 * 0.0019 + 0.0012
    truncationDistScale = 6.0; // in order to consider +- 6*sigma
    weight = 1;
    useCarving = true;
    useColor   = true;
    saveFile = true;
    carvingDist = 0.05;

    nearPlaneDist = 0.05;
    farPlaneDist = 5.0;
    fusionMode = 1;
}

ChiselServer::ChiselServer() :
useColor(false), hasNewData(false), nearPlaneDist(0.05), farPlaneDist(5), isPaused(false), mode(FusionMode::DepthImage)
{
    pDepthCamera = std::allocate_shared<RosCameraTopic>(Eigen::aligned_allocator<RosCameraTopic>());
    pColorCamera = std::allocate_shared<RosCameraTopic>(Eigen::aligned_allocator<RosCameraTopic>());    
    pPointcloudTopic = std::allocate_shared<RosPointCloudTopic>(Eigen::aligned_allocator<RosPointCloudTopic>());          
}

//    ChiselServer::ChiselServer(const ros::NodeHandle& nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode) :
//            nh(nodeHanlde), useColor(color), hasNewData(false), isPaused(false), mode(fusionMode), farPlaneDist(0), nearPlaneDist(0)
//    {
//        chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
//    }

ChiselServer::ChiselServer(int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode) :
useColor(color), hasNewData(false), isPaused(false), mode(fusionMode), farPlaneDist(0), nearPlaneDist(0)
{
    pDepthCamera = std::allocate_shared<RosCameraTopic>(Eigen::aligned_allocator<RosCameraTopic>());
    pColorCamera = std::allocate_shared<RosCameraTopic>(Eigen::aligned_allocator<RosCameraTopic>());    
    pPointcloudTopic = std::allocate_shared<RosPointCloudTopic>(Eigen::aligned_allocator<RosPointCloudTopic>());     
    
    chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
}

ChiselServer::ChiselServer(ChiselServerParams& params) :
useColor(params.useColor), hasNewData(false), isPaused(false),
mode((ChiselServer::FusionMode)params.fusionMode), farPlaneDist(params.farPlaneDist), nearPlaneDist(params.nearPlaneDist)
{
    pDepthCamera = std::allocate_shared<RosCameraTopic>(Eigen::aligned_allocator<RosCameraTopic>());
    pColorCamera = std::allocate_shared<RosCameraTopic>(Eigen::aligned_allocator<RosCameraTopic>());    
    pPointcloudTopic = std::allocate_shared<RosPointCloudTopic>(Eigen::aligned_allocator<RosPointCloudTopic>());       
    
    chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(params.chunkSizeX, params.chunkSizeY, params.chunkSizeZ), params.voxelResolution, params.useColor));

    chisel::Vec4 truncation(params.truncationDistQuad, params.truncationDistLinear, params.truncationDistConst, params.truncationDistScale);

    projectionIntegrator = chisel::ProjectionIntegrator();
    SetupProjectionIntegrator(truncation, static_cast<uint16_t> (params.weight), params.useCarving, params.carvingDist);
}

ChiselServer::~ChiselServer()
{

}

void ChiselServer::Reset()
{
    chiselMap->Reset();
}

void ChiselServer::AdvertiseServices()
{
    //        resetServer = nh.advertiseService("Reset", &ChiselServer::Reset, this);
    //        pauseServer = nh.advertiseService("TogglePaused", &ChiselServer::TogglePaused, this);
    //        saveMeshServer = nh.advertiseService("SaveMesh", &ChiselServer::SaveMesh, this);
    //        getAllChunksServer = nh.advertiseService("GetAllChunks", &ChiselServer::GetAllChunks, this);
}

//    void ChiselServer::SetupMeshPublisher(const std::string& topic)
//    {
//        meshTopic = topic;
////        meshPublisher = nh.advertise<visualization_msgs::Marker>(meshTopic, 1);
//    }

//    void ChiselServer::PublishMeshes()
//    {
////        visualization_msgs::Marker marker;
////        FillMarkerTopicWithMeshes(&marker);
////
////        if(!marker.points.empty())
////            meshPublisher.publish(marker);
//    }
//
//
//    void ChiselServer::SetupChunkBoxPublisher(const std::string& boxTopic)
//    {
//        chunkBoxTopic = boxTopic;
////        chunkBoxPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic, 1);
////        latestChunkPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic + "/latest", 1);
//    }
//
//    void ChiselServer::SetupDepthPosePublisher(const std::string& depthPoseTopic)
//    {
////        depthCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(depthPoseTopic, 1);
//    }
//
//    void ChiselServer::SetupColorPosePublisher(const std::string& colorPoseTopic)
//    {
////        colorCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(colorPoseTopic, 1);
//    }
//
//    void ChiselServer::SetupDepthFrustumPublisher(const std::string& frustumTopic)
//    {
////        depthCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
//    }
//
//    void ChiselServer::SetupColorFrustumPublisher(const std::string& frustumTopic)
//    {
////        colorCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
//    }

void ChiselServer::PublishDepthFrustum()
{
    chisel::Frustum frustum;
    pDepthCamera->cameraModel.SetupFrustum(pDepthCamera->lastPose, &frustum);
    //        visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
    //        depthCamera.frustumPublisher.publish(marker);
}

void ChiselServer::PublishColorFrustum()
{
    chisel::Frustum frustum;
    pColorCamera->cameraModel.SetupFrustum(pColorCamera->lastPose, &frustum);
    //        visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
    //        colorCamera.frustumPublisher.publish(marker);
}

//    visualization_msgs::Marker ChiselServer::CreateFrustumMarker(const chisel::Frustum& frustum)
//    {
//        visualization_msgs::Marker marker;
//        marker.id = 0;
//        marker.header.frame_id = baseTransform;
//        marker.color.r = 1.;
//        marker.color.g = 1.;
//        marker.color.b = 1.;
//        marker.color.a = 1.;
//        marker.pose.position.x = 0;
//        marker.pose.position.y = 0;
//        marker.pose.position.z = 0;
//        marker.pose.orientation.x = 0.0;
//        marker.pose.orientation.y = 0.0;
//        marker.pose.orientation.z = 0.0;
//        marker.pose.orientation.w = 1.0;
//        marker.scale.x = 0.01;
//        marker.scale.y = 0.01;
//        marker.scale.z = 0.01;
//
//        marker.type = visualization_msgs::Marker::LINE_LIST;
//        const chisel::Vec3* lines = frustum.GetLines();
//        for (int i = 0; i < 24; i++)
//        {
//            const chisel::Vec3& linePoint = lines[i];
//            geometry_msgs::Point pt;
//            pt.x = linePoint.x();
//            pt.y = linePoint.y();
//            pt.z = linePoint.z();
//            marker.points.push_back(pt);
//        }
//
//        return marker;
//    }

//    void ChiselServer::PublishDepthPose()
//    {
//        chisel::Transform lastPose = depthCamera.lastPose;
//
////        geometry_msgs::PoseStamped pose;
////        pose.header.frame_id = baseTransform;
////        pose.header.stamp = depthCamera.lastImageTimestamp;
////        pose.pose.position.x = lastPose.translation()(0);
////        pose.pose.position.y = lastPose.translation()(1);
////        pose.pose.position.z = lastPose.translation()(2);
////
////        chisel::Quaternion quat(lastPose.rotation());
////        pose.pose.orientation.x = quat.x();
////        pose.pose.orientation.y = quat.y();
////        pose.pose.orientation.z = quat.z();
////        pose.pose.orientation.w = quat.w();
//
////        depthCamera.lastPosePublisher.publish(pose);
//    }
//
//    void ChiselServer::PublishColorPose()
//    {
//        chisel::Transform lastPose = colorCamera.lastPose;
//
////        geometry_msgs::PoseStamped pose;
////        pose.header.frame_id = baseTransform;
////        pose.header.stamp = colorCamera.lastImageTimestamp;
////        pose.pose.position.x = lastPose.translation()(0);
////        pose.pose.position.y = lastPose.translation()(1);
////        pose.pose.position.z = lastPose.translation()(2);
////
////        chisel::Quaternion quat(lastPose.rotation());
////        pose.pose.orientation.x = quat.x();
////        pose.pose.orientation.y = quat.y();
////        pose.pose.orientation.z = quat.z();
////        pose.pose.orientation.w = quat.w();
////
////        colorCamera.lastPosePublisher.publish(pose);
//    }




//    bool ChiselServer::TogglePaused(chisel_msgs::PauseService::Request& request, chisel_msgs::PauseService::Response& response)
//    {
//        SetPaused(!IsPaused());
//        return true;
//    }
//
//    bool ChiselServer::Reset(chisel_msgs::ResetService::Request& request, chisel_msgs::ResetService::Response& response)
//    {
//        chiselMap->Reset();
//        return true;
//    }

void ChiselServer::SubscribeDepthImage(const std::string& imageTopic, const std::string& infoTopic, const std::string& transform)
{
    pDepthCamera->imageTopic = imageTopic;
    pDepthCamera->transform = transform;
    pDepthCamera->infoTopic = infoTopic;
    //        depthCamera.imageSubscriber = nh.subscribe(depthCamera.imageTopic, 20, &ChiselServer::DepthImageCallback, this);
    //        depthCamera.infoSubscriber = nh.subscribe(depthCamera.infoTopic, 20, &ChiselServer::DepthCameraInfoCallback, this);
}


//    void ChiselServer::DepthCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
//    {
//        SetDepthCameraInfo(cameraInfo);
//    }

void ChiselServer::SetDepthPose(const Eigen::Affine3f& tf)
{
    pDepthCamera->lastPose = tf;
    pDepthCamera->gotPose = true;
}

//    void ChiselServer::SetColorImage(const sensor_msgs::ImageConstPtr& img)
//    {
//        if (!lastColorImage.get())
//        {
//            lastColorImage.reset(ROSImgToColorImg<ColorData>(img));
//        }
//
//        ROSImgToColorImg(img, lastColorImage.get());
//
//        colorCamera.lastImageTimestamp = img->header.stamp;
//        colorCamera.gotImage = true;
//    }

void ChiselServer::SetColorImage(const unsigned char* img, int width, int height, int step, int num_channels, boost::uint64_t timestamp)
{
    if (!lastColorImage.get())
    {
        lastColorImage.reset(ToColorImg<ColorData>(img, width, height, step, num_channels));
    }

    ToColorImg(img, width, height, step, num_channels, lastColorImage.get());

    pColorCamera->lastImageTimestamp = timestamp;
    pColorCamera->gotImage = true;
}

void ChiselServer::SetColorImageMemorySharing(unsigned char* img, int width, int height, int step, int num_channels, boost::uint64_t timestamp)
{
    if (!lastColorImage.get())
    {
        lastColorImage.reset(new chisel::ColorImage<ColorData>());
    }

    //ToColorImg(img, width, height, step, num_channels, lastColorImage.get());
    lastColorImage->SetData(img);
    lastColorImage->SetWidth(width);
    lastColorImage->SetHeight(height);
    lastColorImage->SetNumChannels(num_channels);

    pColorCamera->lastImageTimestamp = timestamp;
    pColorCamera->gotImage = true;
}

void ChiselServer::SetColorPose(const Eigen::Affine3f& tf)
{
    pColorCamera->lastPose = tf;
    pColorCamera->gotPose = true;
}

//    void ChiselServer::SetDepthImage(const sensor_msgs::ImageConstPtr& img)
//    {
//        if (!lastDepthImage.get())
//        {
//          lastDepthImage.reset(new chisel::DepthImage<DepthData>(img->width, img->height));
//        }
//
//        ROSImgToDepthImg(img, lastDepthImage.get());
//        depthCamera.lastImageTimestamp = img->header.stamp;
//        depthCamera.gotImage = true;
//    }

void ChiselServer::SetDepthImage(const float* img, int width, int height, int step, boost::uint64_t timestamp)
{
    if (!lastDepthImage.get())
    {
        lastDepthImage.reset(new chisel::DepthImage<DepthData>(width, height));
    }

    ToDepthImg(img, width, height, step, lastDepthImage.get());
    pDepthCamera->lastImageTimestamp = timestamp;
    pDepthCamera->gotImage = true;
}

void ChiselServer::SetDepthImageMemorySharing(float* img, int width, int height, int step, boost::uint64_t timestamp)
{
    if (!lastDepthImage.get())
    {
        lastDepthImage.reset(new chisel::DepthImage<DepthData>());
    }

    //ToDepthImg(img, width, height, step, lastDepthImage.get());
    lastDepthImage->SetData(img);
    lastDepthImage->SetWidth(width);
    lastDepthImage->SetHeight(height);
    pDepthCamera->lastImageTimestamp = timestamp;
    pDepthCamera->gotImage = true;
}

//    void ChiselServer::DepthImageCallback(sensor_msgs::ImageConstPtr depthImage)
//    {
//        if (IsPaused()) return;
//        SetDepthImage(depthImage);
//
//        bool gotTransform = false;
//        tf::StampedTransform tf;
//
//        int tries = 0;
//        int maxTries = 10;
//
//        while(!gotTransform && tries < maxTries)
//        {
//            tries++;
//            try
//            {
//                transformListener.waitForTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, ros::Duration(0.5));
//                transformListener.lookupTransform(depthCamera.transform, baseTransform, depthImage->header.stamp, tf);
//                depthCamera.gotPose = true;
//                gotTransform = true;
//            }
//            catch (std::exception& e)
//            {
//                ros::Rate lookupRate(0.5f);
//                ROS_WARN("%s\n", e.what());
//            }
//        }
//
//        depthCamera.lastPose = RosTfToChiselTf(tf);
//
//        hasNewData = true;
//    }

//    void ChiselServer::SetColorCameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
//    {
//        colorCamera.cameraModel = RosCameraToChiselCamera(info);
//        colorCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
//        colorCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
//        colorCamera.gotInfo = true;
//    }

void ChiselServer::SetColorCameraInfo(const double fx, const double fy, const double cx, const double cy, const int width, const int height)
{
    pColorCamera->cameraModel = ToChiselCamera(fx,fy,cx,cy,width,height);
    pColorCamera->cameraModel.SetNearPlane(GetNearPlaneDist());
    pColorCamera->cameraModel.SetFarPlane(GetFarPlaneDist());
    pColorCamera->gotInfo = true;
}

//    void ChiselServer::SetDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
//    {
//        depthCamera.cameraModel = RosCameraToChiselCamera(info);
//        depthCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
//        depthCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
//        depthCamera.gotInfo = true;
//    }

void ChiselServer::SetDepthCameraInfo(const double fx, const double fy, const double cx, const double cy, const int width, const int height)
{
    pDepthCamera->cameraModel = ToChiselCamera(fx,fy, cx, cy, width, height);
    pDepthCamera->cameraModel.SetNearPlane(GetNearPlaneDist());
    pDepthCamera->cameraModel.SetFarPlane(GetFarPlaneDist());
    pDepthCamera->gotInfo = true;
}


void ChiselServer::SubscribeColorImage(const std::string& imageTopic, const std::string& infoTopic, const std::string& transform)
{
    pColorCamera->imageTopic = imageTopic;
    pColorCamera->transform = transform;
    pColorCamera->infoTopic = infoTopic;
    //        colorCamera.imageSubscriber = nh.subscribe(colorCamera.imageTopic, 20, &ChiselServer::ColorImageCallback, this);
    //        colorCamera.infoSubscriber = nh.subscribe(colorCamera.infoTopic, 20, &ChiselServer::ColorCameraInfoCallback, this);
}

//    void ChiselServer::ColorCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
//    {
//        if (IsPaused()) return;
//        SetColorCameraInfo(cameraInfo);
//    }
//
//    void ChiselServer::ColorImageCallback(sensor_msgs::ImageConstPtr colorImage)
//    {
//        if (IsPaused()) return;
//        SetColorImage(colorImage);
//
//        bool gotTransform = false;
//        tf::StampedTransform tf;
//
//        int tries = 0;
//        int maxTries = 10;
//
//        while(!gotTransform && tries < maxTries)
//        {
//            tries++;
//            try
//            {
//                transformListener.waitForTransform(colorCamera.transform, baseTransform, colorImage->header.stamp, ros::Duration(0.5));
//                transformListener.lookupTransform(colorCamera.transform, baseTransform, colorImage->header.stamp, tf);
//                colorCamera.gotPose = true;
//                gotTransform = true;
//            }
//            catch (std::exception& e)
//            {
//                ros::Rate lookupRate(0.5f);
//                ROS_WARN("%s\n", e.what());
//            }
//        }
//
//        colorCamera.lastPose = RosTfToChiselTf(tf);
//    }

void ChiselServer::SubscribePointCloud(const std::string& topic)
{
    pPointcloudTopic->cloudTopic = topic;
    pPointcloudTopic->gotCloud = false;
    pPointcloudTopic->gotPose = false;
    pPointcloudTopic->lastPose = chisel::Transform::Identity();
    //        pointcloudTopic.lastTimestamp = ros::Time::now();
    //        pointcloudTopic.cloudSubscriber = nh.subscribe(pointcloudTopic.cloudTopic, 20, &ChiselServer::PointCloudCallback, this);
}

//    void ChiselServer::PointCloudCallback(sensor_msgs::PointCloud2ConstPtr pointcloud)
//    {
//        ROS_INFO("Got point cloud...");
//        if (IsPaused()) return;
//        if (!lastPointCloud.get())
//        {
//            lastPointCloud.reset(new chisel::PointCloud());
//        }
//
//        if (useColor)
//        {
//            ROSPointCloudToChisel<pcl::PointXYZRGB>(pointcloud, lastPointCloud.get(), true);
//        }
//        else
//        {
//            ROSPointCloudToChisel<pcl::PointXYZ>(pointcloud, lastPointCloud.get(), false);
//        }
//        pointcloudTopic.transform = pointcloud->header.frame_id;
//        bool gotTransform = false;
//        tf::StampedTransform tf;
//
//        int tries = 0;
//        int maxTries = 10;
//
//        ROS_INFO("Waiting for transform...");
//        while(!gotTransform && tries < maxTries)
//        {
//            tries++;
//            try
//            {
//                transformListener.waitForTransform(pointcloudTopic.transform, baseTransform, pointcloud->header.stamp, ros::Duration(0.5));
//                transformListener.lookupTransform(pointcloudTopic.transform, baseTransform, pointcloud->header.stamp, tf);
//                pointcloudTopic.gotPose = true;
//                gotTransform = true;
//            }
//            catch (std::exception& e)
//            {
//                ros::Rate lookupRate(0.5f);
//                ROS_WARN("%s\n", e.what());
//            }
//        }
//
//        if (gotTransform)
//        {
//            pointcloudTopic.lastPose = RosTfToChiselTf(tf);
//            pointcloudTopic.lastTimestamp = pointcloud->header.stamp;
//            hasNewData = true;
//        }
//    }


template <class PointType>
void ChiselServer::SetPointCloud(const pcl::PointCloud<PointType>& cloud_camera, chisel::Transform& Twc)
{
    std::cout << "ChiselServer::SetPointCloud() - got point cloud - size: " << cloud_camera.size() << std::endl;
    if (IsPaused()) return;

    if (!lastPointCloud.get())
    {
        lastPointCloud.reset(new chisel::PointCloud());
    }
    
#if 0    
    // check is made outside 
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloudIn, cloudIn, indices);
#endif

    PclPointCloudToChisel(cloud_camera, lastPointCloud.get(), useColor);

    pPointcloudTopic->gotPose = true;
    pPointcloudTopic->lastPose = Twc;
    pPointcloudTopic->lastTimestamp = cloud_camera.header.stamp;
    hasNewData = true;
}



template <class PointType>
void ChiselServer::IntegrateWorldPointCloud(const pcl::PointCloud<PointType>& cloud, chisel::Transform& Twc)
{
    std::cout << "ChiselServer - integrating world point cloud" << std::endl;
    if (IsPaused()) return;

    if (!lastPointCloud.get())
    {
        lastPointCloud.reset(new chisel::PointCloud());
    }
    
#if 0    
    // check is made outside 
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloudIn, cloudIn, indices);
#endif

    // convert by taking into account also the normals 
    PclPointCloudColorNormalToChisel(cloud, lastPointCloud.get());

    pPointcloudTopic->gotPose = true;
    pPointcloudTopic->lastPose = Twc;
    pPointcloudTopic->lastTimestamp = cloud.header.stamp;
    hasNewData = true;
    
    chiselMap->IntegrateWorldPointCloudWithNormals(projectionIntegrator, *lastPointCloud, pPointcloudTopic->lastPose, farPlaneDist);
    hasNewData = false;
}

void ChiselServer::Deform(chisel::MapKfidRt& deformationMap)
{
    std::cout << "ChiselServer - deforming volumetric map" << std::endl;    
    chiselMap->Deform(deformationMap);  
}


void ChiselServer::SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist)
{
    projectionIntegrator.SetCentroids(GetChiselMap()->GetChunkManager().GetCentroids());
    projectionIntegrator.SetTruncator(chisel::TruncatorPtr(new chisel::QuadraticTruncator(truncation(0), truncation(1), truncation(2), truncation(3))));
    projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
    projectionIntegrator.SetCarvingDist(carvingDist);
    projectionIntegrator.SetCarvingEnabled(useCarving);
}

void ChiselServer::IntegrateLastDepthImage(bool updateMesh)
{
    if (!IsPaused() && pDepthCamera->gotInfo && pDepthCamera->gotPose && lastDepthImage.get())
    {
        if (useColor)
        {
            printf("ChiselServer - Integrating depth scan color\n");
            //chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel, lastColorImage, colorCamera.lastPose, colorCamera.cameraModel);
            chiselMap->IntegrateDepthScanColorWithOneCameraModelBGR<DepthData, ColorData>(projectionIntegrator, lastDepthImage, pDepthCamera->lastPose, pDepthCamera->cameraModel, lastColorImage, pColorCamera->lastPose, pColorCamera->cameraModel);
            
        }
        else
        {
            printf("ChiselServer - Integrating depth scan\n");
            chiselMap->IntegrateDepthScan<DepthData>(projectionIntegrator, lastDepthImage, pDepthCamera->lastPose, pDepthCamera->cameraModel);
        }
        printf("ChiselServer - Done with scan\n");
//        PublishLatestChunkBoxes();
        PublishDepthFrustum();

        if(updateMesh)
        {
            chiselMap->UpdateMeshes();
        }
        hasNewData = false;
    }
    else
    {
        std::cout << "ChiselServer - PROBLEM in integrating depth scan!!! ************" << std::endl;        
    }
}

void ChiselServer::IntegrateLastPointCloud(bool updateMesh)
{
    if (!IsPaused() && pPointcloudTopic->gotPose && lastPointCloud.get())
    {
        //std::cout << "ChiselServer - integrating point cloud" << std::endl;
        
//        if(colorCamera.gotInfo)
//        {
//            /// < NOTE:  at present time this is very inefficient!!!        
//            chiselMap->IntegratePointCloudWithCameraPose(projectionIntegrator, *lastPointCloud, pointcloudTopic.lastPose, colorCamera.cameraModel, farPlaneDist);
//        }
//        else
        {
            if( (pDepthCamera->gotInfo) && (lastDepthImage.get()) )
            {
                std::cout << "ChiselServer - integrating point cloud with depth" << std::endl;
                chiselMap->IntegratePointCloudWidthDepth<DepthData>(projectionIntegrator, *lastPointCloud, pPointcloudTopic->lastPose, lastDepthImage, pDepthCamera->cameraModel, farPlaneDist);
                
            }
            else
            {
                std::cout << "ChiselServer - integrating point cloud" << std::endl;
                //chiselMap->IntegratePointCloud(projectionIntegrator, *lastPointCloud, pointcloudTopic.lastPose, farPlaneDist);
                chiselMap->IntegratePointCloudWidthDepth<DepthData>(projectionIntegrator, *lastPointCloud, pPointcloudTopic->lastPose, lastDepthImage, pDepthCamera->cameraModel, farPlaneDist);
            }
        }
        
//        PublishLatestChunkBoxes();
        if(updateMesh)
        {
            chiselMap->UpdateMeshes();
        }
        hasNewData = false;
    }
    else
    {
        std::cout << "ChiselServer - PROBLEM in integrating point cloud!!! ************" << std::endl;
        std::cout << "IsPaused(): " << IsPaused() << std::endl; 
        std::cout << "pointcloudTopic.gotPose: " << pPointcloudTopic->gotPose << std::endl;
        std::cout << "lastPointCloud.get(): " << lastPointCloud.get() << std::endl;
    }
}

void ChiselServer::UpdateMesh()
{
    chiselMap->UpdateMeshes();    
}

//void ChiselServer::PublishLatestChunkBoxes()
//{
//    if (!latestChunkPublisher) return;
//    const chisel::ChunkManager& chunkManager = chiselMap->GetChunkManager();
//    visualization_msgs::Marker marker;
//    marker.header.stamp = ros::Time::now();
//    marker.header.frame_id = baseTransform;
//    marker.ns = "chunk_box";
//    marker.type = visualization_msgs::Marker::CUBE_LIST;
//    marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
//    marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
//    marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
//    marker.pose.position.x = 0;
//    marker.pose.position.y = 0;
//    marker.pose.position.z = 0;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.color.r = 0.3f;
//    marker.color.g = 0.95f;
//    marker.color.b = 0.3f;
//    marker.color.a = 0.6f;
//    const chisel::ChunkSet& latest = chiselMap->GetMeshesToUpdate();
//    for (const std::pair < chisel::ChunkID, bool>& id : latest)
//    {
//        if (chunkManager.HasChunk(id.first))
//        {
//            chisel::AABB aabb = chunkManager.GetChunk(id.first)->ComputeBoundingBox();
//            chisel::Vec3 center = aabb.GetCenter();
//            geometry_msgs::Point pt;
//            pt.x = center.x();
//            pt.y = center.y();
//            pt.z = center.z();
//            marker.points.push_back(pt);
//        }
//    }
//
//    latestChunkPublisher.publish(marker);
//}

//    void ChiselServer::PublishChunkBoxes()
//    {
////        const chisel::ChunkManager& chunkManager = chiselMap->GetChunkManager();
////        visualization_msgs::Marker marker;
////        marker.header.stamp = ros::Time::now();
////        marker.header.frame_id = baseTransform;
////        marker.ns = "chunk_box";
////        marker.type = visualization_msgs::Marker::CUBE_LIST;
////        marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
////        marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
////        marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
////        marker.pose.position.x = 0;
////        marker.pose.position.y = 0;
////        marker.pose.position.z = 0;
////        marker.pose.orientation.x = 0.0;
////        marker.pose.orientation.y = 0.0;
////        marker.pose.orientation.z = 0.0;
////        marker.pose.orientation.w = 1.0;
////        marker.color.r = 0.95f;
////        marker.color.g = 0.3f;
////        marker.color.b = 0.3f;
////        marker.color.a = 0.6f;
////        for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& pair : chunkManager.GetChunks())
////        {
////            chisel::AABB aabb = pair.second->ComputeBoundingBox();
////            chisel::Vec3 center = aabb.GetCenter();
////            geometry_msgs::Point pt;
////            pt.x = center.x();
////            pt.y = center.y();
////            pt.z = center.z();
////            marker.points.push_back(pt);
////        }
////
////        chunkBoxPublisher.publish(marker);
//    }

chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
{
    return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
}

//    void ChiselServer::FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker)
//    {
//        assert(marker != nullptr);
//        marker->header.stamp = ros::Time::now();
//        marker->header.frame_id = baseTransform;
//        marker->scale.x = 1;
//        marker->scale.y = 1;
//        marker->scale.z = 1;
//        marker->pose.orientation.x = 0;
//        marker->pose.orientation.y = 0;
//        marker->pose.orientation.z = 0;
//        marker->pose.orientation.w = 1;
//        marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
//        const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();
//
//        if(meshMap.size() == 0)
//        {
//            return;
//        }
//
//        chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
//        lightDir.normalize();
//        chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
//        lightDir.normalize();
//        const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
//        //int idx = 0;
//        for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
//        {
//            const chisel::MeshPtr& mesh = meshes.second;
//            for (size_t i = 0; i < mesh->vertices.size(); i++)
//            {
//                const chisel::Vec3& vec = mesh->vertices[i];
//                geometry_msgs::Point pt;
//                pt.x = vec[0];
//                pt.y = vec[1];
//                pt.z = vec[2];
//                marker->points.push_back(pt);
//
//                if(mesh->HasColors())
//                {
//                    const chisel::Vec3& meshCol = mesh->colors[i];
//                    std_msgs::ColorRGBA color;
//                    color.r = meshCol[0];
//                    color.g = meshCol[1];
//                    color.b = meshCol[2];
//                    color.a = 1.0;
//                    marker->colors.push_back(color);
//                }
//                else
//                {
//                  if(mesh->HasNormals())
//                  {
//                      const chisel::Vec3 normal = mesh->normals[i];
//                      std_msgs::ColorRGBA color;
//                      chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
//                      color.r = fmin(lambert[0], 1.0);
//                      color.g = fmin(lambert[1], 1.0);
//                      color.b = fmin(lambert[2], 1.0);
//                      color.a = 1.0;
//                      marker->colors.push_back(color);
//                  }
//                  else
//                  {
//                    std_msgs::ColorRGBA color;
//                    color.r = vec[0] * 0.25 + 0.5;
//                    color.g = vec[1] * 0.25 + 0.5;
//                    color.b = vec[2] * 0.25 + 0.5;
//                    color.a = 1.0;
//                    marker->colors.push_back(color);
//                  }
//                }
//                //marker->indicies.push_back(idx);
//                //idx++;
//            }
//        }
//    }


template <class PointType, typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type*>
void ChiselServer::GetPointCloud(pcl::PointCloud<PointType>& output_cloud)
{
    output_cloud.clear();

    output_cloud.header.stamp = getTimestamp();

    //    assert(marker != nullptr);
    //    marker->header.stamp = ros::Time::now();
    //    marker->header.frame_id = baseTransform;
    //    marker->scale.x = 1;
    //    marker->scale.y = 1;
    //    marker->scale.z = 1;
    //    marker->pose.orientation.x = 0;
    //    marker->pose.orientation.y = 0;
    //    marker->pose.orientation.z = 0;
    //    marker->pose.orientation.w = 1;
    //    marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

    const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

    if (meshMap.size() == 0)
    {
        return;
    }

    chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
    lightDir.normalize();
    chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
    lightDir.normalize();
    const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
    //int idx = 0;
    for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
    {
        const chisel::MeshPtr& mesh = meshes.second;
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const chisel::Vec3& vec = mesh->vertices[i];

            //geometry_msgs::Point pt;

            PointType point;

            point.x = vec[0];
            point.y = vec[1];
            point.z = vec[2];

            //marker->points.push_back(pt);

            if (mesh->HasColors())
            {
                //std::cout << "mesh has color " << std::endl; 
                const chisel::Vec3& meshCol = mesh->colors[i];
                //std_msgs::ColorRGBA color;
                point.r = meshCol[0]*255;
                point.g = meshCol[1]*255;
                point.b = meshCol[2]*255;
                //point.a = 255;
                
                //std::cout  << "rgb: " <<meshCol[0] <<", " <<meshCol[1] <<", "<<meshCol[2] << std::endl;
                //marker->colors.push_back(color);
            }
            else
            {
                if (mesh->HasNormals())
                {
                    std::cout << "mesh has normals " << std::endl; 
                    
                    const chisel::Vec3 normal = mesh->normals[i];
                    //std_msgs::ColorRGBA color;
                    chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                    point.r = fmin(lambert[0], 1.0)*255;
                    point.g = fmin(lambert[1], 1.0)*255;
                    point.b = fmin(lambert[2], 1.0)*255;
                    point.a = 255;
                    //marker->colors.push_back(color);
                }
                else
                {
                    std::cout << "mesh has nothing " << std::endl;
                    
                    //std_msgs::ColorRGBA color;
                    point.r = (vec[0] * 0.25 + 0.5)*255;
                    point.g = (vec[1] * 0.25 + 0.5)*255;
                    point.b = (vec[2] * 0.25 + 0.5)*255;
                    point.a = 255;
                    //marker->colors.push_back(color);
                }
            }

            output_cloud.push_back(point);
            //marker->indicies.push_back(idx);
            //idx++;
        }
    }
}

template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type*,
                           typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::kfid>::value>::type*>
void ChiselServer::GetPointCloud(pcl::PointCloud<PointType>& output_cloud)
{
    output_cloud.clear();  

    output_cloud.header.stamp = getTimestamp();

    //    assert(marker != nullptr);
    //    marker->header.stamp = ros::Time::now();
    //    marker->header.frame_id = baseTransform;
    //    marker->scale.x = 1;
    //    marker->scale.y = 1;
    //    marker->scale.z = 1;
    //    marker->pose.orientation.x = 0;
    //    marker->pose.orientation.y = 0;
    //    marker->pose.orientation.z = 0;
    //    marker->pose.orientation.w = 1;
    //    marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

    const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

    if (meshMap.size() == 0)
    {
        return;
    }

    chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
    lightDir.normalize();
    chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
    lightDir.normalize();
    const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
    //int idx = 0;
    for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
    {
        const chisel::MeshPtr& mesh = meshes.second;
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const chisel::Vec3& vec = mesh->vertices[i];

            //geometry_msgs::Point pt;

            PointType point;

            point.x = vec[0];
            point.y = vec[1];
            point.z = vec[2];

            //marker->points.push_back(pt);

            if (mesh->HasColors())
            {
                //std::cout << "mesh has color " << std::endl; 
                const chisel::Vec3& meshCol = mesh->colors[i];
                //std_msgs::ColorRGBA color;
                point.r = meshCol[0]*255;
                point.g = meshCol[1]*255;
                point.b = meshCol[2]*255;
                //point.a = 255;
                
                //std::cout  << "rgb: " <<meshCol[0] <<", " <<meshCol[1] <<", "<<meshCol[2] << std::endl;
                //marker->colors.push_back(color);
                
                const chisel::Vec3& normal = mesh->normals[i];
                
                point.normal_x = normal[0];
                point.normal_y = normal[1];
                point.normal_z = normal[2];
                //std::cout << "normal: " << normal << std::endl; 
            }
            else
            {
                if (mesh->HasNormals())
                {
                    std::cout << "mesh has normals " << std::endl; 
                    
                    const chisel::Vec3& normal = mesh->normals[i];
                    //std_msgs::ColorRGBA color;
                    chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                    point.r = fmin(lambert[0], 1.0)*255;
                    point.g = fmin(lambert[1], 1.0)*255;
                    point.b = fmin(lambert[2], 1.0)*255;
                    point.a = 255;
                    //marker->colors.push_back(color);
                }
                else
                {
                    std::cout << "mesh has nothing " << std::endl;
                    
                    //std_msgs::ColorRGBA color;
                    point.r = (vec[0] * 0.25 + 0.5)*255;
                    point.g = (vec[1] * 0.25 + 0.5)*255;
                    point.b = (vec[2] * 0.25 + 0.5)*255;
                    point.a = 255;
                    //marker->colors.push_back(color);
                }
            }

            output_cloud.push_back(point);
            //marker->indicies.push_back(idx);
            //idx++;
        }
    }
}


template <class PointType, typename std::enable_if<pcl::traits::has_all_fields<PointType, boost::mpl::vector<pcl::fields::kfid, pcl::fields::normal_x> >::value>::type*>
void ChiselServer::GetPointCloud(pcl::PointCloud<PointType>& output_cloud)
{
    output_cloud.clear();  

    output_cloud.header.stamp = getTimestamp();

    //    assert(marker != nullptr);
    //    marker->header.stamp = ros::Time::now();
    //    marker->header.frame_id = baseTransform;
    //    marker->scale.x = 1;
    //    marker->scale.y = 1;
    //    marker->scale.z = 1;
    //    marker->pose.orientation.x = 0;
    //    marker->pose.orientation.y = 0;
    //    marker->pose.orientation.z = 0;
    //    marker->pose.orientation.w = 1;
    //    marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

    const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

    if (meshMap.size() == 0)
    {
        return;
    }

    chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
    lightDir.normalize();
    chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
    lightDir.normalize();
    const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
    //int idx = 0;
    for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
    {
        const chisel::MeshPtr& mesh = meshes.second;
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const chisel::Vec3& vec = mesh->vertices[i];

            //geometry_msgs::Point pt;

            PointType point;

            point.x = vec[0];
            point.y = vec[1];
            point.z = vec[2];

            //marker->points.push_back(pt);

            //if (mesh->HasColors())
            {
                //std::cout << "mesh has color " << std::endl; 
                const chisel::Vec3& meshCol = mesh->colors[i];
                //std_msgs::ColorRGBA color;
                point.r = meshCol[0]*255;
                point.g = meshCol[1]*255;
                point.b = meshCol[2]*255;
                //point.a = 255;
                
                //std::cout  << "rgb: " <<meshCol[0] <<", " <<meshCol[1] <<", "<<meshCol[2] << std::endl;
                //marker->colors.push_back(color);
                
                const chisel::Vec3& normal = mesh->normals[i];
                
                point.normal_x = normal[0];
                point.normal_y = normal[1];
                point.normal_z = normal[2];
                //std::cout << "normal: " << normal << std::endl; 
                
                point.kfid = mesh->kfids[i];
                //std::cout << "kfid: " << point.kfid << std::endl; 
            }
            /*else
            {
                if (mesh->HasNormals())
                {
                    std::cout << "mesh has normals " << std::endl; 
                    
                    const chisel::Vec3& normal = mesh->normals[i];
                    //std_msgs::ColorRGBA color;
                    chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                    point.r = fmin(lambert[0], 1.0)*255;
                    point.g = fmin(lambert[1], 1.0)*255;
                    point.b = fmin(lambert[2], 1.0)*255;
                    point.a = 255;
                    //marker->colors.push_back(color);
                }
                else
                {
                    std::cout << "mesh has nothing " << std::endl;
                    
                    //std_msgs::ColorRGBA color;
                    point.r = (vec[0] * 0.25 + 0.5)*255;
                    point.g = (vec[1] * 0.25 + 0.5)*255;
                    point.b = (vec[2] * 0.25 + 0.5)*255;
                    point.a = 255;
                    //marker->colors.push_back(color);
                }
                
                point.kfid = mesh->kfids[i];                
            }*/

            output_cloud.push_back(point);
            //marker->indicies.push_back(idx);
            //idx++;
        }
    }
}


//    bool ChiselServer::SaveMesh(chisel_msgs::SaveMeshService::Request& request, chisel_msgs::SaveMeshService::Response& response)
//    {
//        bool saveSuccess = chiselMap->SaveAllMeshesToPLY(request.file_name);
//        return saveSuccess;
//    }

bool ChiselServer::SaveMesh(const std::string& filename)
{
    bool saveSuccess = chiselMap->SaveAllMeshesToPLY(filename);
    
    std::cout << "ChiselServer - saved mesh" << std::endl;
    
    return saveSuccess;
}

//    bool ChiselServer::GetAllChunks(chisel_msgs::GetAllChunksService::Request& request, chisel_msgs::GetAllChunksService::Response& response)
//    {
//        const chisel::ChunkMap& chunkmap = chiselMap->GetChunkManager().GetChunks();
//        response.chunks.chunks.resize(chunkmap.size());
//        response.chunks.header.stamp = ros::Time::now();
//        size_t i = 0;
//        for (const std::pair<chisel::ChunkID, chisel::ChunkPtr>& chunkPair : chiselMap->GetChunkManager().GetChunks())
//        {
//            chisel_msgs::ChunkMessage& msg = response.chunks.chunks.at(i);
//            FillChunkMessage(chunkPair.second, &msg);
//            i++;
//        }
//
//        return true;
//    }

template void ChiselServer::SetPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud_camera, chisel::Transform& transform);
template void ChiselServer::SetPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud_camera, chisel::Transform& transform);
template void ChiselServer::SetPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_camera, chisel::Transform& transform);
template void ChiselServer::SetPointCloud(const pcl::PointCloud<pcl::PointSurfelSegment>& cloud_camera, chisel::Transform& transform);

template void ChiselServer::GetPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>& output_cloud);
template void ChiselServer::GetPointCloud<pcl::PointXYZRGBNormal, (void*)0>(pcl::PointCloud<pcl::PointXYZRGBNormal>& output_cloud); 
template void ChiselServer::GetPointCloud<pcl::PointSurfelSegment, (void*)0>(pcl::PointCloud<pcl::PointSurfelSegment>& output_cloud); 

template void ChiselServer::IntegrateWorldPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, chisel::Transform& Twc);
template void ChiselServer::IntegrateWorldPointCloud(const pcl::PointCloud<pcl::PointSurfelSegment>& cloud, chisel::Transform& Twc);

} // namespace chisel 
