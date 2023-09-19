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

#ifndef CHISELSERVER_H_
#define CHISELSERVER_H_

#include "PointSurfelSegment.h" // this should be included before any pcl file!

#include <pcl/common/common.h>

//#include <chisel_msgs/ResetService.h>
//#include <chisel_msgs/PauseService.h>
//#include <chisel_msgs/SaveMeshService.h>
//#include <chisel_msgs/GetAllChunksService.h>

#include <memory>
#include <open_chisel/Chisel.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/pointcloud/PointCloud.h>

//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <visualization_msgs/Marker.h>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>

namespace chisel_server
{

typedef float DepthData;
typedef uint8_t ColorData;


struct ChiselServerParams
{
    
    ChiselServerParams();
        
    int chunkSizeX; 
    int chunkSizeY;
    int chunkSizeZ;
    double voxelResolution;
    double truncationDistQuad;
    double truncationDistLinear;
    double truncationDistConst;
    double truncationDistScale;
    int weight;
    bool useCarving;
    bool useColor;
    bool saveFile;
    double carvingDist;
    
    double nearPlaneDist = 0.05;
    double farPlaneDist = 5.0;
    int fusionMode;
};


class ChiselServer
{
public:

    enum class FusionMode
    {
        DepthImage=0,
        PointCloud=1
    };

    struct RosCameraTopic
    {
        RosCameraTopic():lastImageTimestamp(0),gotPose(false),gotInfo(false),gotImage(false){}

        std::string imageTopic;
        std::string infoTopic;
        std::string transform;

        chisel::PinholeCamera cameraModel;
        //                ros::Subscriber imageSubscriber;
        //                ros::Subscriber infoSubscriber;
        //                ros::Publisher lastPosePublisher;
        //                ros::Publisher frustumPublisher;

        chisel::Transform lastPose;
        //                ros::Time lastImageTimestamp;
        boost::uint64_t lastImageTimestamp;
        bool gotPose;
        bool gotInfo;
        bool gotImage;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct RosPointCloudTopic
    {
        RosPointCloudTopic():lastTimestamp(0),gotPose(false),gotCloud(false){}
        std::string cloudTopic;
        std::string transform;
        //                ros::Subscriber cloudSubscriber;

        chisel::Transform lastPose;
        //                ros::Time lastTimestamp;
        boost::uint64_t lastTimestamp;
        bool gotPose;
        bool gotCloud;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    };

    ChiselServer();
    //ChiselServer(const ros::NodeHandle& nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode);
    ChiselServer(int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode);
    
    ChiselServer(ChiselServerParams& params);

    void Reset(); 
        
    virtual ~ChiselServer();

    inline chisel::ChiselPtr GetChiselMap()
    {
        return chiselMap;
    }

    inline void SetChiselMap(const chisel::ChiselPtr value)
    {
        chiselMap = value;
    }

    inline const std::string& GetBaseTransform() const
    {
        return baseTransform;
    }

    inline const std::string& GetMeshTopic() const
    {
        return meshTopic;
    }

    void SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist);

    //            void SetupMeshPublisher(const std::string& meshTopic);
    //            void SetupChunkBoxPublisher(const std::string& boxTopic);
    //            void SetupDepthPosePublisher(const std::string& depthPoseTopic);
    //            void SetupColorPosePublisher(const std::string& colorPoseTopic);
    //            void SetupDepthFrustumPublisher(const std::string& frustumTopic);
    //            void SetupColorFrustumPublisher(const std::string& frustumTopic);

    //            void PublishMeshes();
    //            void PublishChunkBoxes();
    //            void PublishLatestChunkBoxes();
    //            void PublishDepthPose();
    //            void PublishColorPose();

    void PublishDepthFrustum();
    void PublishColorFrustum();

    void SubscribeDepthImage(const std::string& depthImageTopic, const std::string& cameraInfoTopic, const std::string& transform);
    //            void DepthCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo);
    //            void DepthImageCallback(sensor_msgs::ImageConstPtr depthImage);

    void SubscribeColorImage(const std::string& colorImageTopic, const std::string& cameraInfoTopic, const std::string& transform);
    //            void ColorCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo);
    //            void ColorImageCallback(sensor_msgs::ImageConstPtr colorImage);

    void SubscribePointCloud(const std::string& topic);
    //            void PointCloudCallback(sensor_msgs::PointCloud2ConstPtr pointcloud);

    void IntegrateLastDepthImage(bool updateMesh = true);
    void IntegrateLastPointCloud(bool updateMesh = true);
    //            void FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker);
 
    void UpdateMesh();
    
    template <class PointType>
    void SetPointCloud(const pcl::PointCloud<PointType>& cloud_camera, chisel::Transform& Twc);
    
    template <class PointType>
    void IntegrateWorldPointCloud(const pcl::PointCloud<PointType>& cloud, chisel::Transform& Twc);
    
    void Deform(chisel::MapKfidRt& deformationMap);  
    
    template <class PointType, typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type* = nullptr>
    void GetPointCloud(pcl::PointCloud<PointType>& output_cloud);
    
    template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type* = nullptr,
                               typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::kfid>::value>::type* = nullptr>
    void GetPointCloud(pcl::PointCloud<PointType>& output_cloud);
    
    template <class PointType, typename std::enable_if<pcl::traits::has_all_fields<PointType, boost::mpl::vector<pcl::fields::kfid, pcl::fields::normal_x> >::value>::type* = nullptr>
    void GetPointCloud(pcl::PointCloud<PointType>& output_cloud);    
    
    inline void SetBaseTransform(const std::string& frameName)
    {
        baseTransform = frameName;
    }

    inline bool HasNewData()
    {
        return hasNewData;
    }

    inline float GetNearPlaneDist() const
    {
        return nearPlaneDist;
    }

    inline float GetFarPlaneDist() const
    {
        return farPlaneDist;
    }

    inline void SetNearPlaneDist(float dist)
    {
        nearPlaneDist = dist;
    }

    inline void SetFarPlaneDist(float dist)
    {
        farPlaneDist = dist;
    }
    //
    //            bool Reset(chisel_msgs::ResetService::Request& request, chisel_msgs::ResetService::Response& response);
    //            bool TogglePaused(chisel_msgs::PauseService::Request& request, chisel_msgs::PauseService::Response& response);
    //            bool SaveMesh(chisel_msgs::SaveMeshService::Request& request, chisel_msgs::SaveMeshService::Response& response);
    //            bool GetAllChunks(chisel_msgs::GetAllChunksService::Request& request, chisel_msgs::GetAllChunksService::Response& response);

    bool SaveMesh(const std::string& filename);

    inline bool IsPaused()
    {
        return isPaused;
    }

    inline void SetPaused(bool paused)
    {
        isPaused = paused;
    }

    void AdvertiseServices();

    inline FusionMode GetMode()
    {
        return mode;
    }

    inline void SetMode(const FusionMode& m)
    {
        mode = m;
    }

    //void SetDepthImage(const sensor_msgs::ImageConstPtr& img);
    void SetDepthImage(const float* img, int width, int height, int step, boost::uint64_t timestamp);
    void SetDepthImageMemorySharing(float* img, int width, int height, int step, boost::uint64_t timestamp);
    void SetDepthPose(const Eigen::Affine3f& tf);
    
    //void SetColorImage(const sensor_msgs::ImageConstPtr& img);
    void SetColorImage(const unsigned char* img, int width, int height, int step, int num_channels, boost::uint64_t timestamp);
    void SetColorImageMemorySharing(unsigned char* img, int width, int height, int step, int num_channels, boost::uint64_t timestamp);
    void SetColorPose(const Eigen::Affine3f& tf);
    
    //void SetColorCameraInfo(const sensor_msgs::CameraInfoConstPtr& info);
    void SetColorCameraInfo(const double fx, const double fy, const double cx, const double cy, const int width, const int height);
    //void SetDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr& info);
    void SetDepthCameraInfo(const double fx, const double fy, const double cx, const double cy, const int width, const int height);

protected:
    //            visualization_msgs::Marker CreateFrustumMarker(const chisel::Frustum& frustum);

    //            ros::NodeHandle nh;
    
    chisel::ChiselPtr chiselMap;
    
    //            tf::TransformListener transformListener;
    std::shared_ptr<chisel::DepthImage<DepthData> > lastDepthImage;
    std::shared_ptr<chisel::ColorImage<ColorData> > lastColorImage;
    chisel::PointCloudPtr lastPointCloud;
    chisel::ProjectionIntegrator projectionIntegrator;
    std::string baseTransform;
    std::string meshTopic;
    std::string chunkBoxTopic;
    
    //            ros::Publisher meshPublisher;
    //            ros::Publisher chunkBoxPublisher;
    //            ros::Publisher latestChunkPublisher;
    //            ros::ServiceServer resetServer;
    //            ros::ServiceServer pauseServer;  
    //            ros::ServiceServer saveMeshServer;
    //            ros::ServiceServer getAllChunksServer;
    std::shared_ptr<RosCameraTopic> pDepthCamera;
    std::shared_ptr<RosCameraTopic> pColorCamera;
    std::shared_ptr<RosPointCloudTopic> pPointcloudTopic;
    
    bool useColor;
    bool hasNewData;
    float nearPlaneDist;
    float farPlaneDist;
    bool isPaused;
    FusionMode mode;
    
};
typedef std::shared_ptr<ChiselServer> ChiselServerPtr;
typedef std::shared_ptr<const ChiselServer> ChiselServerConstPtr;


} // namespace chisel 

#endif // CHISELSERVER_H_ 
