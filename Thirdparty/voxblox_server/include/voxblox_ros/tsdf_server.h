#ifndef VOXBLOX_ROS_TSDF_SERVER_H_
#define VOXBLOX_ROS_TSDF_SERVER_H_

#include "PointSurfelSegment.h" // this should be included before pcl files!

//#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#ifndef COMPILE_WITHOUT_ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#endif
#include <memory>
#include <string>

#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#ifndef COMPILE_WITHOUT_ROS
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/FilePath.h>
#endif
#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"

namespace voxblox
{

class TsdfServer
{

public:

    static const std::string kColorModeDefault;
    static const std::string kIntegratorMethodDefault;
    
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TsdfServer();
    TsdfServer(const TsdfMap::Config& config, 
               const TsdfIntegratorBase::Config& integrator_config, 
               const std::string& integration_method = kIntegratorMethodDefault);
    
#ifndef COMPILE_WITHOUT_ROS  
    TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    TsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
               const TsdfMap::Config& config,
               const TsdfIntegratorBase::Config& integrator_config);
#endif

    virtual ~TsdfServer(){}

#ifndef COMPILE_WITHOUT_ROS  
    void getServerConfigFromRosParam(const ros::NodeHandle& nh_private);

    void insertPointcloud(const sensor_msgs::PointCloud2::Ptr& pointcloud);

    void insertFreespacePointcloud(const sensor_msgs::PointCloud2::Ptr& pointcloud);

    virtual void processPointCloudMessageAndInsert(const sensor_msgs::PointCloud2::Ptr& pointcloud_msg,
                                                   const bool is_freespace_pointcloud);
#endif

    template <class PointType>
    void insertPointCloud(const pcl::PointCloud<PointType>& cloud_camera, IsometryTransform& Twc, const bool is_freespace_pointcloud = false); 
    
    template <class PointType>
    void insertWorldPointCloud(const pcl::PointCloud<PointType>& cloud, IsometryTransform& Twc, const bool is_freespace_pointcloud = false);     
    
    void integratePointcloud(const Transformation& T_G_C,
                             const Pointcloud& ptcloud_C, const Colors& colors,
                             const bool is_freespace_pointcloud = false);        
    
    template<typename PointT>    
    void getMeshAsPointcloud(pcl::PointCloud<PointT>& pointcloud)
    {
        fillPointcloudWithMesh(mesh_layer_, ColorMode::kColor, pointcloud);
    }
    

    virtual void newPoseCallback(const Transformation& /*new_pose*/)
    {
        // Do nothing.
    }

    void publishAllUpdatedTsdfVoxels();
    void publishTsdfSurfacePoints();
    void publishTsdfOccupiedNodes();

    virtual void publishSlices();
    virtual void updateMesh(); // Incremental update.
    virtual bool generateMesh(); // Batch update.
    virtual void publishPointclouds(); // Publishes all available pointclouds.
    virtual void publishMap(
                            const bool reset_remote_map = false); // Publishes the complete map
    virtual bool saveMap(const std::string& file_path);
    virtual bool loadMap(const std::string& file_path);

#ifndef COMPILE_WITHOUT_ROS    
    bool clearMapCallback(std_srvs::Empty::Request& request, // NOLINT
                          std_srvs::Empty::Response& response); // NOLINT
    bool saveMapCallback(voxblox_msgs::FilePath::Request& request, // NOLINT
                         voxblox_msgs::FilePath::Response& response); // NOLINT
    bool loadMapCallback(voxblox_msgs::FilePath::Request& request, // NOLINT
                         voxblox_msgs::FilePath::Response& response); // NOLINT
    bool generateMeshCallback(std_srvs::Empty::Request& request, // NOLINT
                              std_srvs::Empty::Response& response); // NOLINT
    bool publishPointcloudsCallback(
                                    std_srvs::Empty::Request& request, // NOLINT
                                    std_srvs::Empty::Response& response); // NOLINT
    bool publishTsdfMapCallback(std_srvs::Empty::Request& request, // NOLINT
                                std_srvs::Empty::Response& response); // NOLINT

    void updateMeshEvent(const ros::TimerEvent& event);
#endif

    std::shared_ptr<TsdfMap> getTsdfMapPtr()
    {
        return tsdf_map_;
    }

    // Accessors for setting and getting parameters.

    double getSliceLevel() const
    {
        return slice_level_;
    }

    void setSliceLevel(double slice_level)
    {
        slice_level_ = slice_level;
    }

    // CLEARS THE ENTIRE MAP!
    virtual void clear();

 
    // Overwrites the layer with what's coming from the topic!
    void tsdfMapCallback(const voxblox_msgs::Layer& layer_msg);

    
    const TsdfIntegratorBase::Config& getIntegratorConfig() const { return tsdf_integrator_->getConfig(); }    

protected:
#ifndef COMPILE_WITHOUT_ROS      
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
#endif

    bool verbose_;

    // Global/map coordinate frame. Will always look up TF transforms to this
    // frame.
    std::string world_frame_;

    // Pointcloud visualization settings.
    double slice_level_;

    // If the system should subscribe to a pointcloud giving points in freespace
    bool use_freespace_pointcloud_;

    // Mesh output settings. Mesh is only written to file if mesh_filename_ is
    // not empty.
    std::string mesh_filename_;
    // How to color the mesh.
    ColorMode color_mode_;

#ifndef COMPILE_WITHOUT_ROS   
    // Will throttle to this message rate.
    ros::Duration min_time_between_msgs_;
#endif

    // What output information to publish
    bool publish_tsdf_info_;
    bool publish_slices_;
    bool publish_tsdf_map_;
    
    // Subscriber settings.
    int pointcloud_queue_size_;    

#ifndef COMPILE_WITHOUT_ROS   
    // Data subscribers.
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber freespace_pointcloud_sub_;

    // Publish markers for visualization.
    ros::Publisher mesh_pub_;
    ros::Publisher tsdf_pointcloud_pub_;
    ros::Publisher surface_pointcloud_pub_;
    ros::Publisher tsdf_slice_pub_;
    ros::Publisher occupancy_marker_pub_;

    // Publish the complete map for other nodes to consume.
    ros::Publisher tsdf_map_pub_;

    // Subscriber to subscribe to another node generating the map.
    ros::Subscriber tsdf_map_sub_;

    // Services.
    ros::ServiceServer generate_mesh_srv_;
    ros::ServiceServer clear_map_srv_;
    ros::ServiceServer save_map_srv_;
    ros::ServiceServer load_map_srv_;
    ros::ServiceServer publish_pointclouds_srv_;
    ros::ServiceServer publish_tsdf_map_srv_;

    // Timers.
    ros::Timer update_mesh_timer_;
#endif

    // Maps and integrators.
    std::shared_ptr<TsdfMap> tsdf_map_;
    std::unique_ptr<TsdfIntegratorBase> tsdf_integrator_;

    // Mesh accessories.
    std::shared_ptr<MeshLayer> mesh_layer_;
    std::unique_ptr<MeshIntegrator<TsdfVoxel> > mesh_integrator_;

    // Transformer object to keep track of either TF transforms or messages from
    // a transform topic.
    Transformer transformer_;
};

} // namespace voxblox

#endif  // VOXBLOX_ROS_TSDF_SERVER_H_
