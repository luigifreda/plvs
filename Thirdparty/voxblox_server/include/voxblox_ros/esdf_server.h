#ifndef VOXBLOX_ROS_ESDF_SERVER_H_
#define VOXBLOX_ROS_ESDF_SERVER_H_

#include <memory>

#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>

#ifndef COMPILE_WITHOUT_ROS
#include <voxblox_msgs/Layer.h>
#endif

#include "voxblox_ros/tsdf_server.h"

namespace voxblox {

class EsdfServer : public TsdfServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EsdfServer(){}
  
#ifndef COMPILE_WITHOUT_ROS  
  EsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  EsdfServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
             const EsdfMap::Config& esdf_config,
             const EsdfIntegrator::Config& esdf_integrator_config,
             const TsdfMap::Config& tsdf_config,
             const TsdfIntegratorBase::Config& tsdf_integrator_config);
#endif
  
  virtual ~EsdfServer() {}

  void publishAllUpdatedEsdfVoxels();
  virtual void publishSlices();

#ifndef COMPILE_WITHOUT_ROS    
  bool generateEsdfCallback(std_srvs::Empty::Request& request,     // NOLINT
                            std_srvs::Empty::Response& response);  // NOLINT
#endif
  
  virtual void updateMesh();
  virtual void publishPointclouds();
  virtual void newPoseCallback(const Transformation& T_G_C);
  virtual void publishMap(const bool reset_remote_map = false);
  virtual bool saveMap(const std::string& file_path);
  virtual bool loadMap(const std::string& file_path);

  // Call updateMesh if you want everything updated; call this specifically
  // if you don't want the mesh or visualization.
  void updateEsdf();
  // Update the ESDF all at once; clear the existing map.
  void updateEsdfBatch(bool full_euclidean = false);

#ifndef COMPILE_WITHOUT_ROS    
  // Overwrites the layer with what's coming from the topic!
  void esdfMapCallback(const voxblox_msgs::Layer& layer_msg);
#endif

  inline std::shared_ptr<EsdfMap> getEsdfMapPtr() { return esdf_map_; }
  inline std::shared_ptr<const EsdfMap> getEsdfMapPtr() const {
    return esdf_map_;
  }

  bool getClearSphere() const { return clear_sphere_for_planning_; }
  void setClearSphere(bool clear_sphere_for_planning) {
    clear_sphere_for_planning_ = clear_sphere_for_planning;
  }
  float getEsdfMaxDistance() const;
  void setEsdfMaxDistance(float max_distance);

  virtual void clear();

 protected:
     
#ifndef COMPILE_WITHOUT_ROS       
  // Publish markers for visualization.
  ros::Publisher esdf_pointcloud_pub_;
  ros::Publisher esdf_slice_pub_;

  // Publish the complete map for other nodes to consume.
  ros::Publisher esdf_map_pub_;

  // Subscriber to subscribe to another node generating the map.
  ros::Subscriber esdf_map_sub_;

  // Services.
  ros::ServiceServer generate_esdf_srv_;
#endif

  bool clear_sphere_for_planning_;
  bool publish_esdf_map_;

  // ESDF maps.
  std::shared_ptr<EsdfMap> esdf_map_;
  std::unique_ptr<EsdfIntegrator> esdf_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_ESDF_SERVER_H_
