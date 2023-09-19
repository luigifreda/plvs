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
//#include <ros/ros.h>
//#include <ros/node_handle.h>
#include <chisel_server/ChiselServer.h>  

int main(int argc, char** argv)
{
//    ROS_INFO("Starting up chisel node.");
//    ros::init(argc, argv, "Chisel");
//    ros::NodeHandle nh("~");
    int chunkSizeX = 8; 
    int chunkSizeY = 8;
    int chunkSizeZ = 8;
    double voxelResolution = 0.03;
    double truncationDistQuad = 0.0019;
    double truncationDistLinear = 0.00152;
    double truncationDistConst = 0.001504;
    double truncationDistScale = 10.0;
    int weight = 1;
    bool useCarving = true;
    bool useColor = true;
    bool saveFile = true;
    double carvingDist = 0.05;
    std::string depthImageTopic;
    std::string depthImageInfoTopic;
    std::string depthImageTransform;
    std::string colorImageTopic;
    std::string colorImageInfoTopic;
    std::string colorImageTransform;
    std::string baseTransform;
    std::string meshTopic;
    std::string chunkBoxTopic;
    std::string fileToSave;
    double nearPlaneDist = 0.05;
    double farPlaneDist = 5.0;
    chisel_server::ChiselServer::FusionMode mode;
    std::string modeString = "PointCloud";
    std::string pointCloudTopic;

//    nh.param("chunk_size_x", chunkSizeX, 32);
//    nh.param("chunk_size_y", chunkSizeY, 32);
//    nh.param("chunk_size_z", chunkSizeZ, 32);
//    nh.param("truncation_constant", truncationDistConst, 0.001504);
//    nh.param("truncation_linear", truncationDistLinear, 0.00152);
//    nh.param("truncation_quadratic", truncationDistQuad, 0.0019);
//    nh.param("truncation_scale", truncationDistScale, 8.0);
//    nh.param("integration_weight", weight, 1);
//    nh.param("use_voxel_carving", useCarving, true);
//    nh.param("use_color", useColor, true);
//    nh.param("save_file_on_exit", saveFile, true);
//    nh.param("carving_dist_m", carvingDist, 0.05);
//    nh.param("voxel_resolution_m", voxelResolution, 0.03);
//    nh.param("near_plane_dist", nearPlaneDist, 0.05);
//    nh.param("far_plane_dist", farPlaneDist, 5.0);
//    nh.param("depth_image_topic", depthImageTopic, std::string("/depth_image"));
//    nh.param("point_cloud_topic", pointCloudTopic, std::string("/camera/depth_registered/points"));
//    nh.param("depth_image_info_topic", depthImageInfoTopic, std::string("/depth_camera_info"));
//    nh.param("depth_image_transform", depthImageTransform, std::string("/camera_depth_optical_frame"));
//    nh.param("color_image_topic", colorImageTopic, std::string("/color_image"));
//    nh.param("color_image_info_topic", colorImageInfoTopic, std::string("/color_camera_info"));
//    nh.param("color_image_transform", colorImageTransform, std::string("/camera_rgb_optical_frame"));
//    nh.param("base_transform", baseTransform, std::string("/camera_link"));
//    nh.param("mesh_topic", meshTopic, std::string("full_mesh"));
//    nh.param("chunk_box_topic", chunkBoxTopic, std::string("chunk_boxes"));
    
//    nh.param("fusion_mode", modeString, std::string("DepthImage"));
//    nh.param("file_path", fileToSave, std::string("/home/mklingen/.ros/chisel.ply"));

    if(modeString == "DepthImage")
    {
//        ROS_INFO("Mode depth image");
        mode = chisel_server::ChiselServer::FusionMode::DepthImage;
    }
    else if(modeString == "PointCloud")
    {
//        ROS_INFO("Mode point cloud");
        mode = chisel_server::ChiselServer::FusionMode::PointCloud;
    }
    else
    {
//        ROS_ERROR("Unrecognized fusion mode %s. Recognized modes: \"DepthImage\", \"PointCloud\"\n", modeString.c_str());
        return -1;
    }

//    ROS_INFO("Subscribing.");
    chisel::Vec4 truncation(truncationDistQuad, truncationDistLinear, truncationDistConst, truncationDistScale);

    chisel_server::ChiselServerPtr server(new chisel_server::ChiselServer(chunkSizeX, chunkSizeY, chunkSizeZ, voxelResolution, useColor, mode));
    server->SetupProjectionIntegrator(truncation, static_cast<uint16_t>(weight), useCarving, carvingDist);

    if (mode == chisel_server::ChiselServer::FusionMode::DepthImage)
    {
        server->SubscribeDepthImage(depthImageTopic, depthImageInfoTopic, depthImageTransform);
    }
    else
    {
        server->SubscribePointCloud(pointCloudTopic);
    }

//    server->SetupDepthPosePublisher("last_depth_pose");
//    server->SetupDepthFrustumPublisher("last_depth_frustum");

    server->SetNearPlaneDist(nearPlaneDist);
    server->SetFarPlaneDist(farPlaneDist);
    server->AdvertiseServices();

    if (useColor && mode == chisel_server::ChiselServer::FusionMode::DepthImage)
    {
        server->SubscribeColorImage(colorImageTopic, colorImageInfoTopic, colorImageTransform);
//        server->SetupColorPosePublisher("last_color_pose");
//        server->SetupColorFrustumPublisher("last_color_frustum");
    }

    server->SetBaseTransform(baseTransform);
//    server->SetupMeshPublisher(meshTopic);
//    server->SetupChunkBoxPublisher(chunkBoxTopic);
//    ROS_INFO("Beginning to loop.");

//    ros::Rate loop_rate(100);

    //while (ros::ok())
    while(1)
    {
//        loop_rate.sleep();
//        ros::spinOnce();

        if(!server->IsPaused() && server->HasNewData())
        {
            switch (server->GetMode())
            {
                case chisel_server::ChiselServer::FusionMode::DepthImage:
                    server->IntegrateLastDepthImage();
                    break;
                case chisel_server::ChiselServer::FusionMode::PointCloud:
                    server->IntegrateLastPointCloud();
                    break;
            }

//            server->PublishMeshes();
//            server->PublishChunkBoxes();

            if(mode == chisel_server::ChiselServer::FusionMode::DepthImage)
            {
//                server->PublishDepthPose();
                server->PublishDepthFrustum();

                if(useColor)
                {
//                    server->PublishColorPose();
                    server->PublishColorFrustum();
                }
            }
        }
    }

    if (saveFile)
    {
        //ROS_INFO("Saving to %s\n", fileToSave.c_str());
        server->GetChiselMap()->SaveAllMeshesToPLY(fileToSave);
    }

}





