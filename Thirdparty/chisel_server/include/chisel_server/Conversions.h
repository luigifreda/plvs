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


#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include "PointSurfelSegment.h" // this should be included before any pcl file!

#include <pcl/common/common.h>
#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <Eigen/Geometry>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/DepthImage.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <geometry_msgs/Transform.h>
//#include <sensor_msgs/PointCloud2.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include <pcl/filters/filter.h>
//#include <sensor_msgs/point_cloud_conversion.h>

namespace chisel_server
{

inline void PclPointCloudToChisel(const pcl::PointCloud<pcl::PointXYZ>& cloudIn, chisel::PointCloud* cloudOut)
{
    assert(!!cloudOut);
    cloudOut->GetMutablePoints().resize(cloudIn.points.size());
    cloudOut->GetMutableKfids().resize(cloudIn.points.size());   

    size_t i = 0;
    for (const pcl::PointXYZ& pt : cloudIn.points)
    {
        chisel::Vec3& xyz = cloudOut->GetMutablePoints().at(i);
        xyz(0) = pt.x;
        xyz(1) = pt.y;
        xyz(2) = pt.z;
        i++;
    }
}

inline void SetPointsAndColors(const pcl::PointCloud<pcl::PointXYZ>& cloudIn, chisel::PointCloud* cloudOut)
{
    // Empty because of template shenanigans
    std::cout << "Chisel::SetPointsAndColors() - empty " << std::endl;    
}

template <class PointType, typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::kfid>::value>::type* = nullptr>
inline void SetPointsAndColors(const pcl::PointCloud<PointType>& cloudIn, chisel::PointCloud* cloudOut)
{
    std::cout << "Chisel::SetPointsAndColors() - using colors " << std::endl;
        
    assert(!!cloudOut);
    cloudOut->GetMutableColors().resize(cloudIn.points.size());

    size_t i = 0;
    const float byteToFloat = 1.0f / 255.0f;
    chisel::Vec3List& cloudOutColors = cloudOut->GetMutableColors();
    chisel::Vec3List& cloudOutPoints = cloudOut->GetMutablePoints();     
    for (const PointType& pt : cloudIn.points)
    {
        chisel::Vec3& xyz = cloudOutPoints[i];
        xyz(0) = pt.x;
        xyz(1) = pt.y;
        xyz(2) = pt.z;
            
        chisel::Vec3& rgb = cloudOutColors[i];
        rgb(0) = pt.r * byteToFloat;
        rgb(1) = pt.g * byteToFloat;
        rgb(2) = pt.b * byteToFloat;
        
        i++;
    }        
}

template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::kfid>::value>::type* = nullptr>
inline void SetPointsAndColors(const pcl::PointCloud<PointType>& cloudIn, chisel::PointCloud* cloudOut)
{
    std::cout << "Chisel::SetPointsAndColors() - using colors (kfid) " << std::endl;
        
    assert(!!cloudOut);
    cloudOut->GetMutableColors().resize(cloudIn.points.size());
    cloudOut->GetMutableKfids().resize(cloudIn.points.size());   
    
    size_t i = 0;
    const float byteToFloat = 1.0f / 255.0f;
    chisel::Vec3List& cloudOutColors = cloudOut->GetMutableColors();
    chisel::Vec3List& cloudOutPoints = cloudOut->GetMutablePoints();    
    chisel::KfidList& cloudKfids = cloudOut->GetMutableKfids();        
    for (const PointType& pt : cloudIn.points)
    {
        chisel::Vec3& xyz = cloudOutPoints[i];
        xyz(0) = pt.x;
        xyz(1) = pt.y;
        xyz(2) = pt.z;
            
        chisel::Vec3& rgb = cloudOutColors[i];
        rgb(0) = pt.r * byteToFloat;
        rgb(1) = pt.g * byteToFloat;
        rgb(2) = pt.b * byteToFloat;
        
        cloudKfids[i] = pt.kfid;
        
        i++;
    }        
}

template <class PointType, typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::kfid>::value>::type* = nullptr> 
inline void SetKfid(const PointType& pt, const size_t i, chisel::KfidList& cloudKfids )
{}

template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::kfid>::value>::type* = nullptr> 
inline void SetKfid(const PointType& pt, const size_t i, chisel::KfidList& cloudKfids )
{
    cloudKfids[i] = pt.kfid; 
}

template <class PointType, typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type* = nullptr> 
inline void SetPointsColorsNormals(const pcl::PointCloud<PointType>& cloudIn, chisel::PointCloud* cloudOut)
{
    SetPointsAndColors(cloudIn, cloudOut);
}

template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type* = nullptr> 
inline void SetPointsColorsNormals(const pcl::PointCloud<PointType>& cloudIn, chisel::PointCloud* cloudOut)
{
    std::cout << "Chisel - using colors and normals " << std::endl;
        
    assert(!!cloudOut);
    cloudOut->GetMutableColors().resize(cloudIn.points.size());
    cloudOut->GetMutableNormals().resize(cloudIn.points.size());    
    
    bool bKfidAvailable = pcl::traits::has_field<PointType, pcl::fields::kfid>::value;
    if(bKfidAvailable)  cloudOut->GetMutableKfids().resize(cloudIn.points.size());       

    size_t i = 0;
    const float byteToFloat = 1.0f / 255.0f;
    chisel::Vec3List& cloudOutColors = cloudOut->GetMutableColors();
    chisel::Vec3List& cloudOutPoints = cloudOut->GetMutablePoints();   
    chisel::Vec3List& cloudOutNormals = cloudOut->GetMutableNormals();      
    chisel::KfidList& cloudKfids = cloudOut->GetMutableKfids(); 
    
    for (const PointType& pt : cloudIn.points)
    {
        chisel::Vec3& xyz = cloudOutPoints[i];
        xyz(0) = pt.x;
        xyz(1) = pt.y;
        xyz(2) = pt.z;
        
        chisel::Vec3& normal = cloudOutNormals[i];
        normal(0) = pt.normal_x;
        normal(1) = pt.normal_y;
        normal(2) = pt.normal_z;        
            
        chisel::Vec3& rgb = cloudOutColors[i];
        rgb(0) = pt.r * byteToFloat;
        rgb(1) = pt.g * byteToFloat;
        rgb(2) = pt.b * byteToFloat;
        
        SetKfid(pt, i, cloudKfids );
        
        i++;
    }
        
}


template <class PointType>
inline void PclPointCloudToChisel(const pcl::PointCloud<PointType>& cloudIn, chisel::PointCloud* cloudOut, bool bUseColor = true)
{
    assert(!!cloudOut);
    cloudOut->GetMutablePoints().resize(cloudIn.points.size());
    
    bool bKfidAvailable = pcl::traits::has_field<PointType, pcl::fields::kfid>::value;
    if(bKfidAvailable)  cloudOut->GetMutableKfids().resize(cloudIn.points.size());                

    if (bUseColor)
    {     
        SetPointsAndColors(cloudIn, cloudOut);
    }
    else
    {
        size_t i = 0;
        chisel::Vec3List& cloudOutPoints = cloudOut->GetMutablePoints(); 
        chisel::KfidList& cloudKfids = cloudOut->GetMutableKfids();         
        
        for (const PointType& pt : cloudIn.points)
        {
            //chisel::Vec3& xyz = cloudOut->GetMutablePoints().at(i);
            chisel::Vec3& xyz = cloudOutPoints[i];
            xyz(0) = pt.x;
            xyz(1) = pt.y;
            xyz(2) = pt.z;
            
            SetKfid(pt, i, cloudKfids );
        
            i++;
        }        
    }
}

template <class PointType>
inline void PclPointCloudColorNormalToChisel(const pcl::PointCloud<PointType>& cloudIn, chisel::PointCloud* cloudOut)
{
    assert(!!cloudOut);
    cloudOut->GetMutablePoints().resize(cloudIn.points.size());
    cloudOut->GetMutableKfids().resize(cloudIn.points.size());  // do it in any case (the kfid is used in the world point cloud integration)           

    SetPointsColorsNormals(cloudIn, cloudOut);
}

//    template <class PointType> void ROSPointCloudToChisel(sensor_msgs::PointCloud2ConstPtr cloudIn, chisel::PointCloud* cloudOut, bool useColor=true)
//    {
//        assert(!!cloudOut);
//        pcl::PointCloud<PointType> pclCloud;
//        pcl::fromROSMsg(*cloudIn, pclCloud);
//
//        //remove NAN points from the cloud
//        std::vector<int> indices;
//        pcl::removeNaNFromPointCloud(pclCloud, pclCloud, indices);
//        PclPointCloudToChisel(pclCloud, cloudOut, useColor);
//    }


//    template <class DataType> void ROSImgToDepthImg(sensor_msgs::ImageConstPtr image, chisel::DepthImage<DataType>* depthImage)
//    {
//            ROS_INFO("Got depth image of format %s", image->encoding.c_str());
//            bool mmImage = false;
//            
//            if (image->encoding == "16UC1")
//            {
//                mmImage = true;
//            }
//            else if (image->encoding == "32FC1")
//            {
//                mmImage = false;
//            }
//            else
//            {
//                ROS_ERROR("Unrecognized depth image format.");
//                return;
//            }
//            
//            if (!mmImage)
//            {
//                size_t dataSize =image->step / image->width;
//                assert(depthImage->GetHeight() == static_cast<int>(image->height) && depthImage->GetWidth() == static_cast<int>(image->width));
//                assert(dataSize == sizeof(DataType));
//                const DataType* imageData = reinterpret_cast<const DataType*>(image->data.data());
//                DataType* depthImageData = depthImage->GetMutableData();
//                int totalPixels = image->width * image->height;
//                for (int i = 0; i < totalPixels; i++)
//                {
//                    depthImageData[i] =  imageData[i];
//                }
//            }
//            else
//            {
//                assert(depthImage->GetHeight() == static_cast<int>(image->height) && depthImage->GetWidth() == static_cast<int>(image->width));
//                const uint16_t* imageData = reinterpret_cast<const uint16_t*>(image->data.data());
//                DataType* depthImageData = depthImage->GetMutableData();
//                int totalPixels = image->width * image->height;
//                for (int i = 0; i < totalPixels; i++)
//                {
//                    depthImageData[i] =  (1.0f / 1000.0f) * imageData[i];
//                }
//            }
//    }

template <class DataType> void ToDepthImg(const float* imgData, int width, int height, int step, chisel::DepthImage<DataType>* depthImage)
{
    
    if (step != width * sizeof (DataType))
    {
        printf("\nERROR: Inconsistent step : %lu. Expected %lu\n", step, width * sizeof (DataType));
        return;
    }
        
//    ROS_INFO("Got depth image of format %s", image->encoding.c_str());
//    bool mmImage = false;
//
//    if (image->encoding == "16UC1")
//    {
//        mmImage = true;
//    }
//    else if (image->encoding == "32FC1")
//    {
//        mmImage = false;
//    }
//    else
//    {
//        ROS_ERROR("Unrecognized depth image format.");
//        return;
//    }

//  if (!mmImage)
//    {
//        size_t dataSize = step / width;
//        //assert(depthImage->GetHeight() == static_cast<int> (height) && depthImage->GetWidth() == static_cast<int> (width));
//        assert(dataSize == sizeof (DataType));
//        const DataType* imageData = reinterpret_cast<const DataType*> (image->data.data());
//        DataType* depthImageData = depthImage->GetMutableData();
//        int totalPixels = image->width * image->height;
//        for (int i = 0; i < totalPixels; i++)
//        {
//            depthImageData[i] = imageData[i];
//        }
//    }
//    else
    {
        //assert(depthImage->GetHeight() == static_cast<int> (image->height) && depthImage->GetWidth() == static_cast<int> (image->width));
        //const uint16_t* imageData = reinterpret_cast<const uint16_t*> (image->data.data());
        DataType* depthImageData = depthImage->GetMutableData();
        int totalPixels = width * height;
        /*for (int i = 0; i < totalPixels; i++)
        {
            depthImageData[i] = imgData[i];
        }*/
        std::memcpy(depthImageData, imgData, sizeof(DataType)*totalPixels);
        
    }
}


//    template <class DataType> void ROSImgToColorImg(sensor_msgs::ImageConstPtr image, chisel::ColorImage<DataType>* colorImage)
//    {
//        size_t numChannels = colorImage->GetNumChannels();
//        size_t dataSize =image->step / image->width;
//        assert(colorImage->GetHeight() == static_cast<int>(image->height) && colorImage->GetWidth() == static_cast<int>(image->width));
//
//        if (dataSize != numChannels * sizeof(DataType))
//        {
//            ROS_ERROR("Inconsistent channel width: %lu. Expected %lu\n", dataSize, numChannels * sizeof(DataType));
//            return;
//        }
//
//        const DataType* imageData = reinterpret_cast<const DataType*>(image->data.data());
//        DataType* colorImageData = colorImage->GetMutableData();
//        int totalPixels = image->width * image->height * numChannels;
//        for (int i = 0; i < totalPixels; i++)
//        {
//            colorImageData[i] =  imageData[i];
//        }
//    }

template <class DataType> void ToColorImg(const unsigned char* imgData, int width, int height, int step, int numChannels, chisel::ColorImage<DataType>* colorImage)
{
    //size_t numChannels = colorImage->GetNumChannels();
    size_t dataSize = step / width;
    assert(colorImage->GetHeight() == static_cast<int> (height) && colorImage->GetWidth() == static_cast<int> (width));

    if (dataSize != numChannels * sizeof (DataType))
    {
        printf("\nERROR: Inconsistent channel width: %lu. Expected %lu\n", dataSize, numChannels * sizeof (DataType));
        return;
    }

    const DataType* imageData = reinterpret_cast<const DataType*> (imgData);
    DataType* colorImageData = colorImage->GetMutableData();
    int totalPixels = width * height * numChannels;
    /*for (int i = 0; i < totalPixels; i++)
    {
        colorImageData[i] = imageData[i];
    }*/
    std::memcpy(colorImageData, imageData, sizeof(DataType)*totalPixels);
}

//    template <class DataType> chisel::ColorImage<DataType>* ROSImgToColorImg(sensor_msgs::ImageConstPtr image)
//    {
//        size_t numChannels = 0;
//
//        if (image->encoding == "mono8")
//        {
//            numChannels = 1;
//        }
//        else if(image->encoding == "bgr8" || image->encoding == "rgb8")
//        {
//            numChannels = 3;
//        }
//        else if(image->encoding == "bgra8")
//        {
//            numChannels = 4;
//        }
//        else
//        {
//            ROS_ERROR("Unsupported color image format %s. Supported formats are mono8, rgb8, bgr8, and bgra8\n", image->encoding.c_str());
//        }
//
//        chisel::ColorImage<DataType>* toReturn = new chisel::ColorImage<DataType>(image->width, image->height, numChannels);
//        ROSImgToColorImg(image, toReturn);
//        return toReturn;
//    }

template <class DataType> chisel::ColorImage<DataType>* ToColorImg(const unsigned char* imgData, int width, int height, int step, int numChannels)
{
//    size_t numChannels = 0;
//
//    if (image->encoding == "mono8")
//    {
//        numChannels = 1;
//    }
//    else if (image->encoding == "bgr8" || image->encoding == "rgb8")
//    {
//        numChannels = 3;
//    }
//    else if (image->encoding == "bgra8")
//    {
//        numChannels = 4;
//    }
//    else
//    {
//        ROS_ERROR("Unsupported color image format %s. Supported formats are mono8, rgb8, bgr8, and bgra8\n", image->encoding.c_str());
//    }

    chisel::ColorImage<DataType>* toReturn = new chisel::ColorImage<DataType>(width, height, numChannels);
    ToColorImg(imgData, width, height, step, numChannels, toReturn);
    return toReturn;
}


//    inline chisel::Transform RosTfToChiselTf(const tf::StampedTransform& tf)
//    {
//        chisel::Transform transform(chisel::Transform::Identity());
//        transform.translation()(0) = tf.getOrigin().x();
//        transform.translation()(1) = tf.getOrigin().y();
//        transform.translation()(2) = tf.getOrigin().z();
//
//
//        chisel::Quaternion quat(chisel::Quaternion::Identity());
//        quat.x() = tf.getRotation().x();
//        quat.y() = tf.getRotation().y();
//        quat.z() = tf.getRotation().z();
//        quat.w() = tf.getRotation().w();
//        transform.linear() = quat.toRotationMatrix();
//
//        return transform.inverse();
//    }

inline chisel::Transform ToChiselTf(const double x, const double y, const double z,
                                    const double qx = 0, const double qy = 0, const double qz = 0, const double qw = 1)
{
    chisel::Transform transform(chisel::Transform::Identity());
    transform.translation()(0) = x;
    transform.translation()(1) = y;
    transform.translation()(2) = z;


    chisel::Quaternion quat(chisel::Quaternion::Identity());
    quat.x() = qx;
    quat.y() = qy;
    quat.z() = qz;
    quat.w() = qw;
    transform.linear() = quat.toRotationMatrix();

    return transform.inverse();
}


//    inline chisel::PinholeCamera RosCameraToChiselCamera(const sensor_msgs::CameraInfoConstPtr& camera)
//    {
//        chisel::PinholeCamera cameraToReturn;
//        chisel::Intrinsics intrinsics;
//        intrinsics.SetFx(camera->P[0]);
//        intrinsics.SetFy(camera->P[5]);
//        intrinsics.SetCx(camera->P[2]);
//        intrinsics.SetCy(camera->P[6]);
//        cameraToReturn.SetIntrinsics(intrinsics);
//        cameraToReturn.SetWidth(camera->width);
//        cameraToReturn.SetHeight(camera->height);
//        return cameraToReturn;
//    }

inline chisel::PinholeCamera ToChiselCamera(const double fx, const double fy, const double cx, const double cy, const int width, const int height)
{
    chisel::PinholeCamera cameraToReturn;
    chisel::Intrinsics intrinsics;
    intrinsics.SetFx(fx);
    intrinsics.SetFy(fy);
    intrinsics.SetCx(cx);
    intrinsics.SetCy(cy);
    cameraToReturn.SetIntrinsics(intrinsics);
    cameraToReturn.SetWidth(width);
    cameraToReturn.SetHeight(height);
    return cameraToReturn;
}

}


#endif // CONVERSIONS_H_ 
