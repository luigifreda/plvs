/*
 * This file is part of PLVS
 * Copyright (C) 2018-present Luigi Freda <luigifreda at gmail dot com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef GL_POINT_CLOUD_H
#define GL_POINT_CLOUD_H


#include <mutex>
#include <string>

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

#include<Eigen/Dense>

#include "PointDefinitions.h"
#include "GlObject.h"

namespace PLVS2
{

template<typename PointT>
class GlPointCloud:
public GlObject
{   
public:

    typedef typename pcl::PointCloud<PointT> PointCloudT;
        
public:

    GlPointCloud();
    GlPointCloud(typename PointCloudT::Ptr pCloud);
  
    virtual ~GlPointCloud();
    
    void Init();
    void Delete();    
    void Draw();
    
    bool Load(const std::string& filename);
    
public: // setters 
    
    void SetPointCloud(typename PointCloudT::Ptr pCloud); 
    void SetFaces(const std::vector<uint32_t>& faces);
    void SetTransform(const Eigen::Affine3d& T);
        
    void SetType(const GlObjectType& type);    
    
    // GL_POINTS or GL_TRIANGLES
    void SetDrawingModePoints(GLint glDrawingModePoints) { glDrawingModePoints_ = glDrawingModePoints; }
    
    
public: // getters     

    int GetNumPoints() const { return vertexBufferNumPoints_; }
    
protected:     
    
    void InvertColors(typename PointCloudT::Ptr pCloud);
        
protected:
    
    std::mutex mutex_;
    
    typename PointCloudT::Ptr pCloud_;
    std::vector<uint32_t> faces_;    
    
    GLuint vertexBufferId_ = 0;    
    int vertexBufferNumPoints_ = 0;
    
    GLuint faceBufferId_ = 0;
    int faceBufferNumFaces_ = 0;    
    
    bool bBufferInitialized_ = false; // true if the vertixBufferID is valid               
    
    int colorOffset_ = 0;
    int stride_ = 0;
    
    int pointSize_ = 2;
    GLint glDrawingModePoints_ = GL_POINTS;    
    
};


namespace PointUtils
{

template <class PointT, typename std::enable_if<!pcl::traits::has_normal<PointT>::value>::type* = nullptr>
inline int colorOffset(const typename pcl::PointCloud<PointT>& pCloud)
{
    return 4; // pcl::PointXYZRGBA
}

template <class PointT, typename std::enable_if<pcl::traits::has_normal<PointT>::value>::type* = nullptr>
inline int colorOffset(const typename pcl::PointCloud<PointT>& pCloud)
{
    return 8; // pcl::PointXYZRGBNormal, pcl::PointSurfelSegment
}

} // namespace PointUtils


#if !USE_NORMALS

/// < list here the types you want to use 
template class GlPointCloud<pcl::PointXYZRGBA>;

#else

template class GlPointCloud<pcl::PointXYZRGBNormal>;
template class GlPointCloud<pcl::PointSurfelSegment>;

#endif


} //namespace PLVS2

#endif


