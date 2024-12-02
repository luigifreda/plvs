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

#include "GlPointCloud.h"
#include "Utils.h"

#include <boost/make_shared.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>


#define USE_GL_MAP 1  // 1/0 enable/disable the use of glMapBuffer for drawing
                      // N.B: if you disable it the system can crash when you have a large number of vertices (e.g. with chisel)
                      // For more information about glMapBuffer take a look here:
                      // https://learnopengl.com/Advanced-OpenGL/Advanced-Data
                      // https://developer.apple.com/library/content/documentation/GraphicsImaging/Conceptual/OpenGL-MacProgGuide/opengl_vertexdata/opengl_vertexdata.html

// Usage is a hint to the GL implementation as to how a buffer object's data store will be accessed.
// This enables the GL implementation to make more intelligent decisions that may significantly impact buffer object performance.
// It does not, however, constrain the actual usage of the data store.
//#define BUFFER_DRAW_MODE GL_STATIC_DRAW  // The data store contents will be modified once and used many times as the source for GL drawing commands.
//#define BUFFER_DRAW_MODE GL_DYNAMIC_DRAW   // The data store contents will be modified repeatedly and used many times as the source for GL drawing commands
#define BUFFER_DRAW_MODE GL_STREAM_DRAW  // is used when your application needs to create transient geometry that is rendered and then discarded. This is most useful when your application must dynamically change vertex data every frame in a way that cannot be performed in a vertex shader. To use a stream vertex buffer, your application initially fills the buffer using glBufferData, then alternates between drawing using the buffer and modifying the buffer.


#define BUFFER_OFFSET(i) ((char*)NULL + (i))

namespace PLVS2
{


template<typename PointT>
GlPointCloud<PointT>::GlPointCloud(): GlObject(kGlObjectCloud),
vertexBufferId_(0), bBufferInitialized_(false), vertexBufferNumPoints_(0), colorOffset_(0), stride_(0)
{}
    

template<typename PointT>
GlPointCloud<PointT>::GlPointCloud(typename PointCloudT::Ptr pCloud): GlObject(kGlObjectCloud)
{
    this->SetPointCloud(pCloud);
}


template<typename PointT>
GlPointCloud<PointT>::~GlPointCloud()
{
    this->Delete();
}


template<typename PointT>
void GlPointCloud<PointT>::SetPointCloud(typename PointCloudT::Ptr pCloud)
{        
    std::unique_lock<std::mutex> lck(mutex_); 
    pCloud_ = pCloud;
    
    colorOffset_ = PointUtils::colorOffset(*pCloud);    
    vertexBufferNumPoints_ = pCloud->size(); 
    stride_ = sizeof(PointT);    
}


template<typename PointT>
void GlPointCloud<PointT>::SetFaces(const std::vector<uint32_t>& faces)
{
    std::unique_lock<std::mutex> lck(mutex_); 
    faces_ = faces;
}


template<typename PointT>
void GlPointCloud<PointT>::SetTransform(const Eigen::Affine3d& T)
{
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10
    typename PointCloudT::Ptr pCloudOut = boost::make_shared< PointCloudT >();
#else
    typename PointCloudT::Ptr pCloudOut = std::make_shared< PointCloudT >(); 
#endif 

    bool bNormalAvailable = pcl::traits::has_field<PointT, pcl::fields::normal_x>::value;
    if(!bNormalAvailable)
        pcl::transformPointCloud(*pCloud_, *pCloudOut, T.matrix());
    else
        pcl::transformPointCloudWithNormals(*pCloud_, *pCloudOut, T.matrix());

    pCloud_ = pCloudOut;
}


template<typename PointT>
void GlPointCloud<PointT>::Init()
{
    std::unique_lock<std::mutex> lck(mutex_); 
        
    // if there are no vertices, done!
    if (!pCloud_)
    {
        return; /// < EXIT POINT
    }
    if (pCloud_->empty())
    {
        return; /// < EXIT POINT
    }  
    
    std::cout << "init point cloud " << std::endl; 
    
    if(!bBufferInitialized_)
    {                
        glGenBuffers(1, &vertexBufferId_);
        glGenBuffers(1, &faceBufferId_);        
        
        // < vertices 
        glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId_);
                
#if !USE_GL_MAP
        glBufferData(GL_ARRAY_BUFFER, cloud->points.size() * stride_, &(pCloud_->points[0].x), BUFFER_DRAW_MODE);
#else
        // from https://learnopengl.com/Advanced-OpenGL/Advanced-Data
        glBufferData(GL_ARRAY_BUFFER, pCloud_->points.size() * stride_, NULL, GL_STREAM_DRAW);
        // get a pointer to memory which can be used to update the buffer
        void *vbo_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
        // now copy data into memory
        memcpy(vbo_ptr, &(pCloud_->points[0].x), pCloud_->points.size() * stride_);
        // make sure to tell OpenGL we're done with the pointer
        glUnmapBuffer(GL_ARRAY_BUFFER);
#endif      
        
        // < faces 
        if (faceBufferNumFaces_ > 0)
        {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faceBufferId_);
#if !USE_GL_MAP
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof (unsigned int) * faceBufferNumFaces_, faces_.data(), BUFFER_DRAW_MODE);
#else
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof (unsigned int) * faceBufferNumFaces_, NULL, GL_STREAM_DRAW);
            // get a pointer to memory which can be used to update the buffer
            void *fbo_ptr = glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);
            // now copy data into memory
            memcpy(fbo_ptr, faces_.data(), sizeof (unsigned int) * faceBufferNumFaces_);
            // make sure to tell OpenGL we're done with the pointer
            glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
#endif
        }        
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);  
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);        

        bBufferInitialized_ = true;
    }
}


template<typename PointT>
void GlPointCloud<PointT>::Delete()
{
    std::unique_lock<std::mutex> lck(mutex_);     
    if (bBufferInitialized_)
    {
        std::unique_lock<std::mutex> lck(mutex_);
        glDeleteBuffers(1, &vertexBufferId_);
        glDeleteBuffers(1, &faceBufferId_);        
        bBufferInitialized_ = false;
    }    
}


template<typename PointT>
void GlPointCloud<PointT>::Draw()
{
    std::unique_lock<std::mutex> lck(mutex_); 
    
    if (!bBufferInitialized_)
    {
        return;
    }
    
    // if there are no vertices, done!
    if (!pCloud_)
    {     
        return; /// < EXIT POINT
    }
    if (pCloud_->empty())
    {          
        return; /// < EXIT POINT
    }    
    
    vertexBufferNumPoints_ = pCloud_->points.size();
    faceBufferNumFaces_ = faces_.size();    
    
    glPointSize(pointSize_);

    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId_);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, stride_, BUFFER_OFFSET(0)); // the starting point of the VBO, for the vertices

    //glBindBuffer(GL_ARRAY_BUFFER,colorBufferId);
    if( displayMode_ == kDisplayModeWireFrame )
    {
        glColor3f(0.08f, 0.83f, 0.98f);
    }
    else
    {
        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(3, GL_UNSIGNED_BYTE, stride_, BUFFER_OFFSET(colorOffset_ * sizeof (float)));
    }

    if (displayMode_ == kDisplayModeWireFrame)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0f);
    }
    else
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    
    if (displayMode_ == kDisplayModePoints)
    {
        if(vertexBufferNumPoints_ > 0)
        {
            glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints_);
        }
    }
    else
    {
        if (faceBufferNumFaces_ > 0)
        {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faceBufferId_);
            glDrawElements(GL_TRIANGLES, faceBufferNumFaces_, GL_UNSIGNED_INT, 0);
        }
        else
        {
            if(vertexBufferNumPoints_ > 0)
            {
                glDrawArrays(glDrawingModePoints_, 0, vertexBufferNumPoints_);
            }
        }
    }

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    /// < end drawing, restore default GL attributes

    if (displayMode_ == kDisplayModeWireFrame)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glLineWidth(1.f);
    }

    // bind with 0, so, switch back to normal pointer operation
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);    
}


template<typename PointT>
void GlPointCloud<PointT>::SetType(const GlObjectType& type) 
{ 
    if(type == kGlObjectCloud) glDrawingModePoints_ = GL_POINTS;
    if(type == kGlObjectCloudMesh) glDrawingModePoints_ = GL_TRIANGLES;
}


template<typename PointT>
bool GlPointCloud<PointT>::Load(const std::string& filename)
{      
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10
    typename PointCloudT::Ptr pCloudIn = boost::make_shared< PointCloudT >();
#else
    typename PointCloudT::Ptr pCloudIn = std::make_shared< PointCloudT >(); 
#endif 

    std::cout << "GlPointCloud::LoadMap(): " << filename << std::endl;     
    {
    if (!Utils::fileExist(filename)) 
    {
        std::cerr << "Cannot open dense map file: " << filename << std::endl;
        return false;
    }
    }    
    
    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(filename, mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *pCloudIn); 
    
    // < N.B.: here we assume each face is a triangle 
    faces_.clear();
    if(mesh.polygons.size()> 0) 
    {
        for(size_t ii=0; ii<mesh.polygons.size(); ii++)
        {
            // each triangle/polygon
            pcl::Vertices poly = mesh.polygons[ii];
            for(size_t jj = 0; jj < poly.vertices.size (); jj++)
            { 
                // each point
                uint32_t pt = poly.vertices[jj];            
                faces_.push_back(pt);
            }
        }
    }
    faceBufferNumFaces_ = faces_.size();
    
    std::cout << "GlPointCloud::LoadMap(): number of points: " << pCloudIn->size() << std::endl; 
    std::cout << "GlPointCloud::LoadMap(): number of polygons: " << mesh.polygons.size() << std::endl;     
    
    bool bHasNormals = false;
    for(size_t ii=0; ii < mesh.cloud.fields.size(); ii++)
    {
        const pcl::PCLPointField& field = mesh.cloud.fields[ii];
        if( field.name == "normal_x") bHasNormals = true; 
    }
    
    /*if(!bHasNormals)
    {
        // estimate normals 
        this->ComputeNormals(pPointCloud_);
    }*/
    
    // invert back RGB colors (we used BGR in PointCloudMapping)
    this->InvertColors(pCloudIn);
    
    this->SetPointCloud(pCloudIn);
    
    return true;
}

template<typename PointT>
void GlPointCloud<PointT>::InvertColors(typename PointCloudT::Ptr pCloud)
{        
    if (pCloud->empty()) return;
    
    // invert back RGB colors (we used BGR in PointCloudMapping)
    for (size_t jj = 0, jjEnd=pCloud->size(); jj < jjEnd; jj++)
    {
        std::swap(pCloud->points[jj].r, pCloud->points[jj].b); 
    }    
}

} //namespace PLVS2



