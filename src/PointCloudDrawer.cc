/*
 * This file is part of PLVS.
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


//#define GL_GLEXT_PROTOTYPES 1


#include <stdio.h>
#include <sstream>
#include <chrono>

//#include <GL/glx.h>
//#include <GL/gl.h>
//#include <GL/glu.h>

#include "PointCloudDrawer.h"

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Core>


#include "PointCloudMapping.h"
#include "Shaders.h"



#define USE_GL_MAP 1  // 1/0 enable/disable the use of glMapBuffer for drawing
                      // N.B: if you disable it the system can crash when you have a large number of vertices (e.g. with chisel)
                      // For more information about glMapBuffer take a look here:
                      // https://learnopengl.com/Advanced-OpenGL/Advanced-Data
                      // https://developer.apple.com/library/content/documentation/GraphicsImaging/Conceptual/OpenGL-MacProgGuide/opengl_vertexdata/opengl_vertexdata.html

using namespace std;

#define BUFFER_OFFSET(i) ((char*)NULL + (i))

// Usage is a hint to the GL implementation as to how a buffer object's data store will be accessed.
// This enables the GL implementation to make more intelligent decisions that may significantly impact buffer object performance.
// It does not, however, constrain the actual usage of the data store.
//#define BUFFER_DRAW_MODE GL_STATIC_DRAW  // The data store contents will be modified once and used many times as the source for GL drawing commands.
//#define BUFFER_DRAW_MODE GL_DYNAMIC_DRAW   // The data store contents will be modified repeatedly and used many times as the source for GL drawing commands
#define BUFFER_DRAW_MODE GL_STREAM_DRAW  // is used when your application needs to create transient geometry that is rendered and then discarded. This is most useful when your application must dynamically change vertex data every frame in a way that cannot be performed in a vertex shader. To use a stream vertex buffer, your application initially fills the buffer using glBufferData, then alternates between drawing using the buffer and modifying the buffer.

void check_gl_errors()
{
    /* Check for errors */
    GLenum error;
    error = glGetError();
    switch (error)
    {
    case GL_NO_ERROR:
        //std::cout <<  "[check_gl_errors()]:\t\tError: No error!" << std::endl;
        break;

    case GL_INVALID_ENUM:
        std::cout << "[check_gl_errors()]:\t\tError: GL_INVALID_ENUM" << std::endl;
        break;

    case GL_INVALID_VALUE:
        std::cout << "[check_gl_errors()]:\t\tError: GL_INVALID_VALUE" << std::endl;
        break;

    case GL_INVALID_OPERATION:
        std::cout << "[check_gl_errors()]:\t\tError: GL_INVALID_OPERATION" << std::endl;
        break;

    case GL_STACK_OVERFLOW:
        std::cout << "[check_gl_errors()]:\t\tError: GL_STACK_OVERFLOW" << std::endl;
        break;

    case GL_STACK_UNDERFLOW:
        std::cout << "[check_gl_errors()]:\t\tError: GL_STACK_UNDERFLOW" << std::endl;
        break;

    case GL_OUT_OF_MEMORY:
        std::cout << "[check_gl_errors()]:\t\tError: GL_OUT_OF_MEMORY" << std::endl;
        break;
    }

}

// An array of 3 vectors which represents 3 vertices
static const GLfloat quad_data[] =
{
    1.0f,   1.0f, 0.0f,
    1.0f,   0.9f, 0.0f,
   -1.0f,   0.9f, 0.0f,
   -1.0f,   1.0f, 0.0f,
};
static const int quad_data_size = 4;


namespace PLVS2
{

PointCloudDrawer::PointCloudDrawer()
{    
    pPointCloudMapping_ = 0;
    cloud_timestamp_ = 0;

    vertexBufferId_ = 0;
    vertexBufferNumPoints_ = 0;
    
    faceBufferId_ = 0;    
    faceBufferNumFaces_ = 0;

    vertexCarvedBufferId_ = 0;
    vertexCarvedBufferNumPoints_ = 0;
    
    bBufferInitialized_ = false;    

    glDrawingModePoints_ = GL_POINTS;

    bDisplayCarved_ = false;
    bDisplayNormals_ = false;
    bDisplaySegments_ = false;

    labelConfidenceThreshold_ = 0;

    displayMode_ = kDisplayModeFaces;
    displayModeBeforeAR_ = displayMode_;
    
    mbUseAR = false;
}

PointCloudDrawer::~PointCloudDrawer()
{
    std::cout << "PointCloudDrawer::~PointCloudDrawer() - start" << std::endl; 
    this->DestroyBuffers();
    std::cout << "PointCloudDrawer::~PointCloudDrawer() - end" << std::endl;     
}

void PointCloudDrawer::SetPointCloudMapping(std::shared_ptr<PointCloudMapping> pPointCloudMapping)
{
    pPointCloudMapping_ = pPointCloudMapping;

    if( (pPointCloudMapping_->GetMapType() == PointCloudMapTypes::kChisel) ||
        (pPointCloudMapping_->GetMapType() == PointCloudMapTypes::kVoxblox)
      )
    {
        glDrawingModePoints_ = GL_TRIANGLES;
    }

    labelConfidenceThreshold_ = std::max(pPointCloudMapping_->GetSegmentationLabelConfidenceThreshold(),0);
}

void PointCloudDrawer::DestroyBuffers()
{ 
    if (bBufferInitialized_)
    {
        std::cout << "PointCloudDrawer::DestroyBuffers() - start" << std::endl;            
        unique_lock<mutex> lck(cloud_mutex_);
        glDeleteBuffers(1, &vertexBufferId_);
        glDeleteBuffers(1, &faceBufferId_);
        glDeleteBuffers(1, &vertexCarvedBufferId_);        
        bBufferInitialized_ = false;
        
        pPointCloudMapping_.reset();
        std::cout << "PointCloudDrawer::DestroyBuffers() - end" << std::endl;        
    }         
}

void PointCloudDrawer::RefreshPC()
{
    if (!pPointCloudMapping_) return; /// < EXIT POINT

    //unique_lock<mutex> lck(cloud_mutex_); // already locked in the calling DrawPC()

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    std::uint64_t current_cloud_timestamp = pPointCloudMapping_->GetMapTimestamp();
    if (current_cloud_timestamp > cloud_timestamp_)
    {
        //std::cout << "PointCloudDrawer::refreshPC() - asking new point cloud" << std::endl;
        //cloud_ = pPointCloudMapping_->GetMap();
        pPointCloudMapping_->GetMap(pCloud_, pCloudCarved_, faces_, bDisplayCarved_);
        //std::cout << "PointCloudDrawer::refreshPC() - got output" << std::endl;
    }
    else
    {
        // no need to get a new point cloud
        return;
    }

    // if there are no vertices, done!
    if (!pCloud_)
    {
        //std::cout << "PointCloudDrawer::refreshPC() - got empty pointer" << std::endl;
        return; /// < EXIT POINT
    }
    if (pCloud_->empty())
    {
        //std::cout << "PointCloudDrawer::refreshPC() - got empty point cloud" << std::endl;
        return; /// < EXIT POINT
    }

    ///  < here we update the VBOs

    vertexBufferNumPoints_ = pCloud_->size();
    faceBufferNumFaces_ = faces_.size();

    cloud_timestamp_ = pCloud_->header.stamp;
    std::cout << "PointCloudDrawer::refreshPC(), #points=" << vertexBufferNumPoints_ << ", #faces=" << faceBufferNumFaces_ << std::endl << std::flush;
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif    
    std::cout << "\nPointCloudDrawer::refreshPC(), point cloud update time: " <<  std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() << std::endl;

    // generate buffers
    if (!bBufferInitialized_)
    {
        glGenBuffers(1, &vertexBufferId_);
        //glGenBuffers(1, &colorBufferId_);
        glGenBuffers(1, &faceBufferId_);

        glGenBuffers(1, &vertexCarvedBufferId_);

        //glGenBuffers(1, &vertexBufferDebugId_);

        bBufferInitialized_ = true;
    }

    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId_); // for vertex coordinates

#if !USE_GL_MAP
    glBufferData(GL_ARRAY_BUFFER, sizeof (PointCloudMapping::PointT) * vertexBufferNumPoints_, &(pCloud_->points[0].x), BUFFER_DRAW_MODE);
    //glBindBuffer(GL_ARRAY_BUFFER,colorBufferId);
    //glBufferData(GL_ARRAY_BUFFER, 3 * vertexBufferNumPoints,&(cloud_->points[0].x), BUFFER_DRAW_MODE);
#else
    // from https://learnopengl.com/Advanced-OpenGL/Advanced-Data
    glBufferData(GL_ARRAY_BUFFER, sizeof (PointCloudMapping::PointT) * vertexBufferNumPoints_, NULL, GL_STREAM_DRAW);
    // get a pointer to memory which can be used to update the buffer
    void *vbo_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    // now copy data into memory
    memcpy(vbo_ptr, &(pCloud_->points[0].x), sizeof (PointCloudMapping::PointT) * vertexBufferNumPoints_);
    // make sure to tell OpenGL we're done with the pointer
    glUnmapBuffer(GL_ARRAY_BUFFER);
#endif


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


    if ((bDisplayCarved_)&&(pCloudCarved_))
    {
        vertexCarvedBufferNumPoints_ = pCloudCarved_->size();
        if (vertexCarvedBufferNumPoints_ > 0)
        {
            glBindBuffer(GL_ARRAY_BUFFER, vertexCarvedBufferId_); // for vertex coordinates
#if !USE_GL_MAP
            glBufferData(GL_ARRAY_BUFFER, sizeof (PointCloudMapping::PointT) * vertexCarvedBufferNumPoints_, &(pCloudCarved_->points[0].x), BUFFER_DRAW_MODE);
#else
            glBufferData(GL_ARRAY_BUFFER, sizeof (PointCloudMapping::PointT) * vertexCarvedBufferNumPoints_, NULL, GL_STREAM_DRAW);
            // get a pointer to memory which can be used to update the buffer
            void *vbo_carved_ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
            // now copy data into memory
            memcpy(vbo_carved_ptr, &(pCloudCarved_->points[0].x), sizeof (PointCloudMapping::PointT) * vertexCarvedBufferNumPoints_);
            // make sure to tell OpenGL we're done with the pointer
            glUnmapBuffer(GL_ARRAY_BUFFER);
#endif
        }
    }
    else
    {
        vertexCarvedBufferNumPoints_ = 0;
    }

    // debug
    //glBindBuffer(GL_ARRAY_BUFFER, vertexBufferDebugId_);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(quad_data), quad_data, BUFFER_DRAW_MODE);


    // bind with 0, so, switch back to normal pointer operation
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t3 = std::chrono::monotonic_clock::now();
#endif

    std::cout << "\nPointCloudDrawer::refreshPC(), point cloud GL buffers update time: " <<  std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count() << std::endl;

}

void PointCloudDrawer::DrawPC(float pointSize, float alpha)
{
    unique_lock<mutex> lck(cloud_mutex_); /// < we lock here for all the subsequent subcalls

    RefreshPC();

    if (!bBufferInitialized_)
    {
        return;
    }

    if(!bDisplaySegments_)
    {
        DrawPoints(pointSize, alpha);
    }
    DrawNormals();
    DrawSegments(pointSize);

}

void PointCloudDrawer::DrawPoints(float pointSize, float alpha)
{
    glPointSize(pointSize);

    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId_);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(0)); // the starting point of the VBO, for the vertices

    //glBindBuffer(GL_ARRAY_BUFFER,colorBufferId);
    if( mbUseAR && (displayMode_ == kDisplayModeWireFrame) )
    {
        glColor3f(0.08f, 0.83f, 0.98f);
    }
    else
    {
        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(3, GL_UNSIGNED_BYTE, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(POINT_TYPE_COLOR_OFFSET * sizeof (float)));
    }
    //glColorPointer(GL_BGRA, GL_UNSIGNED_BYTE, sizeof(PointCloudMapping::PointT), BUFFER_OFFSET(4 * sizeof (float)));
    //glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(PointCloudMapping::PointT), BUFFER_OFFSET(4 * sizeof (float)));

#if COMPUTE_NORMALS
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(POINT_TYPE_NORMAL_OFFSET * sizeof (float)));
#endif


    // glDrawArrays(glDrawingModePoints_, 0, vertexBufferNumPoints_);

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
        //glPointSize(2.0);
        //glBindBuffer(GL_ARRAY_BUFFER, _vertexBuffer);
        //glDrawArrays(GL_POINTS, 0, _currentMeshInterleaved->vertices.size());
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

#if COMPUTE_NORMALS
    glDisableClientState(GL_NORMAL_ARRAY);
#endif
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);



    /// < now display carved point cloud
    if (bDisplayCarved_ && (vertexCarvedBufferNumPoints_ > 0))
    {
        glPointSize(pointSize+2.0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexCarvedBufferId_);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(0)); // the starting point of the VBO, for the vertices

        glColor3f(0.99f, 0.f, 0.99f);

        glDrawArrays(GL_POINTS, 0, vertexCarvedBufferNumPoints_);

        glDisableClientState(GL_VERTEX_ARRAY);
    }


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

void PointCloudDrawer::DrawNormals()
{
    if (!(normalsProgram_ && bDisplayNormals_)) return;

    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId_);

    // vertices
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(0));

    // colors
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_FALSE, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(POINT_TYPE_COLOR_OFFSET * sizeof (float)));

#if COMPUTE_NORMALS
    // normals
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(POINT_TYPE_NORMAL_OFFSET * sizeof (float)));
#endif

    normalsProgram_->Bind();
    normalsProgram_->setUniform(Uniform("matModelView", normalsProgram_->GetProjectionModelViewMatrix())); // this mat is set in the main viewer

    //glDrawArrays(glDrawingModePoints_, 0, vertexBufferNumPoints_);
    if(vertexBufferNumPoints_ > 0)
    {
        glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints_);
    }

    normalsProgram_->Unbind();

    /// < disable all stuff and unbind buffers
#if COMPUTE_NORMALS
    glDisableVertexAttribArray(2);
#endif
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    // bind with 0, so, switch back to normal pointer operation
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


void PointCloudDrawer::DrawSegments(float pointSize)
{
    if (!(segmentsProgram_ && bDisplaySegments_)) return;

#if COMPUTE_SEGMENTS

    glPointSize(pointSize+3.0f);

    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferId_);

    // vertices
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(0));

    // labels and their data
    glEnableVertexAttribArray(1);
    //glVertexAttribPointer(1, 1, GL_UNSIGNED_SHORT, GL_FALSE, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(POINT_TYPE_LABEL_OFFSET)); // does not work properly
    glVertexAttribIPointer(1, 2, GL_UNSIGNED_INT, sizeof (PointCloudMapping::PointT), BUFFER_OFFSET(POINT_TYPE_LABEL_OFFSET));


    segmentsProgram_->Bind();
    segmentsProgram_->setUniform(Uniform("matModelView", segmentsProgram_->GetProjectionModelViewMatrix())); // this mat is set in the main viewer
    segmentsProgram_->setUniform(Uniform("labelConfidenceThreshold",labelConfidenceThreshold_));

    //glDrawArrays(glDrawingModePoints_, 0, vertexBufferNumPoints_);
    if(vertexBufferNumPoints_ > 0)
    {
        glDrawArrays(GL_POINTS, 0, vertexBufferNumPoints_);
    }

    segmentsProgram_->Unbind();

    /// < disable all stuff and unbind buffers

    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    // bind with 0, so, switch back to normal pointer operation
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glPointSize(1.0);

//    /// < debug
//
//    // 1rst attribute buffer : vertices
//    glEnableVertexAttribArray(0);
//    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferDebugId_);
//    glVertexAttribPointer(
//       0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
//       3,                  // size
//       GL_FLOAT,           // type
//       GL_FALSE,           // normalized?
//       0,                  // stride
//       (void*)0            // array buffer offset
//    );
//
//
//
//    glActiveTexture(GL_TEXTURE0 + 0);
//    glBindTexture(GL_TEXTURE_2D, debugFontTextureID_);
//
//    debugProgram_->Bind();
//    debugProgram_->SetTexture("txr_font", 0);
//    debugProgram_->SetUniform("fxs",float(8.0)/float(1024));
//    debugProgram_->SetUniform("fys",float(8.0)/float(768));
//
//
//    // Draw the triangle !
//    glDrawArrays(GL_QUADS, 0, quad_data_size); // Starting from vertex 0; 3 vertices total -> 1 triangle
//
//
//    debugProgram_->Unbind();
//
//
//    glDisableVertexAttribArray(0);
//    glBindBuffer(GL_ARRAY_BUFFER, 0);

#endif
}

void PointCloudDrawer::SavePointCloud()
{
    pPointCloudMapping_->SaveMap();
}

void PointCloudDrawer::SetDisplayMode(int val)
{
    //displayMode = (displayMode + 1) % 3;
    displayMode_ = val;
    switch (displayMode_)
    {
    case kDisplayModeFaces:
        std::cout << "Display Mode is Faces ************ \n";
        break;
    case kDisplayModeWireFrame:
        std::cout << "Display Mode is WireFrame ************ \n";
        break;
    case kDisplayModePoints:
        std::cout << "Display Mode is Points ************ \n";
        break;
    default:
        displayMode_ = (int) kDisplayModePoints;
        std::cout << "Default Display Mode is Points ************ \n";
    }
}

void PointCloudDrawer::SetDisplayCarved(bool val)
{
    bDisplayCarved_ = val;

    //    unique_lock<mutex> lck(cloud_mutex_);
    //    vertexCarvedBufferNumPoints_ = 0;
    //    pCloudCarved_ = 0;
}

void PointCloudDrawer::SetDisplayNormals(bool val)
{
    bDisplayNormals_ = val;
}

void PointCloudDrawer::SetDisplaySegments(bool val)
{
    bDisplaySegments_ = val;
}

void PointCloudDrawer::SetUseAR(bool value) 
{ 
    if(!mbUseAR && value) 
    {
        displayModeBeforeAR_ = displayMode_;  // store old display mode 
        displayMode_ = kDisplayModeWireFrame; // force wireframe in AR mode 
    }
    else
    {
        displayMode_ = displayModeBeforeAR_;  // restore the last display mode before the AR switch       
    }
        
    mbUseAR = value; 
}

void PointCloudDrawer::SetNormalsProgram(std::shared_ptr<Shader>& normalsProgram)
{
    normalsProgram_ = normalsProgram;
}

void PointCloudDrawer::SetSegmentsProgram(std::shared_ptr<Shader>& segmentsProgram)
{
    segmentsProgram_ = segmentsProgram;
}

void PointCloudDrawer::SetDebugProgram(std::shared_ptr<Shader>& debugProgram)
{
    debugProgram_ = debugProgram;
}

void PointCloudDrawer::SetDebugFontTextureId(const int id)
{
    debugFontTextureID_ = id;
}

void PointCloudDrawer::SetLabelConfidenceThreshold(const unsigned int th)
{
    labelConfidenceThreshold_ = th;
}

} //namespace PLVS2
