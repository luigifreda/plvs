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

#ifndef POINT_CLOUD_DRAWER_H
#define POINT_CLOUD_DRAWER_H


#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

#include <GL/glew.h>
#include <GL/gl.h>

#include <sstream>
#include <fstream>

#include "PointDefinitions.h"

#include <pcl/common/transforms.h>

namespace PLVS2
{

class PointCloudMapping;
class Shader;

class PointCloudDrawer
{
public:

    typedef POINT_TYPE PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    enum DisplayMode {kDisplayModeFaces=1, kDisplayModeWireFrame=2, kDisplayModePoints=3};

public:

    PointCloudDrawer();
    ~PointCloudDrawer();

    void SetPointCloudMapping(std::shared_ptr<PointCloudMapping> pPointCloudMapping);

    void DrawPC(float pointSize = 2, float alpha = 1);
    void RefreshPC();
    void DestroyBuffers();

    void DrawPoints(float pointSize = 2, float alpha = 1);
    void DrawNormals();
    void DrawSegments(float pointSize = 2);

    void SavePointCloud();

    void SetDisplayMode(int val);

    void SetDisplayCarved(bool val);

    void SetDisplayNormals(bool val);

    void SetDisplaySegments(bool val);

    void SetUseAR(bool value);

    void SetNormalsProgram(std::shared_ptr<Shader>& normalsProgram);

    void SetSegmentsProgram(std::shared_ptr<Shader>& segmentsProgram);

    void SetDebugProgram(std::shared_ptr<Shader>& debugProgram);

    void SetDebugFontTextureId(const int id);

    void SetLabelConfidenceThreshold(const unsigned int th);

public:

    std::shared_ptr<PointCloudMapping>& GetPointCloudMapping() { return pPointCloudMapping_; }

protected:

    std::shared_ptr<PointCloudMapping> pPointCloudMapping_;

    std::mutex cloud_mutex_;
    PointCloudT::Ptr pCloud_;
    PointCloudT::Ptr pCloudCarved_;
    std::vector<unsigned int> faces_;
    std::uint64_t cloud_timestamp_;

    // buffer & how many
    GLuint vertexBufferId_;  //GLuint colorBufferId_;
    int vertexBufferNumPoints_;
    
    GLuint faceBufferId_;
    int faceBufferNumFaces_;
    
    GLuint vertexCarvedBufferId_;
    int vertexCarvedBufferNumPoints_;
    
    bool bBufferInitialized_; // true if the vertixBufferID is valid

    int displayMode_;
    int displayModeBeforeAR_;

    GLint glDrawingModePoints_;

    std::atomic_bool bDisplayCarved_;
    std::atomic_bool bDisplayNormals_;
    std::atomic_bool bDisplaySegments_;

    std::shared_ptr<Shader> normalsProgram_;
    std::shared_ptr<Shader> segmentsProgram_;
    std::shared_ptr<Shader> debugProgram_;

    GLuint debugFontTextureID_;
    GLuint vertexBufferDebugId_;

    unsigned int labelConfidenceThreshold_;

    bool mbUseAR; // use AR visualization;
};


} //namespace PLVS2

#endif


