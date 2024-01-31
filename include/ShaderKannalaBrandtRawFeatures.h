/*
 * This file is part of PLVS
 * Copyright (C) 2018-present Luigi Freda <luigifreda at gmail dot com>
 * This file was originally part of ElasticFusion and was modified for its integration in PLVS. 
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

#pragma once

#include "Shaders.h" 
#include "GeometricCamera.h"

namespace PLVS2  
{

class ShaderKannalaBrandtRawFeatures : public Shader
{
public:
    ShaderKannalaBrandtRawFeatures() = default;
    virtual ~ShaderKannalaBrandtRawFeatures();

    void Load(const std::string& vertex_shader_file, const std::string& fragment_shader_file);

    void SetUniforms();

    void DrawPoints(const std::vector<Eigen::Vector3f>& points, float pointSize=5.0f);
    void DrawLines(const std::vector<Eigen::Vector3f>& lines, float lineWidth=3.0f);

public: 

    void SetCamera(const GeometricCamera* camera){ camera_ = camera; }

protected:

    GLuint matProjectionUniformLoc_;
    GLuint matModelViewUniformLoc_;
    GLuint colorUniformLoc_;
    GLuint alphaUniformLoc_;
    GLuint minDistanceColorUniformLoc_;
    GLuint maxDistanceColorUniformLoc_;
    GLuint maxDistanceRenderUniformLoc_;
    GLuint kbIntrinsicsk1k2k3k4UniformLoc_; // k1 k2 k3 k4

    GLuint pointsBufferId_ = 0;
    GLuint linesBufferId_ = 0;

    std::shared_ptr<Shader> program_;   

    Eigen::Vector3f color_ = Eigen::Vector3f(0.471,0.98,0.125);
    float maxDistanceRender_ = std::numeric_limits<float>::max();
    float minDistanceColor_ = 0.1f;
    float maxDistanceColor_ = 20.0f;
    float alpha_ = 1.0f;   

    const GeometricCamera* camera_ = nullptr;
};


inline ShaderKannalaBrandtRawFeatures::~ShaderKannalaBrandtRawFeatures()
{
    if (pointsBufferId_) glDeleteBuffers(1, &pointsBufferId_);
    if (linesBufferId_) glDeleteBuffers(1, &linesBufferId_);        
}

inline void ShaderKannalaBrandtRawFeatures::Load(const std::string& vertex_shader_file, const std::string& fragment_shader_file)
{
    this->AddShaderFromFile(pangolin::GlSlVertexShader, getShadersDir() + "/" + vertex_shader_file, {}, {getShadersDir()});
    this->AddShaderFromFile(pangolin::GlSlFragmentShader, getShadersDir() + "/" + fragment_shader_file, {}, {getShadersDir()});
    this->Link();

    matProjectionUniformLoc_ = glGetUniformLocation(programId(), "matProjection");
    matModelViewUniformLoc_ = glGetUniformLocation(programId(), "matModelView");

    maxDistanceRenderUniformLoc_ = glGetUniformLocation(programId(), "maxDistanceRender");

    colorUniformLoc_ = glGetUniformLocation(programId(), "color");
    alphaUniformLoc_ = glGetUniformLocation(programId(), "alpha");
    minDistanceColorUniformLoc_ = glGetUniformLocation(programId(), "minDistanceColor");
    maxDistanceColorUniformLoc_ = glGetUniformLocation(programId(), "maxDistanceColor");

    kbIntrinsicsk1k2k3k4UniformLoc_ = glGetUniformLocation(programId(), "kbIntrinsicsk1k2k3k4");
}

inline void ShaderKannalaBrandtRawFeatures::SetUniforms()
{
    const Eigen::Matrix4f projection = pangolin::ToEigen<float>(GetProjectionMatrix()); 
    const Eigen::Matrix4f modelView = pangolin::ToEigen<float>(GetModelViewMatrix());

    glUniform3fv(colorUniformLoc_, 1, color_.data());
    glUniformMatrix4fv(matProjectionUniformLoc_, 1, GL_FALSE, projection.data());
    glUniformMatrix4fv(matModelViewUniformLoc_, 1, GL_FALSE, modelView.data());
    glUniform1f(alphaUniformLoc_, alpha_);

    glUniform1f(maxDistanceRenderUniformLoc_, maxDistanceRender_);
    glUniform1f(minDistanceColorUniformLoc_, minDistanceColor_);
    glUniform1f(maxDistanceColorUniformLoc_, maxDistanceColor_);

    const float fx = camera_->getParameter(0);
    const float fy = camera_->getParameter(1);
    const float cx = camera_->getParameter(2);
    const float cy = camera_->getParameter(3);

    const float k1 = camera_->getParameter(4);
    const float k2 = camera_->getParameter(5);
    const float k3 = camera_->getParameter(6);
    const float k4 = camera_->getParameter(7);

    glUniform4f (kbIntrinsicsk1k2k3k4UniformLoc_, k1, k2, k3, k4);        
}

inline void ShaderKannalaBrandtRawFeatures::DrawPoints(const std::vector<Eigen::Vector3f>& points, float pointSize)
{
    if(!pointsBufferId_) glGenBuffers(1, &pointsBufferId_);

    size_t numPoints = points.size();
    if(numPoints == 0) return;

    glBindBuffer(GL_ARRAY_BUFFER, pointsBufferId_);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(Eigen::Vector3f), points.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glPointSize(pointSize);

    // vertices
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, pointsBufferId_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    Bind();

    SetUniforms();

    glDrawArrays(GL_POINTS, 0, numPoints);

    Unbind();

    glDisableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glPointSize(1.0);     
}

inline void ShaderKannalaBrandtRawFeatures::DrawLines(const std::vector<Eigen::Vector3f>& lines, float lineWidth)
{
    if(!linesBufferId_) glGenBuffers(1, &linesBufferId_);

    size_t numLines = lines.size()/2; 
    if(numLines == 0) return;

    glBindBuffer(GL_ARRAY_BUFFER, linesBufferId_);
    glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(Eigen::Vector3f), lines.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glLineWidth(lineWidth);

    // vertices
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, linesBufferId_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    Bind();

    SetUniforms();

    glDrawArrays(GL_LINES, 0, 2 * numLines);

    Unbind();
           
    glDisableVertexAttribArray(0);
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glLineWidth(1.0);        
}    

} // namespace PLVS2

