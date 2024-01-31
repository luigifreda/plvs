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
/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#ifndef SHADERS_SHADERS_H_
#define SHADERS_SHADERS_H_

#include <memory>
#include <string>
#include <Eigen/Core>
#include <pangolin/gl/glsl.h>


#define XSTR(x) #x
#define STR(x) XSTR(x)

namespace PLVS2  
{

class Uniform
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
        Uniform(const std::string & id, const int & v)
         : id(id),
           i(v),
           t(INT)
        {}

        Uniform(const std::string & id, const unsigned int & v)
         : id(id),
           i(v),
           t(UINT)
        {}
                 
        Uniform(const std::string & id, const float & v)
         : id(id),
           f(v),
           t(FLOAT)
        {}

        Uniform(const std::string & id, const Eigen::Vector2f & v)
         : id(id),
           v2(v),
           t(VEC2)
        {}

        Uniform(const std::string & id, const Eigen::Vector3f & v)
         : id(id),
           v3(v),
           t(VEC3)
        {}

        Uniform(const std::string & id, const Eigen::Vector4f & v)
         : id(id),
           v4(v),
           t(VEC4)
        {}

        Uniform(const std::string & id, const Eigen::Matrix4f & v)
         : id(id),
           m4(v),
           t(MAT4)
        {}

        std::string id;

        int i;
        float f;
        Eigen::Vector2f v2;
        Eigen::Vector3f v3;
        Eigen::Vector4f v4;
        Eigen::Matrix4f m4;

        enum Type
        {
            INT,
            UINT,
            FLOAT,
            VEC2,
            VEC3,
            VEC4,
            MAT4,
            NONE
        };

        Type t;
};


class Shader : public pangolin::GlSlProgram
{
    public:
        Shader() = default; 

        GLuint programId() const
        {
            return prog;
        }

        void setUniform(const Uniform & v)
        {
            GLuint loc = glGetUniformLocation(prog, v.id.c_str());

            switch(v.t)
            {
                case Uniform::INT:
                    glUniform1i(loc, v.i);
                    break;
                case Uniform::UINT:
                    glUniform1ui(loc, v.i);
                    break;                    
                case Uniform::FLOAT:
                    glUniform1f(loc, v.f);
                    break;
                case Uniform::VEC2:
                    glUniform2f(loc, v.v2(0), v.v2(1));
                    break;
                case Uniform::VEC3:
                    glUniform3f(loc, v.v3(0), v.v3(1), v.v3(2));
                    break;
                case Uniform::VEC4:
                    glUniform4f(loc, v.v4(0), v.v4(1), v.v4(2), v.v4(3));
                    break;
                case Uniform::MAT4:
                    glUniformMatrix4fv(loc, 1, false, v.m4.data());
                    break;
                default:
                    assert(false && "Uniform type not implemented!");
                    break;
            }
        }
        
        void SetTexture(const std::string& id, int numTextureUnit)
        {
            GLuint loc = glGetUniformLocation(prog, id.c_str());
            glUniform1i(loc,numTextureUnit);
        }
        
        void SetProjectionModelViewMatrix(const pangolin::OpenGlMatrix& mvp)
        {
            matProjModelView_ = mvp; 
        }

        void SetModelViewMatrix(const pangolin::OpenGlMatrix& mv)
        {
            matModelView_ = mv; 
        }

        void SetProjectionMatrix(const pangolin::OpenGlMatrix& mp)
        {
            matProjection_ = mp; 
        }

    public: 

        const pangolin::OpenGlMatrix& GetProjectionModelViewMatrix() const
        {
            return matProjModelView_;
        }

        const pangolin::OpenGlMatrix& GetModelViewMatrix() const
        {
            return matModelView_;
        }

        const pangolin::OpenGlMatrix& GetProjectionMatrix() const
        {
            return matProjection_;
        }

    protected: 

        pangolin::OpenGlMatrix matProjModelView_; // combined projection and modelview
        pangolin::OpenGlMatrix matModelView_;
        pangolin::OpenGlMatrix matProjection_;
};

static inline std::string getShadersDir()
{
    std::string currentVal = STR(SHADERS_DIR); //SHADERS_DIR set by compilers flag 

    assert(pangolin::FileExists(currentVal) && "Shader directory not found!");

    return currentVal;
}
        
static inline std::shared_ptr<Shader> loadProgramGeomFromFile(const std::string& vertex_shader_file, const std::string& geometry_shader_file)
{
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    program->AddShaderFromFile(pangolin::GlSlVertexShader, getShadersDir() + "/" + vertex_shader_file, {}, {getShadersDir()});
    program->AddShaderFromFile(pangolin::GlSlGeometryShader, getShadersDir() + "/" + geometry_shader_file, {}, {getShadersDir()});
    program->Link();

    return program;
}

static inline std::shared_ptr<Shader> loadProgramFromFile(const std::string& vertex_shader_file)
{
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    program->AddShaderFromFile(pangolin::GlSlVertexShader, getShadersDir() + "/" + vertex_shader_file, {}, {getShadersDir()});
    program->Link();

    return program;
}

static inline std::shared_ptr<Shader> loadProgramFromFile(const std::string& vertex_shader_file, const std::string& fragment_shader_file)
{
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    program->AddShaderFromFile(pangolin::GlSlVertexShader, getShadersDir() + "/" + vertex_shader_file, {}, {getShadersDir()});
    program->AddShaderFromFile(pangolin::GlSlFragmentShader, getShadersDir() + "/" + fragment_shader_file, {}, {getShadersDir()});
    program->Link();

    return program;  
}

static inline std::shared_ptr<Shader> loadProgramFromFile(const std::string& vertex_shader_file, const std::string& fragment_shader_file, const std::string& geometry_shader_file)
{
    std::shared_ptr<Shader> program = std::make_shared<Shader>();

    program->AddShaderFromFile(pangolin::GlSlVertexShader, getShadersDir() + "/" + vertex_shader_file, {}, {getShadersDir()});
    program->AddShaderFromFile(pangolin::GlSlGeometryShader, getShadersDir() + "/" + geometry_shader_file, {}, {getShadersDir()});
    program->AddShaderFromFile(pangolin::GlSlFragmentShader, getShadersDir() + "/" + fragment_shader_file, {}, {getShadersDir()});
    program->Link();

    return program;
}

} // namespace PLVS2

#endif /* SHADERS_SHADERS_H_ */
