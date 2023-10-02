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

#ifndef GL_OBJECT_H
#define GL_OBJECT_H

#include <memory>

#include "GlColors.h"

#ifndef UNUSED_VAR
#define UNUSED_VAR(x) (void)x
#endif 

namespace PLVS2
{

enum GlObjectType { kGlObjectNone=0, kGlObjectCloud, kGlObjectCloudMesh, kGlObjectModel, kNumGlObjectType };
static const std::string GlObjectTypeNames[] = { "none", "cloud", "cloudmesh", "model" };    
    
class GlObject
{

public:

    typedef std::shared_ptr<GlObject> Ptr;
    typedef std::shared_ptr<const GlObject> ConstPtr;
    
    enum DisplayMode {kDisplayModeFaces=1, kDisplayModeWireFrame=2, kDisplayModePoints=3};        
        
public :

    GlObject(const GlObjectType& type = kGlObjectNone):color_(GlColors::White()),bEnable_(true),type_(type){}
    GlObject(const GlColor& color):color_(color),bEnable_(true){}

    virtual ~GlObject() {}
    
public: 

    static GlObject::Ptr CreateObject( const std::string& strType, const std::string& filename );

public :

    virtual void Init(){}

    virtual void Draw() = 0;
    virtual void Update(){}
    virtual void Delete(){}
    
    virtual bool Load(const std::string& filename){ UNUSED_VAR(filename); return true; }
    virtual void Save(const std::string& filename){ UNUSED_VAR(filename); }

public: // setters

    void SetEnable(bool val) { bEnable_ = val;} 
    void SetColor(const GlColor& color){ color_ = color;}
    
    virtual void SetType(const GlObjectType& type) { type_ = type; }    
    virtual void SetDisplayMode(int val);    

public: // getters 

    bool IsEnabled() const { return bEnable_; }
    
protected:
    
    void GLSetColor3f()
    {
        glColor3f(color_.r,color_.g,color_.b);
    }

    void GLSetColor3f(const GlColor& color)
    {
        glColor3f(color.r,color.g,color.b);
    }

    void GLSetColor4f()
    {
        glColor4f(color_.r,color_.g,color_.b,color_.a);
    }

    void GLSetColor4f(const GlColor& color)
    {
        glColor4f(color.r,color.g,color.b,color.a);
    }

    void GLSetColorMaterial(const GlColor& color)
    {
        GLfloat matAmbientDiffuse[4]= { color.r, color.g, color.b, 1.0 };
        glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,matAmbientDiffuse);
    }

    void GLSetColorMaterial4f(const GlColor& color)
    {
        GLfloat matAmbientDiffuse[4]= { color.r, color.g, color.b, color.a };
        glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,matAmbientDiffuse);
    }

    void GLSetColorMaterial()
    {
        GLSetColorMaterial(color_);
    }

    void GLSetShininessMaterial(float shine)
    {
        GLfloat matShininess[] = {shine};
        glMaterialfv(GL_FRONT, GL_SHININESS, matShininess);
    }

protected: 
 
    GlColor color_;
    bool bEnable_; // true/false drawn/not-drawn
    
    GlObjectType type_ = kGlObjectNone; // object type 
    
    int displayMode_ = kDisplayModeFaces;        
};


} //namespace PLVS2

#endif
