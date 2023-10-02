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

#include <opencv2/core/core.hpp>

#include "GlObject.h"
#include "GlPointCloud.h"
#include "Utils.h"
#include "PointDefinitions.h"


typedef POINT_TYPE PointT;

namespace PLVS2
{

GlObject::Ptr GlObject::CreateObject( const std::string& strType, const std::string& filename )
{
    GlObjectType type = kGlObjectNone;
    for(size_t ii=0; ii<(kNumGlObjectType-1); ii++)
    {
        if( strType == GlObjectTypeNames[ii] ) 
        {
            type = static_cast<GlObjectType>(ii);
            break;
        }
    }
    
    //std::cout << filename << ": " << GlObjectTypeNames[(int)type] << std::endl;
    
    GlObject::Ptr pObject; 
    switch(type)
    {
    case kGlObjectCloud:
        pObject = std::make_shared<GlPointCloud<PointT> >();
        pObject->Load(filename);
        break; 
    case kGlObjectCloudMesh:
        pObject = std::make_shared<GlPointCloud<PointT> >();
        pObject->Load(filename);
        pObject->SetType(kGlObjectCloudMesh);
        break; 
    case kGlObjectModel:
        break; 
        
    default:
    case kGlObjectNone:
        break;         
    }
    
    return pObject;
}

void GlObject::SetDisplayMode(int val)
{
    displayMode_ = val;
    switch (displayMode_)
    {
    case kDisplayModeFaces:
        break;
    case kDisplayModeWireFrame:
        break;
    case kDisplayModePoints:
        break;
    default:
        displayMode_ = (int) kDisplayModePoints;
    }
}

} //namespace PLVS2



