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

#include "GlObjectList.h"
#include "GlPointCloud.h"
#include "Utils.h"

namespace PLVS2
{

bool GlObjectList::Load(const std::string& fileStorageName)
{
    std::cout << std::endl;
        
    cv::FileStorage fSettings(fileStorageName, cv::FileStorage::READ);   
        
    bool bLoad = static_cast<int> (Utils::GetParam(fSettings, "GlObjects.on", 0)) != 0; 
        
    if(bLoad)
    {
        cv::FileNode objects = fSettings["GlObjects.list"];
        cv::FileNodeIterator it = objects.begin(), it_end = objects.end();
        int idx = 0;
        std::vector<double> pose;

        // iterate through a sequence using FileNodeIterator
        for( ; it != it_end; ++it, idx++ )
        {
            std::string filename = (std::string)(*it)["filename"];
            std::string strType = (std::string)(*it)["type"];
            
            pose.clear();
            (*it)["pose"] >> pose; // you can easily read numerical arrays using FileNode >> std::vector operator.
            
            std::cout << "loading object #" << idx << ": ";
            std::cout 
            << "filename=" << filename << ", "
            << "type=" << strType << ", "
            << "pose: [";
            for( int i = 0; i < (int)pose.size(); i++ ) std::cout << " " << pose[i];
            std::cout << " ]" << std::endl;
            
            GlObject::Ptr pObject = GlObject::CreateObject( strType, filename );
            this->PushFront(pObject);
            
        }        
    }
    
    std::cout << std::endl;    
    
    return true;
        
}

} //namespace PLVS2



