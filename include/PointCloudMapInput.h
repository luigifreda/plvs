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

#ifndef POINTCLOUDMAPINPUT_H
#define POINTCLOUDMAPINPUT_H

#include <mutex>

#include "PointDefinitions.h"
#include "PointCloudKeyFrame.h"

#include <pcl/common/transforms.h>



namespace PLVS2
{


///	\class PointCloudMapInput
///	\author Luigi Freda
///	\brief A class for managing the input of a PointCloudMap object 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapInput
{
public:
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    
    typedef PointCloudKeyFrame<PointT> PointCloudKeyFrameT;
    
    typedef std::shared_ptr<PointCloudMapInput<PointT> > Ptr;
    typedef std::shared_ptr<const PointCloudMapInput<PointT> > ConstPtr;
    
    enum InputType {kPointCloud, kDepthImage, kPointCloudAndDepthImage, kColorAndDepthImages};
        
public:
  
    PointCloudMapInput(typename PointCloudKeyFrameT::Ptr pPointCloudKeyFrame_in,
                       const double& minRange_in,
                       const double& maxRange_in,
                       const double& imageDepthScale_in,
                       const boost::uint64_t timestamp_in)
    :pPointCloudKeyFrame(pPointCloudKeyFrame_in),
     minRange(minRange_in),maxRange(maxRange_in),imageDepthScale(imageDepthScale_in),timestamp(timestamp_in),type(kPointCloud){}
        
public:        

    typename PointCloudKeyFrameT::Ptr  pPointCloudKeyFrame; 
    
    const double minRange;
    const double maxRange;
    
    const double imageDepthScale;
    
    const boost::uint64_t timestamp;
        
    InputType type;
};


} //namespace PLVS2

#endif /* POINTCLOUDMAPINPUT_H */

