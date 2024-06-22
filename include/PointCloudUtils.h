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

#ifndef POIN_CLOUD_TUTILS_H
#define POIN_CLOUD_TUTILS_H

#include "PointDefinitions.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace PLVS2
{

namespace PointCloudUtils
{


// C++17 helpers for checking if 'x','y','z' are fields in 'PointT'.
#define DEFINE_HAS_MEMBER(name, member)                                                                                \
  template <typename T, typename = void>                                                                               \
  struct Has##name##Member : std::false_type                                                                           \
  {                                                                                                                    \
  };                                                                                                                   \
  template <typename T>                                                                                                \
  struct Has##name##Member<T, std::void_t<decltype(T::member)>> : std::true_type                                       \
  {                                                                                                                    \
  };

DEFINE_HAS_MEMBER(X, x)
DEFINE_HAS_MEMBER(Y, y)
DEFINE_HAS_MEMBER(Z, z)

DEFINE_HAS_MEMBER(NormalX, normal_x)
DEFINE_HAS_MEMBER(NormalY, normal_y)
DEFINE_HAS_MEMBER(NormalZ, normal_z)

DEFINE_HAS_MEMBER(Rgba, rgba)

DEFINE_HAS_MEMBER(Kfid, kfid)
DEFINE_HAS_MEMBER(Depth, depth)

DEFINE_HAS_MEMBER(Label, label)
DEFINE_HAS_MEMBER(LabelConfidence, label_confidence)


template <typename PointT>
constexpr bool is3dPointType()
{
  if constexpr (HasXMember<PointT>::value && HasYMember<PointT>::value && HasZMember<PointT>::value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
constexpr bool has3dNormalFields()
{
  if constexpr (HasNormalXMember<PointT>::value && HasNormalYMember<PointT>::value && HasNormalZMember<PointT>::value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
constexpr bool hasRgbaFields()
{
  if constexpr (HasRgbaMember<PointT>::value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
constexpr bool hasKfidField()
{
  if constexpr (HasKfidMember<PointT>::value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
constexpr bool hasDepthField()
{
  if constexpr (HasDepthMember<PointT>::value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
constexpr bool hasLabelField()
{
  if constexpr (HasLabelMember<PointT>::value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
constexpr bool hasLabelConfidenceField()
{
  if constexpr (HasLabelConfidenceMember<PointT>::value)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointIn, typename PointOut>
typename pcl::PointCloud<PointOut>::Ptr copyCloud(const typename pcl::PointCloud<PointIn>::Ptr& cloud)
{
    typename pcl::PointCloud<PointOut>::Ptr copy(new pcl::PointCloud<PointOut>());
    copy->reserve(cloud->size());
    copy->header = cloud->header;
    copy->is_dense = cloud->is_dense;

    for (const PointIn& p : cloud->points)
    {
        PointOut pout;
        pout.x = p.x;
        pout.y = p.y;
        pout.z = p.z;
        if constexpr (has3dNormalFields<PointOut>() && has3dNormalFields<PointIn>())
        {
            pout.normal_x = p.normal_x;
            pout.normal_y = p.normal_y;
            pout.normal_z = p.normal_z;
        }
        if constexpr (hasRgbaFields<PointOut>() && hasRgbaFields<PointIn>())
        {
            pout.rgba = p.rgba;
        }
        if constexpr (hasKfidField<PointOut>() && hasKfidField<PointIn>())
        {
            pout.kfid = p.kfid;
        } 
        if constexpr (hasDepthField<PointOut>() && hasDepthField<PointIn>())
        {
            pout.depth = p.depth;
        }     
        if constexpr (hasLabelField<PointOut>() && hasLabelField<PointIn>())
        {
            pout.label = p.label;
        }      
        if constexpr (hasLabelConfidenceField<PointOut>() && hasLabelConfidenceField<PointIn>())
        {
            pout.label_confidence = p.label_confidence;
        }         
        copy->push_back(pout);
    }
  return copy;
}


template <typename PointIn, typename PointOut, typename Transform, typename Scalar>
typename pcl::PointCloud<PointOut>::Ptr transformCloud(const typename pcl::PointCloud<PointIn>::Ptr& cloud,
                                                        const Transform& transform)
{
    typename pcl::PointCloud<PointOut>::Ptr transformedCloud(new pcl::PointCloud<PointOut>());
    transformedCloud->reserve(cloud->size());
    transformedCloud->header = cloud->header; 
    transformedCloud->is_dense = transformedCloud->is_dense;

    for (const PointIn& p : cloud->points)
    {
        Eigen::Matrix<Scalar,3,1> point(p.x, p.y, p.z);
        const auto pointOut = transform * point;
        PointOut pout;
        pout.x = pointOut.x();
        pout.y = pointOut.y();
        pout.z = pointOut.z();
        if constexpr (has3dNormalFields<PointOut>() && has3dNormalFields<PointIn>())
        {
            const Eigen::Matrix<Scalar,3,1> normal(p.normal_x, p.normal_y, p.normal_z);
            const auto normalOut = transform * normal;
            pout.normal_x = normalOut.x();
            pout.normal_y = normalOut.y();
            pout.normal_z = normalOut.z();
        }
        if constexpr (hasRgbaFields<PointOut>() && hasRgbaFields<PointIn>())
        {
            pout.rgba = p.rgba;
        }
        if constexpr (hasKfidField<PointOut>() && hasKfidField<PointIn>())
        {
            pout.kfid = p.kfid;
        } 
        if constexpr (hasDepthField<PointOut>() && hasDepthField<PointIn>())
        {
            pout.depth = p.depth;
        }     
        if constexpr (hasLabelField<PointOut>() && hasLabelField<PointIn>())
        {
            pout.label = p.label;
        }      
        if constexpr (hasLabelConfidenceField<PointOut>() && hasLabelConfidenceField<PointIn>())
        {
            pout.label_confidence = p.label_confidence;
        }             
        transformedCloud->push_back(pout);
    }
    return transformedCloud;
}


template <typename PointIn, typename PointOut, typename Transform, typename Scalar>
void transformCloud(const typename pcl::PointCloud<PointIn>& cloud, 
                    typename pcl::PointCloud<PointOut>& transformedCloud,
                    const Transform& transform)
{
    transformedCloud.reserve(cloud.size());
    transformedCloud.header = cloud.header; 
    transformedCloud.is_dense = cloud.is_dense;     

    for (const PointIn& p : cloud.points)
    {
        Eigen::Matrix<Scalar,3,1> point(p.x, p.y, p.z);
        const auto pointOut = transform * point;
        PointOut pout;
        pout.x = pointOut.x();
        pout.y = pointOut.y();
        pout.z = pointOut.z();
        if constexpr (has3dNormalFields<PointOut>() && has3dNormalFields<PointIn>())
        {
            const Eigen::Matrix<Scalar,3,1> normal(p.normal_x, p.normal_y, p.normal_z);
            const auto normalOut = transform * normal;
            pout.normal_x = normalOut.x();
            pout.normal_y = normalOut.y();
            pout.normal_z = normalOut.z();
        }
        if constexpr (hasRgbaFields<PointOut>() && hasRgbaFields<PointIn>())
        {
            pout.rgba = p.rgba;
        }
        if constexpr (hasKfidField<PointOut>() && hasKfidField<PointIn>())
        {
            pout.kfid = p.kfid;
        } 
        if constexpr (hasDepthField<PointOut>() && hasDepthField<PointIn>())
        {
            pout.depth = p.depth;
        }     
        if constexpr (hasLabelField<PointOut>() && hasLabelField<PointIn>())
        {
            pout.label = p.label;
        }      
        if constexpr (hasLabelConfidenceField<PointOut>() && hasLabelConfidenceField<PointIn>())
        {
            pout.label_confidence = p.label_confidence;
        }             
        transformedCloud.push_back(pout);
    }
}

} //namespace PointCloudUtils

} //namespace PLVS2


#endif /* POINTUTILS_H */

