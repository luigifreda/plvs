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

#ifndef POINTUTILS_H
#define POINTUTILS_H

#include "PointDefinitions.h"
#include "LabelMap.h"

#include <opencv2/core/core.hpp>
#include <pcl/kdtree/kdtree_flann.h>


namespace PLVS2
{

namespace PointUtils
{

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::depth>::value>::type* = nullptr>
inline void updateDepth(const float& depth, PointT& point)
{
    
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::depth>::value>::type* = nullptr>
inline void updateDepth(const float& depth, PointT& point)
{
    point.depth = depth; 
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline void transformPoint(const PointT& mapPointW, const cv::Mat& Rcw, const cv::Mat& tcw, PointT& mapPointC)
{
    mapPointC = mapPointW;
    
    const cv::Mat mPw     = (cv::Mat_<float>(3, 1) << mapPointW.x, mapPointW.y, mapPointW.z);

    // 3D in camera coordinates
    const cv::Mat mPc = Rcw * mPw + tcw;
    
    mapPointC.x = mPc.at<float>(0);
    mapPointC.y = mPc.at<float>(1);
    mapPointC.z = mPc.at<float>(2);
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline void transformPoint(const PointT& mapPointW, const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, PointT& mapPointC)
{
    mapPointC = mapPointW;
    
    const Eigen::Vector3f mPw(mapPointW.x, mapPointW.y, mapPointW.z);

    // 3D in camera coordinates
    const Eigen::Vector3f mPc = Rcw * mPw + tcw;
    
    mapPointC.x = mPc(0);
    mapPointC.y = mPc(1);
    mapPointC.z = mPc(2);
}


template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline void transformPoint(const PointT& mapPointW, const cv::Mat& Rcw, const cv::Mat& tcw, PointT& mapPointC)
{
    mapPointC = mapPointW;
    
    const cv::Mat mPw         = (cv::Mat_<float>(3, 1) << mapPointW.x, mapPointW.y, mapPointW.z);
    const cv::Mat normalw     = (cv::Mat_<float>(3, 1) << mapPointW.normal_x, mapPointW.normal_y, mapPointW.normal_z);

    // 3D in camera coordinates
    const cv::Mat mPc     = Rcw * mPw + tcw;
    const cv::Mat normalc = Rcw * normalw;
    
    mapPointC.x = mPc.at<float>(0);
    mapPointC.y = mPc.at<float>(1);
    mapPointC.z = mPc.at<float>(2);
    
    mapPointC.normal_x = normalc.at<float>(0);
    mapPointC.normal_y = normalc.at<float>(1);
    mapPointC.normal_z = normalc.at<float>(2);
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline void transformPoint(const PointT& mapPointW, const Eigen::Matrix3f& Rcw, const Eigen::Vector3f& tcw, PointT& mapPointC)
{
    mapPointC = mapPointW;
    
    const Eigen::Vector3f mPw(mapPointW.x, mapPointW.y, mapPointW.z);
    const Eigen::Vector3f normalw(mapPointW.normal_x, mapPointW.normal_y, mapPointW.normal_z);

    // 3D in camera coordinates
    const Eigen::Vector3f mPc     = Rcw * mPw + tcw;
    const Eigen::Vector3f normalc = Rcw * normalw;
    
    mapPointC.x = mPc(0);
    mapPointC.y = mPc(1);
    mapPointC.z = mPc(2);
    
    mapPointC.normal_x = normalc(0);
    mapPointC.normal_y = normalc(1);
    mapPointC.normal_z = normalc(2);
}

template <typename T> 
inline T pow2(const T& x)
{
    return x*x;
}

template <typename PointT> 
inline float distance (const PointT& p1, const PointT& p2)
{
    return sqrt( pow2(p1.x-p2.x) + pow2(p1.y-p2.y) + pow2(p1.z-p2.z) );
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline float normalsAngle(const PointT& p1, const PointT& p2)
{
    return 0;
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline float normalsAngle(const PointT& p1, const PointT& p2)
{
    return acos(p1.normal_x * p2.normal_x + p1.normal_y * p2.normal_y + p1.normal_z * p2.normal_z);
}


template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline float normalsDotProduct(const PointT& p1, const PointT& p2)
{
    return 1; 
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::normal_x>::value>::type* = nullptr>
inline float normalsDotProduct(const PointT& p1, const PointT& p2)
{
    return (p1.normal_x*p2.normal_x + p1.normal_y*p2.normal_y + p1.normal_z*p2.normal_z); 
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline bool isValidLabel(const PointT& point)
{
    return 1;
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline bool isValidLabel(const PointT& point)
{
    return point.label > 0;
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updateLabelMap(LabelMap& labelMap, const PointT& mapPoint, const PointT& scanPoint)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updateLabelMap(LabelMap& labelMap, const PointT& mapPoint, const PointT& scanPoint)
{            
    //std::cout << "voting match: " << mapPoint.label << ", " << scanPoint.label << std::endl; 
    labelMap.Get(mapPoint.label, scanPoint.label)+=1;     
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updateCloudFromLabelMap(const LabelMap& labelMap, typename pcl::PointCloud<PointT>::Ptr& pCloudW, typename pcl::PointCloud<PointT>::Ptr& pCloudC)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updateCloudFromLabelMap(const LabelMap& labelMap, typename pcl::PointCloud<PointT>::Ptr& pCloudW, typename pcl::PointCloud<PointT>::Ptr& pCloudC)
{
    const std::vector<int>& scanLabelAssociations = labelMap.GetScanLabelBestMatch();
    if(scanLabelAssociations.empty()) 
    {
        std::cout << "updateCloudFromLabelMap() - WARNING - associations empty!!!" << std::endl; 
        return; 
    }
    for(size_t jj=0, jjEnd=pCloudW->size(); jj<jjEnd; jj++)
    {
        uint32_t& oldLabel = pCloudW->points[jj].label;
        const int newLabel = scanLabelAssociations[pCloudW->points[jj].label];

        if(newLabel>0)
        {
            oldLabel = newLabel;
            pCloudC->points[jj].label = oldLabel;
        }
        else
        {
            if(oldLabel>0)
            {
                oldLabel += GlobalLabelMap::GetMap().GetNumLabels();
                pCloudC->points[jj].label = oldLabel;
            }
        }
    }
}

/*
template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updatePointLabelMap(GlobalLabelMap::MapLabelAssociations& map, 
                                const GlobalLabelMap::LabelType& minMapLabelToMerge,  
                                const GlobalLabelMap::LabelType& maxMapLabelToMerge,
                                GlobalLabelMap::MapLabelCardinality& mapLabelsCardinality,
                                const bool& bDoit, 
                                PointT& mapPoint)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updatePointLabelMap(GlobalLabelMap::MapLabelAssociations& map,                             
                                const GlobalLabelMap::LabelType& minMapLabelToMerge,  
                                const GlobalLabelMap::LabelType& maxMapLabelToMerge,
                                GlobalLabelMap::MapLabelCardinality& mapLabelsCardinality,                                   
                                const bool& bDoit, 
                                PointT& mapPoint)
{            
    if(!bDoit) return; 
    
    mapLabelsCardinality[mapPoint.label]++;
    
    if( (mapPoint.label < minMapLabelToMerge) || (mapPoint.label > maxMapLabelToMerge) ) return;     
    if(map.count(mapPoint.label))
    {
        mapPoint.label = map[mapPoint.label];
    }
}
*/

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updatePointLabelMap(GlobalLabelMap::MapLabelAssociations& map, 
                                const GlobalLabelMap::LabelType& minMapLabelToMerge,  
                                const GlobalLabelMap::LabelType& maxMapLabelToMerge,
                                PointT& mapPoint)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void updatePointLabelMap(GlobalLabelMap::MapLabelAssociations& map,                             
                                const GlobalLabelMap::LabelType& minMapLabelToMerge,  
                                const GlobalLabelMap::LabelType& maxMapLabelToMerge,                                
                                PointT& mapPoint)
{            
    if( (mapPoint.label < minMapLabelToMerge) || (mapPoint.label > maxMapLabelToMerge) ) return;     
    
//    if(map.count(mapPoint.label))
//    {
//        mapPoint.label = map[mapPoint.label];
//    }
    GlobalLabelMap::MapLabelAssociations::iterator it = map.find(mapPoint.label);    
    if( it != map.end()) mapPoint.label = it->second;
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr> 
inline int getLabelConfidence(const PointT& p) 
{
    return 1;
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr> 
inline int getLabelConfidence(const PointT& p) 
{
    return p.label_confidence;
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr> 
inline void printDataForLabel(const PointT& p, int label, int& cout, float data) 
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr> 
inline void printDataForLabel(const PointT& p, int label, int& count, float data) 
{
    if(p.label==label)
    {
        count++;
        std::cout << "point " << count << " " << p << std::endl;
        std::cout << "data: " << data << std::endl; 
    }
}


template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void computeScanPcLabeCardinality(const typename pcl::PointCloud<PointT>::Ptr& pCloud, const int numLabels, std::vector<unsigned int>& cardinality)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void computeScanPcLabeCardinality(const typename pcl::PointCloud<PointT>::Ptr& pCloud, const int numLabels, std::vector<unsigned int>& cardinality)
{
    cardinality = std::vector<unsigned int>(numLabels,0);
    
    for(size_t ii=0, iiEnd=pCloud->size(); ii<iiEnd; ii++)
    {
        const int& label = pCloud->points[ii].label;
        //if(label> 0) // this slows down!
        {
            cardinality[label]++;
        }
    }
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void resetIsolatedLabels(typename pcl::PointCloud<PointT>::Ptr& pCloud, const int numNeighbors, const int numEqualLabelTh)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::label>::value>::type* = nullptr>
inline void resetIsolatedLabels(typename pcl::PointCloud<PointT>::Ptr& pCloud, const int numNeighbors, const int numEqualLabelTh)
{
    if(pCloud->empty()) return; 
    
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (pCloud);
    
    std::vector<int> pointIdxNKNSearch(numNeighbors);
    std::vector<float> pointNKNSquaredDistance(numNeighbors);
    
    for (size_t i = 0; i < pCloud->points.size (); ++i)
    {
        const PointT& searchPoint = pCloud->points[i];
        uint32_t& label = pCloud->points[i].label; 
        int numEqualLabels = 0; 
        if ( kdtree.nearestKSearch (searchPoint, numNeighbors, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t jj = 0; jj < pointIdxNKNSearch.size (); ++jj)
                if(pCloud->points[pointIdxNKNSearch[jj]].label == label)
                    numEqualLabels++;
        } 
        if(numEqualLabels<numEqualLabelTh)
        {
            label = 0; 
        }
    }
}

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::kfid>::value>::type* = nullptr>
inline void setKFid(PointT& point, const int& kfid)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::kfid>::value>::type* = nullptr>
inline void setKFid(PointT& point, const int& kfid)
{
    point.kfid = kfid;
}


} //namespace PointUtils

} //namespace PLVS2


#endif /* POINTUTILS_H */

