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

#ifndef POINTCLOUDMAP_TYPES_H
#define POINTCLOUDMAP_TYPES_H

#include <string>
#include <vector>

namespace PLVS2
{
    class PointCloudMapTypes
    {
    public: 
        enum PointCloudMapType
        {
            kVoxelGrid = 0,
            kOctomap,
            kOctreePoint,
            kChisel,
            kFastFusion,
            kVoxblox,
            kNumPointCloudMapType
        };
        
        static std::vector<std::string> kPointCloudMapTypeStrings;
    };
    
    
    ///	\class PointCloudMapParameters
    ///	\author Luigi Freda
    ///	\brief Abstract class for managing point cloud parameters  
    ///	\note
    ///	\date
    ///	\warning
    struct PointCloudMapParameters
    {        
        std::string pointCloudMapStringType;
        PointCloudMapTypes::PointCloudMapType pointCloudMapType;
        
        double resolution = 0.05; 

        double minDepthDistance = 10.0;  // [m]
        double maxDepthDistance = 0.01;  // [m]
        double imageDepthScale = 1.0;

        bool bUseCarving;

        int nPointCounterThreshold = 0;
        bool bRemoveUnstablePoints;
        bool bResetOnSparseMapChange = true;

        // cloud deformation based on pose graph (KFs) Adjustment         
        bool bCloudDeformationOnSparseMapChange;  

        // depth filtering 
        bool bFilterDepthImages;
        int depthFilterDiameter = 2*3+1;  // diameter of the depth filter  
        double depthFilterSigmaDepth = 0.02;  
        double depthSigmaSpace = 5; 

        // segmentation         
        bool bSegmentationOn; 
        float sementationMaxDepth = 3.0;  // [m]
        float segmentationMinFi = 0.97;    // dot product in [0,1]
        float segmentationMaxDelta = 0.02; // [m] max allowed distance of two vertices on a convex surface 
        int segmentationSingleDepthMinComponentArea = 20;
        int segmentationLineDrawThinckness;
        float segmentationMaxAngleForNormalAssociation = 20;
        float segmentationMinCosForNormalAssociation; // automatically set on the basis of the previous value segmentationMaxAngleForNormalAssociation
        int segmentationLabelConfidenceThreshold = 5;
        bool bSegmentationErosionDilationOn;  
        
        int nDownsampleStep;
    };

} //namespace PLVS2


#endif /* POINTUTILS_H */

