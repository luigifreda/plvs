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


#include "LabelMap.h"

namespace PLVS2
{

const float LabelMap::kLabelsMatchingMinOverlaPercDefault = 0.2;
float LabelMap::skLabelsMatchingMinOverlapPerc = LabelMap::kLabelsMatchingMinOverlaPercDefault;

const float LabelMap::kLabelsMatchingMinOverlapPointsDefault = 0; 
unsigned int LabelMap::skLabelsMatchingMinOverlapPoints = LabelMap::kLabelsMatchingMinOverlapPointsDefault;
    
LabelMap::LabelMap():bAlreadyProcessed_(false)
{
    
}
    
void LabelMap::Clear()
{
    bAlreadyProcessed_ = false; 
    
    map_.clear();
    scanImgLabelsCardinality_.clear();
    scanPcLabelsCardinality_.clear();
    scanLabelBestMatch_.clear();
    scanLabelBestMatchConfidence_.clear();
    scanLabelAllMatches_.clear();
    scanPcLabelsCardinality_.clear();
}

bool LabelMap::ComputeBestMatches()
{
    if(bAlreadyProcessed_) 
    {
        // already computed 
        return false;
    }
        
    const int numScanLabels = scanImgLabelsCardinality_.size();
    
    // init label matching data
    scanLabelBestMatch_ = std::vector<int>(numScanLabels,-1);
    scanLabelBestMatchConfidence_ = std::vector<float>(numScanLabels,0);
    
    //GlobalLabelMap::MapLabelCardinality& mapLabelsCardinality = GlobalLabelMap::GetMap().GetMapLabelsCardinality();
    
    for(_LabelMap::iterator it = map_.begin(); it != map_.end(); it++)
    {
        const LabelPair& labelPair = it->first;
        const LabelType& mapLabel  = labelPair.first;
        const LabelType& scanLabel = labelPair.second;
        
        if( (scanLabel == 0) || (mapLabel == 0) ) continue; 
        
        //const int& scanLabelCardinality = scanImgLabelsCardinality_[scanLabel];
        const int& scanLabelCardinality = scanPcLabelsCardinality_[scanLabel];
        if(scanLabelCardinality>0)
        {
            const unsigned int& numMatchedPoints = it->second; 
            
            if(numMatchedPoints < skLabelsMatchingMinOverlapPoints) continue; 
                    
            const float newConfidence = float(numMatchedPoints)/scanLabelCardinality; // overlap percentage 
            float& storedConfidence = scanLabelBestMatchConfidence_[scanLabel];
            const LabelType& storedMapLabel = scanLabelBestMatch_[scanLabel];
            if(newConfidence > LabelMap::skLabelsMatchingMinOverlapPerc) 
            {
                // we have another confidence which is bigger than the given threshold 
                if( (storedMapLabel>0) && ( storedMapLabel != mapLabel) )
                {
                    if( (newConfidence > GlobalLabelMap::skLabelsMatchingMinOverlapPerc) && (storedConfidence > GlobalLabelMap::skLabelsMatchingMinOverlapPerc) )
                    {
                        GlobalLabelMap::GetMap().Update(mapLabel, storedMapLabel);
                    }
                }
                
                if(newConfidence > storedConfidence )
                {
                    storedConfidence = newConfidence; 
                    scanLabelBestMatch_[scanLabel] = mapLabel; 
                }
            } 
        }
    }
    
    map_.clear();
    bAlreadyProcessed_ = true; 
    
    return true; 
}

void LabelMap::PrintMatches()
{
    std::cout << "LabelMap::PrintMatches()" << std::endl; 
    
    //GlobalLabelMap::MapLabelCardinality& mapLabelsCardinality = GlobalLabelMap::GetMap().GetMapLabelsCardinality();
    
    for(size_t scanLabel=0, scanLabelEnd=scanLabelBestMatch_.size(); scanLabel<scanLabelEnd; scanLabel++)
    {
        int mapLabel = scanLabelBestMatch_[scanLabel];
        if( mapLabel < 0) continue; 
        int cardinality = scanPcLabelsCardinality_[scanLabel];
        std::cout << "scanLabel: " << scanLabel << ", mapLabel: " << mapLabel 
                << ", np: "<<  floor(scanLabelBestMatchConfidence_[scanLabel]*cardinality)
                << ", confidence: " << scanLabelBestMatchConfidence_[scanLabel] 
                << ", |scan seg|: " << cardinality << std::endl; 
                // << ", |map seg|: " << mapLabelsCardinality[mapLabel] << std::endl; 

    }
}

/// < < < < < <  < < < < <  < < < < <  < < < < <  < < < < <  < < < < < 

const int GlobalLabelMap::kMapLabelsAssociationMinConfidenceDefault = 3; // confidence counter 
int GlobalLabelMap::skMapLabelsAssociationMinConfidence = GlobalLabelMap::kMapLabelsAssociationMinConfidenceDefault;

const float GlobalLabelMap::kLabelsMatchingMinOverlaPercDefault = 0.2; // percentage 
float GlobalLabelMap::skLabelsMatchingMinOverlapPerc = GlobalLabelMap::kLabelsMatchingMinOverlaPercDefault;

GlobalLabelMap::GlobalLabelMap():numLabels_(0)
{
    minLabelToMerge_ = std::numeric_limits<LabelType>::max();
    maxLabelToMerge_ = std::numeric_limits<LabelType>::min();
}
        
void GlobalLabelMap::Update(LabelType mapLabel1, LabelType mapLabel2)
{
    /// <  insert an ordered pair (mapLabel1,mapLabel2) with mapLabel1 < mapLabel2
    if(mapLabel1 > mapLabel2)
    {
        std::swap(mapLabel1,mapLabel2);
    }
    
    LabelPair labelPair(mapLabel1,mapLabel2);
    
//    if(map_.count(labelPair)==0)
//    {
//        map_.insert(std::pair<LabelPair,int>(labelPair,0)); // insert with zero confidence 
//    }
//    else
//    {
//        map_[labelPair]+=1;
//    }
    _GlobalLabelMap::iterator it = map_.find(labelPair);
    if(it==map_.end())
    {
        map_.insert(std::pair<LabelPair,int>(labelPair,0)); // insert with zero confidence 
    }
    else
    {
        it->second += 1;
    }    
    
    lastEntries_.insert(labelPair);
}

void GlobalLabelMap::UpdateAll()
{                
    bool bErasedElem = false;
    
    minLabelToMerge_ = std::numeric_limits<LabelType>::max();
    maxLabelToMerge_ = std::numeric_limits<LabelType>::min();
    
    // decrease the confidences of all the pairs which were not observed 
    for(_GlobalLabelMap::iterator it = map_.begin(); it != map_.end();)
    {
        bErasedElem = false;
        const LabelPair& mapLabelPair = it->first;
        int& confidence = it->second; 
        
        // decrease all the entries which have not been seen 
        if(lastEntries_.count(mapLabelPair)==0)
        {
            // we did not see the association again 
            if(confidence>1)
            {
                confidence-= 1; 
            }
            else
            {
                it = map_.erase(it); // confidence is zero, hence we remove the element 
                bErasedElem = true; 
            }
        }
        else
        {
            // we see it again 
            if(confidence >= skMapLabelsAssociationMinConfidence)
            {
                const LabelType& mapLabel1 = mapLabelPair.first;
                const LabelType& mapLabel2 = mapLabelPair.second;
                labelMapForMerging_[mapLabel2]=mapLabel1;
                if(mapLabel2 < minLabelToMerge_)  minLabelToMerge_ = mapLabel2;
                if(mapLabel2 > maxLabelToMerge_)  maxLabelToMerge_ = mapLabel2;                
                it = map_.erase(it);
                bErasedElem = true;
            }
        }
        if(!bErasedElem) it++;
    }
    
    lastEntries_.clear();
}

void GlobalLabelMap::PrintMatches()
{
    std::cout << "GlobalLabelMap::PrintMatches()" << std::endl; 
    for(MapLabelAssociations::iterator it = labelMapForMerging_.begin(); it != labelMapForMerging_.end(); it++)
    {
        std::cout << "labels: (" << it->first << ", " << it->second << ")" << std::endl; 
    }
}

} //namespace PLVS2
