/*
 * This file is part of PLVS.

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

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <set>
#include <unordered_set>

#include <boost/functional/hash.hpp>

#ifndef LABELMAP_H
#define LABELMAP_H

namespace PLVS2
{

class GlobalLabelMap; 


class LabelMap
{
        
public: 
    
    static const float kLabelsMatchingMinOverlaPercDefault; 
    static float skLabelsMatchingMinOverlapPerc;
    static void SetLabelsAssociationMinOverlapPerc(float val) {skLabelsMatchingMinOverlapPerc = val; }
    
    static const float kLabelsMatchingMinOverlapPointsDefault; 
    static unsigned int skLabelsMatchingMinOverlapPoints;
    static void SetLabelsAssociationMinNum(float val) {skLabelsMatchingMinOverlapPoints = val; }
     
public: 
    
    struct PairHash 
    {
        template <class T1, class T2>
        std::size_t operator () (const std::pair<T1,T2> &p) const 
        {
            std::size_t seed = 0;
            boost::hash_combine(seed, p.first);
            boost::hash_combine(seed, p.second);
            return seed;
        }
    };
    
    typedef unsigned int LabelType; 
    typedef std::pair<LabelType,LabelType> LabelPair; // <map label, scan label>
    //typedef std::pair<LabelType, unsigned int> LabelMatch;
    typedef std::unordered_map<LabelType/*key */, float /*val num points matched*/> LabelMatchMap;
    typedef std::unordered_map<LabelPair/*key */, unsigned int /*val num points matched*/, PairHash> _LabelMap;
    
public:
    
    LabelMap();

    void Clear();
    
    bool ComputeBestMatches();
    
    void PrintMatches(); 
    
public: 
    
    bool IsAlreadyProcessed() const { return bAlreadyProcessed_; }
    
    unsigned int& Get(const LabelType& mapLabel, const LabelType& scanLabel)
    {
        return map_[std::make_pair(mapLabel,scanLabel)];
    }    
    
    unsigned int& operator[] ( const LabelPair& labelPair )
    {
        return map_[labelPair];
    }
    
    std::vector<int>& GetScanLabelBestMatch() { return scanLabelBestMatch_; }
    const std::vector<int>& GetScanLabelBestMatch() const { return scanLabelBestMatch_; }
    
    //std::vector<float>& GetScanLabelBestMatchConfidence() { return scanLabelBestMatchConfidence_; }
    //const std::vector<float>& GetScanLabelBestMatchConfidence() const { return scanLabelBestMatchConfidence_; }
    
    std::vector<unsigned int>& GetScanImgLabelsCardinality() {return scanImgLabelsCardinality_; }
    std::vector<unsigned int>& GetScanPCLabelsCardinality() {return scanPcLabelsCardinality_; }

    size_t GetNumLabels() const { return scanImgLabelsCardinality_.size(); }
    
protected:

    _LabelMap map_; 
    
    std::vector<unsigned int> scanImgLabelsCardinality_; // number of points per scanlabel on the image
    std::vector<unsigned int> scanPcLabelsCardinality_; // number of points per scanlabel in the point cloud
    
    std::vector<int> scanLabelBestMatch_; // scanLabelMatch_[scanlabel]= maplabel ; -1 if invalid   <- best label associations
    std::vector<float> scanLabelBestMatchConfidence_; // scanLabelMatchConfidence_[scanlabel]= confidence    
    
    
    std::vector<LabelMatchMap> scanLabelAllMatches_; 
    
    bool bAlreadyProcessed_; 
};




class GlobalLabelMap
{
        
public:
    
    static const int kMapLabelsAssociationMinConfidenceDefault; 
    static int skMapLabelsAssociationMinConfidence;
    static void SetMapLabelsAssociationMinConfidence(int val) {skMapLabelsAssociationMinConfidence = val; }
        
    static const float kLabelsMatchingMinOverlaPercDefault; 
    static float skLabelsMatchingMinOverlapPerc;
    static void SetLabelsAssociationMinOverlapPerc(float val) {skLabelsMatchingMinOverlapPerc = val; }
    
public: 
        
    typedef LabelMap::LabelType LabelType;
    typedef LabelMap::LabelPair LabelPair; 
    typedef std::unordered_map<LabelPair/*key */, int /*val confidence counter*/, LabelMap::PairHash> _GlobalLabelMap;
    
    typedef std::unordered_map<LabelType,LabelType> MapLabelAssociations;
    typedef std::unordered_map<LabelType,size_t> MapLabelCardinality;
        
public:
    
    static GlobalLabelMap & GetMap()
    {
        static GlobalLabelMap instance;
        return instance;
    }
        
public:
    
    GlobalLabelMap();

    void Clear();
        
    void Update(LabelType mapLabel1, LabelType mapLabel2);
    
    void UpdateAll(); 
    
    void PrintMatches(); 
    
public: 
    
    void SetNumLabels(const size_t& val) { numLabels_ = val; }
    
    void AddNumLabels(const size_t& val) {numLabels_+=val;}
    
public: 
    
    int& Get(const LabelType& mapLabel1, const LabelType& mapLabel2)
    {
        return map_[std::make_pair(mapLabel1,mapLabel2)];
    }    
    
    int& operator[] ( const LabelPair& labelPair )
    {
        return map_[labelPair];
    }
    
    const size_t& GetNumLabels() const {return numLabels_; }
    
    MapLabelAssociations& GetLabelMapForMerging() { return labelMapForMerging_; } 
    const LabelType& GetMinLabelToMerge() const { return minLabelToMerge_; }
    const LabelType& GetMaxLabelToMerge() const { return maxLabelToMerge_; }
    
    MapLabelCardinality& GetMapLabelsCardinality() { return mapLabelsCardinality_; }
    
protected:

    _GlobalLabelMap map_; 
    
    std::unordered_set<LabelPair, LabelMap::PairHash> lastEntries_;
    
    size_t numLabels_; 
    
    MapLabelAssociations labelMapForMerging_; 
    LabelType minLabelToMerge_; 
    LabelType maxLabelToMerge_; 
    
    MapLabelCardinality mapLabelsCardinality_; 
    
};




} //namespace PLVS2


#endif /* LABELMAP_H */

