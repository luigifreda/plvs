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
/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "Pointers.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include "BoostArchiver.h"

#include<mutex>


namespace PLVS2
{

class KeyFrame;
class Frame;
class Map;


class KeyFrameDatabase
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDatabase(){}
    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(const KeyFramePtr& pKF);

   void erase(const KeyFramePtr& pKF);

   void clear();
   void clearMap(Map* pMap);

   // Loop Detection(DEPRECATED)
   std::vector<KeyFramePtr> DetectLoopCandidates(KeyFramePtr& pKF, float minScore);

   // Loop and Merge Detection
   void DetectCandidates(KeyFramePtr pKF, float minScore,vector<KeyFramePtr>& vpLoopCand, vector<KeyFramePtr>& vpMergeCand);
   void DetectBestCandidates(KeyFramePtr pKF, vector<KeyFramePtr> &vpLoopCand, vector<KeyFramePtr> &vpMergeCand, int nMinWords);
   void DetectNBestCandidates(KeyFramePtr pKF, vector<KeyFramePtr> &vpLoopCand, vector<KeyFramePtr> &vpMergeCand, int nNumCandidates);

   // Relocalization
   std::vector<KeyFramePtr> DetectRelocalizationCandidates(Frame* F, Map* pMap);

   void PreSave();
   void PostLoad(map<long unsigned int, KeyFramePtr> mpKFid);
   void SetORBVocabulary(ORBVocabulary* pORBVoc, bool clearInvertedFile=true);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFramePtr> > mvInvertedFile; // for each BOW word we get the list of KFs sharing it 

  // Mutex
  std::mutex mMutex;
};

} // namespace PLVS2

#endif
