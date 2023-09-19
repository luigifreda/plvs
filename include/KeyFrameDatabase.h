/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
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

#include<mutex>
#include "BoostArchiver.h"


namespace PLVS
{

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(const KeyFramePtr& pKF);

   void erase(const KeyFramePtr& pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFramePtr> DetectLoopCandidates(KeyFramePtr& pKF, float minScore);

   // Relocalization
   std::vector<KeyFramePtr> DetectRelocalizationCandidates(Frame* F);

public:
   // for serialization
   KeyFrameDatabase() {}
   void SetORBvocabulary(ORBVocabulary *pOrbv) {mpVoc=pOrbv;}
private:
   // serialize is recommended to be private
   friend class boost::serialization::access;
   template<class Archive>
   void serialize(Archive &ar, const unsigned int version);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFramePtr> > mvInvertedFile; // wordId -> keyframe

  // Mutex
  std::mutex mMutex;
};

} //namespace PLVS

#endif
