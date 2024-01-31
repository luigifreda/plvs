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


#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

//#include "Tracking.h"
//#include "MapPoint.h"
//#include "Atlas.h"
#include "Frame.h"
#include "Pointers.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>

#include<mutex>
#include <unordered_set>


namespace PLVS2
{

class Tracking;
class Viewer;
class MapObject;
class Atlas; 

class MapObjectData
{
public:
    std::vector<cv::Point> vObjectDetectedCorners; 
    std::vector<cv::Point> vObjectReprojectedCorners;     
    bool bOjectDetected; 
    bool bOjectVisible;     
    int nObjectNumInliers; 
};

class FrameDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameDrawer(Atlas* pAtlas);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame(float imageScale=1.f);
    cv::Mat DrawRightFrame(float imageScale=1.f);

    
    void setUseColorImg( bool value ) { mbUseColor = value; }
    
    cv::Mat& GetFrame() { return mIm; }

    bool both;

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm, mImRight;
    bool mbUseColor; // use color img or bw img

    int N;
    std::vector<cv::KeyPoint> mvCurrentKeys,mvCurrentKeysRight;
    std::vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    std::vector<cv::KeyPoint> mvIniKeys;
    std::vector<int> mvIniMatches;
    int mState;
    
    bool mbLineTracking;
    int Nlines;
    std::vector<cv::line_descriptor_c::KeyLine> mvCurrentKeyLines, mvCurrentKeyLinesRight;
    std::vector<bool> mvbLineMap, mvbLineVO;
    int mnLinesTracked, mnLinesTrackedVO;
    
    bool mbObjectTracking;    
    int NObjects; 
    std::vector<MapObjectPtr > mvpCurrentObjects;     
    std::vector<MapObjectData> mvCurrentObjectsData;
    int mnObjectsTotNumInliers;
    bool mbObjectsDetected;

    //std::vector<float> mvCurrentDepth;
    //float mThDepth;

    Atlas* mpAtlas;

    std::mutex mMutex;

    std::vector<pair<cv::Point2f, cv::Point2f> > mvTracks;

    Frame mCurrentFrame;
    std::vector<MapPointPtr> mvpLocalMapPoints;
    std::vector<cv::KeyPoint> mvMatchedKeys;
    std::vector<MapPointPtr> mvpMatchedMPs;
    std::vector<cv::KeyPoint> mvOutlierKeys;
    std::vector<MapPointPtr> mvpOutlierMPs;
    
    std::vector<MapLinePtr> mvpLocalMapLines;
    std::vector<cv::line_descriptor_c::KeyLine> mvMatchedKeyLines;
    std::vector<MapLinePtr> mvpMatchedMLs;
    std::vector<cv::line_descriptor_c::KeyLine>  mvOutlierKeyLines;
    std::vector<MapLinePtr> mvpOutlierMLs;

    // std::map<long unsigned int, cv::Point2f> mmProjectPoints;
    // std::map<long unsigned int, cv::Point2f> mmMatchedPointsInImage;
    
    // typedef Frame::LineEndPoints LineEndPoints;  
    // std::map<long unsigned int, LineEndPoints> mmProjectLines;
    // std::map<long unsigned int, LineEndPoints> mmMatchedLinesInImage;

};

} // namespace PLVS2

#endif // FRAMEDRAWER_H
