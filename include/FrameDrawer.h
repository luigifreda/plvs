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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>

#include<mutex>


namespace PLVS
{

class Tracking;
class Viewer;
class MapObject;

class MapObjectData
{
public:
    vector<cv::Point> vObjectDetectedCorners; 
    vector<cv::Point> vObjectReprojectedCorners;     
    bool bOjectDetected; 
    bool bOjectVisible;     
    int nObjectNumInliers; 
};

///	\class FrameDrawer
///	\author Raúl Mur-Artal
///	\brief Draw the last frame. It used in the viewer 
///	\note
///	\date
///	\warning
class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();
    
    void setUseColorImg( bool value ) { mbUseColor = value; }
    
    cv::Mat& GetFrame() { return mIm; }

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    bool mbUseColor; // use color img or bw img
    
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    
    bool mbLineTracking;
    int Nlines;
    vector<cv::line_descriptor_c::KeyLine> mvCurrentKeyLines;
    vector<bool> mvbLineMap, mvbLineVO;
    int mnLinesTracked, mnLinesTrackedVO;
    
    bool mbObjectTracking;    
    int NObjects; 
    vector<MapObjectPtr > mvpCurrentObjects;     
    vector<MapObjectData> mvCurrentObjectsData;
    int mnObjectsTotNumInliers;
    bool mbObjectsDetected;
    
    Map* mpMap;

    std::mutex mMutex;
};

} //namespace PLVS

#endif // FRAMEDRAWER_H
