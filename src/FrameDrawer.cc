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

#include "FrameDrawer.h"
#include "Tracking.h"
#include "MapLine.h"
#include "MapObject.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

static const std::vector<cv::Scalar> kColorLinesOctave {
cv::Scalar(32.00, 250.00, 120.00), // level 0
cv::Scalar(24.00, 187.50, 90.00), // level 1
cv::Scalar(18.00, 140.62, 67.50), // level 2
cv::Scalar(13.50, 105.47, 50.62), // level 3
cv::Scalar(13.50, 105.47, 50.62), // level 4
cv::Scalar(13.50, 105.47, 50.62), // level 5
cv::Scalar(13.50, 105.47, 50.62), // level 6
cv::Scalar(13.50, 105.47, 50.62), // level 7
cv::Scalar(13.50, 105.47, 50.62), // level 8
cv::Scalar(13.50, 105.47, 50.62), // level 9
cv::Scalar(13.50, 105.47, 50.62), // level 10
cv::Scalar(13.50, 105.47, 50.62), // level 11
};
static const std::vector<cv::Scalar>  kColorPointsOctave {
cv::Scalar(32.00, 250.00, 120.00), // level 0
cv::Scalar(28.00, 218.75, 105.00), // level 1
cv::Scalar(24.50, 191.41, 91.88), // level 2
cv::Scalar(21.44, 167.48, 80.39), // level 3
cv::Scalar(18.76, 146.55, 70.34), // level 4
cv::Scalar(16.41, 128.23, 61.55), // level 5
cv::Scalar(14.36, 112.20, 53.86), // level 6
cv::Scalar(12.57, 98.17, 47.12), // level 7
cv::Scalar(12.57, 98.17, 47.12), // level 8
cv::Scalar(12.57, 98.17, 47.12), // level 9
cv::Scalar(12.57, 98.17, 47.12), // level 10
cv::Scalar(12.57, 98.17, 47.12), // level 11
};


#define SHOW_ALL_DETECTED_LINES 0
    
#define SHOW_REPROJECTED_OBJECT_CORNERS 1

namespace PLVS
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));

    mnTracked  =0;
    mnTrackedVO=0;

    mnLinesTracked  =0;
    mnLinesTrackedVO=0;

    mbUseColor = false;
    
    Nlines = 0;
    mbLineTracking = false;
    
    NObjects = 0;
    mbObjectTracking = false;
    mnObjectsTotNumInliers = 0;
    mbObjectsDetected = false;

}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state
    
    std::vector<cv::line_descriptor_c::KeyLine> vCurrentKeyLines;
    vector<bool> vbLineVO, vbLineMap; // Tracked MapLines in current frame
    
    vector<MapObjectData> vCurrentObjectsData; 

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            if(mbLineTracking)
            {
                vCurrentKeyLines = mvCurrentKeyLines;
                vbLineVO  = mvbLineVO;
                vbLineMap = mvbLineMap;
            }
            
            if(mbObjectTracking)
            {
                vCurrentObjectsData = mvCurrentObjectsData;         
            }
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;

            if(mbLineTracking)
            {
                vCurrentKeyLines = mvCurrentKeyLines;
            }
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(size_t i=0, iEnd=vMatches.size(); i<iEnd; i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked  =0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            int nOctaveP = vCurrentKeys[i].octave;

            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    //cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(kBlueComponentPointsOctaveMap[vCurrentKeys[i].octave],255,0),-1);
                    cv::circle(im,vCurrentKeys[i].pt,2,kColorPointsOctave[nOctaveP],-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }

        if(mbLineTracking)
        {
            mnLinesTracked  =0;
            mnLinesTrackedVO=0;
            const int nLines = vCurrentKeyLines.size();
            for(int i=0;i<nLines;i++)
            {
                if(vbLineVO[i] || vbLineMap[i])
                {
                    // This is a match to a MapLine in the map
                    int nOctaveL = vCurrentKeyLines[i].octave;

                    if(vbLineMap[i])
                    {
                        //cv::line(im,vCurrentKeyLines[i].getStartPoint(),vCurrentKeyLines[i].getEndPoint(),cv::Scalar(kBlueComponentLinesOctaveMap[vCurrentKeyLines[i].octave],255,0),2);
                        cv::line(im,vCurrentKeyLines[i].getStartPoint(),vCurrentKeyLines[i].getEndPoint(),kColorLinesOctave[nOctaveL],2);

                        mnLinesTracked++;
                    }
                    else // This is match to a "visual odometry" MapLine created in the last frame
                    {
                        cv::line(im,vCurrentKeyLines[i].getStartPoint(),vCurrentKeyLines[i].getEndPoint(),cv::Scalar(255,0,0),2);

                        mnLinesTrackedVO++;
                    }
                }
#if SHOW_ALL_DETECTED_LINES
                else
                {
                    // in order to show all detected lines
                    cv::line(im,vCurrentKeyLines[i].getStartPoint(),vCurrentKeyLines[i].getEndPoint(),cv::Scalar(255,0,255));
                }
#endif
            }
        }
        
        if(mbObjectTracking)
        {
            const int nObjects = vCurrentObjectsData.size();
            mbObjectsDetected = (nObjects>0);
            mnObjectsTotNumInliers = 0;
            for(int i=0;i<nObjects;i++)
            {            
                MapObjectData& data = vCurrentObjectsData[i];
                mnObjectsTotNumInliers+=data.nObjectNumInliers;
                        
#if SHOW_REPROJECTED_OBJECT_CORNERS        
                if(data.bOjectVisible)
                {
                    if( data.vObjectReprojectedCorners.size() == 4)
                    {
                        const cv::Scalar color = cv::Scalar(0,0,255);
                        cv::line(im,data.vObjectReprojectedCorners[0],data.vObjectReprojectedCorners[1],color,3);
                        cv::line(im,data.vObjectReprojectedCorners[1],data.vObjectReprojectedCorners[2],color,3);
                        cv::line(im,data.vObjectReprojectedCorners[2],data.vObjectReprojectedCorners[3],color,3);
                        cv::line(im,data.vObjectReprojectedCorners[3],data.vObjectReprojectedCorners[0],color,3);
                    }          
                }
#endif        
        
                if(data.bOjectDetected)
                {
                    if( data.vObjectDetectedCorners.size() == 4)
                    {
                        const cv::Scalar color = cv::Scalar(255,0,0);
                        cv::line(im,data.vObjectDetectedCorners[0],data.vObjectDetectedCorners[1],color,2);
                        cv::line(im,data.vObjectDetectedCorners[1],data.vObjectDetectedCorners[2],color,2);
                        cv::line(im,data.vObjectDetectedCorners[2],data.vObjectDetectedCorners[3],color,2);
                        cv::line(im,data.vObjectDetectedCorners[3],data.vObjectDetectedCorners[0],color,2);
                    }

                }
            }
        }
                
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        int nMLs = mpMap->MapLinesInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << " Pmatches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO Pmatches: " << mnTrackedVO;
        if(nMLs>0)
        {
            s << ", MLs: " << nMLs << " Lmatches: " << mnLinesTracked;
            if(mnLinesTrackedVO>0)
                s << ", + VO Lmatches: " << mnLinesTrackedVO;
        }
        if(mbObjectsDetected)
        {
            s << ", OD inliers: " << mnObjectsTotNumInliers;            
        }
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);

    if( !mbUseColor || pTracker->mImRGB.empty() )
        pTracker->mImGray.copyTo(mIm);
    else
        pTracker->mImRGB.copyTo(mIm);

    mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);

    mbLineTracking = pTracker->IsLineTracking();
    if(mbLineTracking)
    {
        mvCurrentKeyLines = pTracker->mCurrentFrame.mvKeyLines;
        Nlines = mvCurrentKeyLines.size();
        mvbLineVO = vector<bool>(Nlines,false);
        mvbLineMap = vector<bool>(Nlines,false);
    }
    
    mbObjectTracking = pTracker->IsObjectTracking();
    if(mbObjectTracking)
    {
        mvpCurrentObjects = pTracker->mCurrentFrame.mvpMapObjects; // N.B.: this returns just the ones visible or detected in the current frame
        //mvpCurrentObjects = mpMap->GetAllMapObjects();
        NObjects = mvpCurrentObjects.size();
    }

    mbOnlyTracking = pTracker->mbOnlyTracking;


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPointPtr pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }

        if(mbLineTracking)
        {
            for(int i=0;i<Nlines;i++)
            {
                MapLinePtr pML = pTracker->mCurrentFrame.mvpMapLines[i];
                if(pML)
                {
                    if(!pTracker->mCurrentFrame.mvbLineOutlier[i])
                    {
                        if(pML->Observations()>0)
                            mvbLineMap[i]=true;
                        else
                            mvbLineVO[i]=true;
                    }
                }
            }
        }
        
        if(mbObjectTracking)
        {        
            mvCurrentObjectsData.resize(NObjects);
            for(int i=0;i<NObjects;i++)
            {            
                MapObjectPtr& pMObj=mvpCurrentObjects[i];  
                MapObjectData& data = mvCurrentObjectsData[i];
                data.bOjectDetected = pMObj->IsOjectDetectedInCurrentFrame();
                data.bOjectVisible = pMObj->IsOjectVisibleInCurrentFrame();
                data.nObjectNumInliers = pMObj->GetNumInliers();
            
#if SHOW_REPROJECTED_OBJECT_CORNERS           
                if(data.bOjectVisible)
                {
                    vector<cv::Point2f> cornersReprojected = pMObj->GetCurrentImgCornersReprojected();
                    data.vObjectReprojectedCorners.resize(cornersReprojected.size());
                    // transform from Point2f to Point
                    for(size_t ii=0,iiEnd=cornersReprojected.size();ii<iiEnd;ii++)
                    {
                        data.vObjectReprojectedCorners[ii] = cv::Point(round(cornersReprojected[ii].x),round(cornersReprojected[ii].y));
                    }                
                }
#endif                 
                if(data.bOjectDetected)
                {
                    data.nObjectNumInliers = pMObj->GetNumInliers();

                    vector<cv::Point2f> corners = pMObj->GetCurrentImgCorners();
                    data.vObjectDetectedCorners.resize(corners.size());
                    // transform from Point2f to Point
                    for(size_t ii=0,iiEnd=corners.size();ii<iiEnd;ii++)
                    {
                        data.vObjectDetectedCorners[ii] = cv::Point(round(corners[ii].x),round(corners[ii].y));
                    }
                }            
            }
       
        }    
        
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace PLVS
