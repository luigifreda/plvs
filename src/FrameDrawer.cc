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

#include "FrameDrawer.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapObject.h"
#include "Atlas.h"
#include "Pointers.h"

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

namespace PLVS2
{

FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));

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

cv::Mat FrameDrawer::DrawFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state
    
    std::vector<cv::line_descriptor_c::KeyLine> vCurrentKeyLines;
    vector<bool> vbLineVO, vbLineMap; // Tracked MapLines in current frame
    
    vector<MapObjectData> vCurrentObjectsData; 

    //vector<float> vCurrentDepth;
    //float thDepth;

    Frame currentFrame;
    vector<MapPointPtr> vpLocalMapPoints;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPointPtr> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPointPtr> vpOutlierMPs;
    // map<long unsigned int, cv::Point2f> mProjectPoints;
    // map<long unsigned int, cv::Point2f> mMatchedInImage;
    
    vector<MapLinePtr> vpLocalMapLines;
    vector<cv::line_descriptor_c::KeyLine> vMatchedKeyLines;
    vector<MapLinePtr> vpMatchedMLs;
    vector<cv::line_descriptor_c::KeyLine>  vOutlierKeyLines;
    vector<MapLinePtr> vpOutlierMLs;
    // map<long unsigned int, LineEndPoints> mProjectLines;
    // map<long unsigned int, LineEndPoints> mMatchedLinesInImage;    

    cv::Scalar standardColor(0,255,0);
    cv::Scalar odometryColor(255,0,0);

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
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;

            vpLocalMapPoints = mvpLocalMapPoints;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            // mProjectPoints = mmProjectPoints;
            // mMatchedInImage = mmMatchedPointsInImage;
            
            vpLocalMapLines = mvpLocalMapLines;
            vMatchedKeyLines = mvMatchedKeyLines;
            vpMatchedMLs = mvpMatchedMLs;
            vOutlierKeyLines = mvOutlierKeyLines;
            vpOutlierMLs = mvpOutlierMLs;
            // mProjectLines = mmProjectLines;
            // mMatchedLinesInImage = mmMatchedLinesInImage;            

            //vCurrentDepth = mvCurrentDepth;
            //thDepth = mThDepth;

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

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(size_t i=0, iEnd=vMatches.size(); i<iEnd; i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,standardColor);
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        {
            cv::Point2f pt1,pt2;
            if(imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im,pt1,pt2, standardColor,5);
        }

    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            int nOctaveP = vCurrentKeys[i].octave;

            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = vCurrentKeys[i].pt;
                    pt1.x=vCurrentKeys[i].pt.x-r;
                    pt1.y=vCurrentKeys[i].pt.y-r;
                    pt2.x=vCurrentKeys[i].pt.x+r;
                    pt2.y=vCurrentKeys[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    //cv::rectangle(im,pt1,pt2,standardColor);
                    //cv::circle(im,point,2,standardColor,-1);
                    cv::circle(im,point,2,kColorPointsOctave[nOctaveP],-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    //cv::rectangle(im,pt1,pt2,odometryColor);
                    //cv::circle(im,point,2,odometryColor,-1);
		    cv::circle(im,point,2,odometryColor,-1);
                    mnTrackedVO++;
                }
            }
            /*else
            {
                cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);
            }*/
        }

        if(mbLineTracking)
        {
	    /// TODO: Luigi add line endpoints scaling 
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
                        cv::line(im,vCurrentKeyLines[i].getStartPoint()/imageScale, vCurrentKeyLines[i].getEndPoint()/imageScale, kColorLinesOctave[nOctaveL],2);

                        mnLinesTracked++;
                    }
                    else // This is match to a "visual odometry" MapLine created in the last frame
                    {
                        cv::line(im,vCurrentKeyLines[i].getStartPoint()/imageScale, vCurrentKeyLines[i].getEndPoint()/imageScale, cv::Scalar(255,0,0),2);

                        mnLinesTrackedVO++;
                    }
                }
#if SHOW_ALL_DETECTED_LINES
                else
                {
                    // in order to show all detected lines
                    cv::line(im,vCurrentKeyLines[i].getStartPoint()/imageScale, vCurrentKeyLines[i].getEndPoint()/imageScale, cv::Scalar(255,0,255));
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
                        cv::line(im,data.vObjectReprojectedCorners[0]/imageScale, data.vObjectReprojectedCorners[1]/imageScale, color,3);
                        cv::line(im,data.vObjectReprojectedCorners[1]/imageScale, data.vObjectReprojectedCorners[2]/imageScale, color,3);
                        cv::line(im,data.vObjectReprojectedCorners[2]/imageScale, data.vObjectReprojectedCorners[3]/imageScale, color,3);
                        cv::line(im,data.vObjectReprojectedCorners[3]/imageScale, data.vObjectReprojectedCorners[0]/imageScale, color,3);
                    }          
                }
#endif        
        
                if(data.bOjectDetected)
                {
                    if( data.vObjectDetectedCorners.size() == 4)
                    {
                        const cv::Scalar color = cv::Scalar(255,0,0);
                        cv::line(im,data.vObjectDetectedCorners[0]/imageScale, data.vObjectDetectedCorners[1]/imageScale, color,2);
                        cv::line(im,data.vObjectDetectedCorners[1]/imageScale, data.vObjectDetectedCorners[2]/imageScale, color,2);
                        cv::line(im,data.vObjectDetectedCorners[2]/imageScale, data.vObjectDetectedCorners[3]/imageScale, color,2);
                        cv::line(im,data.vObjectDetectedCorners[3]/imageScale, data.vObjectDetectedCorners[0]/imageScale, color,2);
                    }

                }
            }
        }
                
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    std::vector<cv::line_descriptor_c::KeyLine> vCurrentKeyLines;
    vector<bool> vbLineVO, vbLineMap; // Tracked MapLines in current frame

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;

            if(mbLineTracking)
            {
                vCurrentKeyLines = mvCurrentKeyLinesRight;
                vbLineVO  = mvbLineVO;
                vbLineMap = mvbLineMap;
            }            
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;

            if(mbLineTracking)
            {
                vCurrentKeyLines = mvCurrentKeyLinesRight;
            }            
        }
    } // destroy scoped mutex -> release mutex

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }

                cv::line(im,pt1,pt2,cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            int nOctaveP = mvCurrentKeysRight[i].octave;
                        
            if(vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvCurrentKeysRight[i].pt / imageScale;
                    float px = mvCurrentKeysRight[i].pt.x / imageScale;
                    float py = mvCurrentKeysRight[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvCurrentKeysRight[i].pt;
                    pt1.x=mvCurrentKeysRight[i].pt.x-r;
                    pt1.y=mvCurrentKeysRight[i].pt.y-r;
                    pt2.x=mvCurrentKeysRight[i].pt.x+r;
                    pt2.y=mvCurrentKeysRight[i].pt.y+r;
                }

                // This is a match to a MapPoint in the map
                if(vbMap[i + Nleft])
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    //cv::circle(im,point,2,cv::Scalar(0,255,0),-1);
                    cv::circle(im,point,2,kColorPointsOctave[nOctaveP],-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    //cv::circle(im,point,2,cv::Scalar(255,0,0),-1);
                    cv::circle(im,point,2,kColorPointsOctave[nOctaveP],-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    if(mbLineTracking && (state==Tracking::OK))
    {
        mnLinesTracked  =0;
        mnLinesTrackedVO=0;
        const int nLines = mvCurrentKeyLinesRight.size();
        const int nLinesLeft = mvCurrentKeyLines.size();        
        for(int i=0;i<nLines;i++)
        {
            if(vbLineVO[i + nLinesLeft] || vbLineMap[i + nLinesLeft])
            {
                // This is a match to a MapLine in the map
                int nOctaveL = mvCurrentKeyLinesRight[i].octave;

                if(vbLineMap[i + nLinesLeft])
                {
                    cv::line(im,mvCurrentKeyLinesRight[i].getStartPoint()/imageScale, mvCurrentKeyLinesRight[i].getEndPoint()/imageScale, kColorLinesOctave[nOctaveL],2);

                    mnLinesTracked++;
                }
                else // This is match to a "visual odometry" MapLine created in the last frame
                {
                    cv::line(im,mvCurrentKeyLinesRight[i].getStartPoint()/imageScale, mvCurrentKeyLinesRight[i].getEndPoint()/imageScale, cv::Scalar(255,0,0),2);

                    mnLinesTrackedVO++;
                }
            }
#if SHOW_ALL_DETECTED_LINES
            else
            {
                // in order to show all detected lines
                cv::line(im,mvCurrentKeyLinesRight[i].getStartPoint()/imageScale, mvCurrentKeyLinesRight[i].getEndPoint()/imageScale, cv::Scalar(255,0,255));
            }
#endif
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
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        int nMLs = mpAtlas->MapLinesInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", PMatches: " << mnTracked;
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

    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    //mThDepth = pTracker->mCurrentFrame.mThDepth;
    //mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

    if(both)
    {
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else
    {
        N = mvCurrentKeys.size();
    }

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);

    mbLineTracking = pTracker->IsLineTracking();
    if(mbLineTracking)
    {
        mvCurrentKeyLines = pTracker->mCurrentFrame.mvKeyLines;
        if(both)
        {
            mvCurrentKeyLinesRight = pTracker->mCurrentFrame.mvKeyLinesRight;
            Nlines = mvCurrentKeyLines.size() + mvCurrentKeyLinesRight.size();
        }
        else
        {
            Nlines = mvCurrentKeyLines.size();
        }
        
        mvbLineVO = vector<bool>(Nlines,false);
        mvbLineMap = vector<bool>(Nlines,false);
    }
    
    mbObjectTracking = pTracker->IsObjectTracking();
    if(mbObjectTracking)
    {
        mvpCurrentObjects = pTracker->mCurrentFrame.mvpMapObjects; // N.B.: this returns just the ones visible or detected in the current frame
        //mvpCurrentObjects = mpAtlas->GetAllMapObjects();
        NObjects = mvpCurrentObjects.size();
    }

    mbOnlyTracking = pTracker->mbOnlyTracking;

    //Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    // mmProjectPoints = mCurrentFrame.mmProjectPoints;
    // mmMatchedPointsInImage.clear();
    
    // mmProjectLines = mCurrentFrame.mmProjectLines;
    // mmMatchedLinesInImage.clear();    

    mvpLocalMapPoints = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);
    
    if(mbLineTracking)
    {
        mvpLocalMapLines = pTracker->GetLocalMapMLS();
        
        mvMatchedKeyLines.clear();
        mvMatchedKeyLines.reserve(Nlines);
        mvpMatchedMLs.clear();
        mvpMatchedMLs.reserve(Nlines);
        mvOutlierKeyLines.clear();
        mvOutlierKeyLines.reserve(Nlines);
        mvpOutlierMLs.clear();
        mvpOutlierMLs.reserve(Nlines);    
    }

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

                    //mmMatchedPointsInImage[pMP->mnId] = mvCurrentKeys[i].pt;

                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    mvOutlierKeys.push_back(mvCurrentKeys[i]);
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
                        
                        // mmMatchedLinesInImage[pML->mnId] = std::make_pair(cv::Point2f(mvCurrentKeyLines[i].startPointX,mvCurrentKeyLines[i].startPointY),
                        //                                                   cv::Point2f(mvCurrentKeyLines[i].startPointX,mvCurrentKeyLines[i].startPointY));
                    }
                    else
                    {
                        mvpOutlierMLs.push_back(pML);
                        mvOutlierKeyLines.push_back(mvCurrentKeyLines[i]);
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

} // namespace PLVS2
