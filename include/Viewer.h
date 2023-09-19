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


#ifndef VIEWER_H
#define VIEWER_H

#include <memory>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>

#include "GlObjectList.h"

namespace PLVS
{

class System;  
class Tracking;
class FrameDrawer;
class MapDrawer;
class PointCloudDrawer;

///	\class Viewer
///	\author Raúl Mur-Artal
///	\brief The viewer draws the map and the current camera pose. It uses Pangolin.
///	\note
///	\date
///	\warning
class Viewer
{

public:    
    static const int kUiWidth;
    
    static const std::string kMapWindowName; 
    static const std::string kFrameWindowName;
    static const float kViewpointYtopDefualt;
    
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const std::string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

  bool shouldFinished();

    bool isStopped();

    void Release();
    
    void SetPointCloudDrawer(std::shared_ptr<PointCloudDrawer> pPointCloudDrawer) { mpPointCloudDrawer = pPointCloudDrawer; }
    
    void SetCameraCalibration(float fx, float fy, float cx, float cy);
    
private:
    
    bool Stop();
    
    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im, bool bColor = true);
        
private: 
    
    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;
    
    std::shared_ptr<PointCloudDrawer> mpPointCloudDrawer;

    // 1/fps in ms
    double mT;
    int mImageWidth, mImageHeight;
    int mGLviewportWidth, mGLviewportHeight;
    int mUiWidth = kUiWidth;     
    float mfx, mfy, mcx, mcy;  
    bool mbRGB; 

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    float mViewpointYtop;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbRequestQuit;
    bool mbStopped;
    bool mbStopRequested;
    bool mbReuseMap;
    std::mutex mMutexStop;
    
    pangolin::OpenGlMatrixSpec mCamP;
    std::mutex mMutexCamP;    
    
    GlObjectList mGlObjectList;
};

}


#endif // VIEWER_H
	

