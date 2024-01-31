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


#ifndef VIEWER_H
#define VIEWER_H

//#include "FrameDrawer.h"
//#include "MapDrawer.h"
//#include "Tracking.h"
//#include "System.h"
#include "Settings.h"



#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>

#include "GlObjectList.h"

#include <mutex>

namespace PLVS2
{

class System;  
class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;
class PointCloudDrawer;

class Viewer
{

public:    
    static const int kUiWidth;
    
    static const std::string kMapWindowName; 
    static const std::string kFrameWindowName;
    static const float kViewpointXtopDefault;      
    static const float kViewpointYtopDefault;  
    
    enum ViewMode {kNoneView, kCameraView, kTopView, kSideView};

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const std::string &strSettingPath, Settings* settings);

    void newParameterLoader(Settings* settings);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool shouldFinished();

    bool isStopped();

    bool isStepByStep();

    void Release();

    void SetTrackingPause();

    void SetPointCloudDrawer(std::shared_ptr<PointCloudDrawer> pPointCloudDrawer) { mpPointCloudDrawer = pPointCloudDrawer; }
    
    void SetCameraCalibration(float fx, float fy, float cx, float cy);

    bool both;
private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    bool Stop();

    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im, bool bColor = true);

    System* mpSystem=nullptr;
    FrameDrawer* mpFrameDrawer=nullptr;
    MapDrawer* mpMapDrawer=nullptr;
    Tracking* mpTracker=nullptr;
    
    std::shared_ptr<PointCloudDrawer> mpPointCloudDrawer;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageScale; 
    float mImageViewerScale;
    
    int mGLviewportWidth, mGLviewportHeight;    
    int mUiWidth = kUiWidth; 
    float mfx, mfy, mcx, mcy;
    GeometricCamera* mpCamera1 = nullptr;  
    bool mbRGB; 

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    float mViewpointXside;
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

    bool mbStopTrack;

    ViewMode mViewMode = kCameraView; 
    
    pangolin::OpenGlMatrixSpec mCamP;
    std::mutex mMutexCamP;    
    
    GlObjectList mGlObjectList;
};

}


#endif // VIEWER_H
	

