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


#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <mutex>

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>
#include <pangolin/gl/glplatform.h>
#include <pangolin/gl/glfont.h>
#include <GLFW/glfw3.h>

#ifdef HAVE_OPENNI
#undef HAVE_OPENNI
#endif
#ifdef HAVE_OPENNI2
#undef HAVE_OPENNI2
#endif

#include "PointCloudDrawer.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Shaders.h"
#include "MapObject.h"


#define REAL_TIME_MODE 0  // more computational demanding (since cv::waitKey() is not as accurate) ... it tries to work at camera FPS

namespace PLVS
{

const int Viewer::kUiWidth = 180;

const std::string Viewer::kMapWindowName  = "PLVS: Map Viewer";
const std::string Viewer::kFrameWindowName = "PLVS: Current Frame";

const float Viewer::kViewpointYtopDefualt = -4.0; // [m] height from which the eye sees the map from the top


Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false),
mbReuseMap(false), mbRequestQuit(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if (fps < 1)
        fps = 30;
    mT = 1e3 / fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if (mImageWidth < 1 || mImageHeight < 1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }
    
    mfx = fSettings["Camera.fx"];
    mfy = fSettings["Camera.fy"];
    mcx = fSettings["Camera.cx"];
    mcy = fSettings["Camera.cy"];

    mbRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);

    mGLviewportWidth = fSettings["Viewer.GLwidth"];
    mGLviewportHeight = fSettings["Viewer.GLheight"];
    if (mGLviewportWidth < 1 || mGLviewportHeight < 1)
    {
        mGLviewportWidth = 1024;//1280;
        mGLviewportHeight = 768;//720;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];

    mViewpointYtop = Utils::GetParam(fSettings, "Viewer.ViewpointYtop", kViewpointYtopDefualt);
    
    mGlObjectList.Load(strSettingPath);
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    bool bLinesActive = mpTracker->IsLineTracking();
    bool bObjectTracking = mpTracker->IsObjectTracking();

    const cv::Mat& K = mpTracker->getMatK();
    const cv::Mat& DistCoef = mpTracker->getMatDistCoef();

    glfwInit();
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    int screenWidth = mode->width;
    int screenHeight = mode->height;
    std::cout << "screen resolution [" << screenWidth << ", " << screenHeight << "] " << std::endl;  
    if(screenWidth >= 3840 &&  screenHeight >= 2400)
    {
        const float scale = 1.8; 
        mGLviewportWidth *= scale; 
        mGLviewportHeight *= scale; 
        mUiWidth *= scale; 
    } 

    pangolin::CreateWindowAndBind(kMapWindowName, mGLviewportWidth+mUiWidth, mGLviewportHeight);

#if COMPUTE_NORMALS
    std::shared_ptr<Shader> normalsProgram;
    normalsProgram = std::shared_ptr<Shader>(loadProgramFromFile("normals.vert", "normals.frag", "normals.geom"));
    if (mpPointCloudDrawer)
    {
        mpPointCloudDrawer->SetNormalsProgram(normalsProgram);
    }

    if(screenWidth >= 3840 &&  screenHeight >= 2400)
    {
        // can do this only after a pangolin context has been created 
        pangolin::set_font_size(30);
    }

    // Choose a sensible left UI Panel width based on the width of 20
    // charectors from the default font.
    mUiWidth = 20* pangolin::default_font().MaxWidth();

#if COMPUTE_SEGMENTS
    std::vector<Image4Viewer>* pVecImages = 0;
    if (mpPointCloudDrawer)
    {
        pVecImages = &(mpPointCloudDrawer->GetPointCloudMapping()->GetVecImages());
    }

    std::shared_ptr<Shader> segmentsProgram;
    segmentsProgram = std::shared_ptr<Shader>(loadProgramFromFile("segments.vert", "segments.frag", "segments.geom"));
    if (mpPointCloudDrawer)
    {
        mpPointCloudDrawer->SetSegmentsProgram(segmentsProgram);
    }

//    std::shared_ptr<Shader> debugTextProgram;
//    debugTextProgram = std::shared_ptr<Shader>(loadProgramFromFile("text.vert", "text.frag"));
//    GLTexture glTexture;
//    glTexture.InitGLTexture(getShadersDir()+"/fonts.png");
//    if (mpPointCloudDrawer)
//    {
//        mpPointCloudDrawer->SetDebugProgram(debugTextProgram);
//        mpPointCloudDrawer->SetDebugFontTextureId(glTexture.GetTextureId());
//    }

#endif

#endif // end if COMPUTE_NORMALS

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(mUiWidth));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuTopCamera("menu.Top Camera", false, true);
    pangolin::Var<bool> menuARCamera("menu.AR Camera", false, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuShowLines("menu.Show Lines", true, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
    pangolin::Var<bool> menuShowPointCloud("menu.Show Point Cloud", true, true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
    pangolin::Var<bool> menuDisplayUnstable("menu.Show Carved", false, true); // now the unstable points are passed as carved ones
    pangolin::Var<bool> menuDisplayNormals("menu.Show Normals", false, true);
    pangolin::Var<bool> menuDisplaySegments("menu.Show Segments", false, true);
    pangolin::Var<bool> menuDisplayObjects("menu.Show Objects", false, true);    
    pangolin::Var<bool> menuReset("menu.Reset", false, false);
    pangolin::Var<bool> menuPause("menu.Pause", false, false);
    pangolin::Var<bool> menuSave("menu.Save", false, false);
    pangolin::Var<bool> menuBA("menu.Bundle Adjust", false, false);
    pangolin::Var<int> menuParamGlPointSize("menu.Point Size", 3, 1, 10);
    pangolin::Var<int> menuDisplayMode("menu.Display Mode", 1, 1, 3);
    pangolin::Var<int> menuParamLabelConfidenceTh("menu.Label Conf Th", 5, 0, 20);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                                      pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
                                      pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
                                      );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::Display("cam")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(mUiWidth), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    // new AR
    pangolin::View& d_image = pangolin::Display("image")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(mUiWidth), 1.0,(float)mImageWidth/mImageHeight)
            .SetLock(pangolin::LockLeft, pangolin::LockTop).SetHandler(new pangolin::Handler3D(s_cam)); // use the same handler to capture the interaction with the cam!

    pangolin::GlTexture imageTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    {
    unique_lock<mutex> lock(mMutexCamP);
    mCamP = pangolin::ProjectionMatrixRDF_TopLeft(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.001,1000);
    }
    //

    mGlObjectList.Init();
    
    // 
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    pangolin::OpenGlMatrix TwcTop;
    TwcTop.SetIdentity();

    pangolin::OpenGlMatrix Tcw;
    Tcw.SetIdentity();

    cv::namedWindow(kFrameWindowName);

    bool bFollow = true;
    bool bTopCamera = false;
    bool bAR = false;
    bool bLocalizationMode = false;

    int iDisplayMode = 0;
    bool bDisplayCarved = false;
    bool bDisplayNormals = false;
    bool bDisplaySegments = false;

    int labelConfTh = menuParamLabelConfidenceTh;

    boost::posix_time::ptime start;
    int milliseconds_to_sleep = 0;

    while( !pangolin::ShouldQuit() )
    {
        start = boost::posix_time::microsec_clock::local_time();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if (menuTopCamera != bTopCamera)
        {
            if(menuTopCamera)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, mViewpointYtop, -0.01, 0, 0, 0, 0.0, -1.0, 0.0));
            }
            else
            {
                bFollow ^= 1; // trigger a bFollow update
            }
            bTopCamera = menuTopCamera;
        }

        if (menuFollowCamera != bFollow)
        {
            if(menuFollowCamera)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
            }
            bFollow = menuFollowCamera;
        }
        if(bFollow)
        {
            if(!bTopCamera)
            {
                s_cam.Follow(Twc);
                if(bDisplayNormals || bDisplaySegments) Tcw = Twc.Inverse();
            }
            else
            {
                // set only the translation
                TwcTop.m[12] = Twc.m[12];
                TwcTop.m[13] = Twc.m[13];
                TwcTop.m[14] = Twc.m[14];

                s_cam.Follow(TwcTop);
                if(bDisplayNormals || bDisplaySegments) Tcw = TwcTop.Inverse();
            }
        }


        if (menuARCamera != bAR)
        {
            if(menuARCamera)
            {
                mpFrameDrawer->setUseColorImg(true);
                //s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));                
                mpMapDrawer->setUseAR(true);
                if(mpPointCloudDrawer) mpPointCloudDrawer->setUseAR(true);                
            }
            else
            {
                mpFrameDrawer->setUseColorImg(false);
                mpMapDrawer->setUseAR(false);
                if(mpPointCloudDrawer) mpPointCloudDrawer->setUseAR(false);                

            }
            bAR = menuARCamera;
        }
        if(bAR)
        {
            // Activate camera view
            d_image.Activate();
            glColor3f(1.0,1.0,1.0);

            // Draw image
            //if(menu_drawim)
            cv::Mat im = mpFrameDrawer->GetFrame();
            cv::Mat imu;
            cv::undistort(im,imu,K,DistCoef);

            if( im.channels() == 3 )
            {
                if(!mbRGB)
                {
                    cv::cvtColor(imu,imu,cv::COLOR_RGB2BGR);
                }
                DrawImageTexture(imageTexture, imu);
            }
            else
            {
                cv::Mat imuColor(imu.cols, imu.rows, CV_8UC3);
                cv::cvtColor(imu, imuColor, cv::COLOR_GRAY2BGR);
                //cv::imshow("test", imuColor);
                DrawImageTexture(imageTexture, imuColor);
            }
            glClear(GL_DEPTH_BUFFER_BIT);

            // Load camera projection
            glMatrixMode(GL_PROJECTION);
            {
            unique_lock<mutex> lock(mMutexCamP);            
            mCamP.Load();
            }

            glMatrixMode(GL_MODELVIEW);

            // Load camera pose
            Tcw = Twc.Inverse();
            Tcw.Load();
        }


#if COMPUTE_NORMALS
        if(bDisplayNormals || bDisplaySegments)
        {
            pangolin::OpenGlMatrix mvp;
            if( !bAR )
            {
                mvp = s_cam.GetProjectionModelViewMatrix()*Tcw;
            }
            else
            {
                unique_lock<mutex> lock(mMutexCamP);
                mvp = mCamP * Tcw;
            }

            if (bDisplayNormals)
            {
                normalsProgram->SetProjectionModelViewMatrix(mvp);
            }

#if COMPUTE_SEGMENTS
            if (bDisplaySegments)
            {
                segmentsProgram->SetProjectionModelViewMatrix(mvp);
            }
#endif

        }
#endif  // end if   COMPUTE_NORMALS

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if(!bAR)
        {
            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        }

        mpMapDrawer->DrawCurrentCamera(Twc);
        if ( (!bAR) && (menuShowKeyFrames || menuShowGraph) )
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);

        if (menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        if (menuShowLines && bLinesActive)
            mpMapDrawer->DrawMapLines();
        
        if(bObjectTracking)
            mpMapDrawer->DrawMapObjects();
        
        if(menuDisplayObjects)
            mGlObjectList.Draw();        

        if (mpPointCloudDrawer)
        {
            if (menuShowPointCloud)
                mpPointCloudDrawer->DrawPC(menuParamGlPointSize);

            if (bDisplayCarved != menuDisplayUnstable)
            {
                bDisplayCarved = menuDisplayUnstable;
                mpPointCloudDrawer->SetDisplayCarved(bDisplayCarved);
            }

            if (bDisplayNormals != menuDisplayNormals)
            {
                bDisplayNormals = menuDisplayNormals;
                mpPointCloudDrawer->SetDisplayNormals(bDisplayNormals);
            }

            if(labelConfTh != menuParamLabelConfidenceTh)
            {
                labelConfTh = menuParamLabelConfidenceTh;
                mpPointCloudDrawer->SetLabelConfidenceThreshold(labelConfTh);
            }

            if (bDisplaySegments != menuDisplaySegments)
            {
                bDisplaySegments = menuDisplaySegments;
                mpPointCloudDrawer->SetDisplaySegments(bDisplaySegments);
            }                  
        }
        
        if (iDisplayMode != menuDisplayMode)
        {
            iDisplayMode = menuDisplayMode;
            if(mpPointCloudDrawer)  mpPointCloudDrawer->SetDisplayMode(menuDisplayMode);
            mGlObjectList.SetDisplayMode(menuDisplayMode);
        }       

        pangolin::FinishFrame();
        glFinish();

        if (menuSave)
        {
            if (mpPointCloudDrawer)
            {
                std::cout << "Saving volumetric map... " << std::endl;
                mpPointCloudDrawer->SavePointCloud();
                std::cout << "...done" << std::endl;
            }
            if (bObjectTracking)
            {               
                const std::vector<MapObjectPtr > &vpMObjs = mpMapDrawer->mpMap->GetAllMapObjects();
                for(size_t i=0, iend=vpMObjs.size(); i<iend;i++)
                {
                    vpMObjs[i]->SaveRefObservations();
                }
            }
            mpSystem->SaveMap();
            menuSave = false;
        }

        if (menuBA)
        {
            mpSystem->StartGlobalBundleAdjustment();
            menuBA = false;
        }

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow(kFrameWindowName, im);

#if COMPUTE_SEGMENTS
        if (pVecImages )
        {
            for(size_t i=0; i<pVecImages->size();i++)
            {
                const Image4Viewer& imgi = pVecImages->at(i);
                if(imgi.bReady)
                    cv::imshow(imgi.name, imgi.img);
            }
        }
#endif

#if REAL_TIME_MODE
        cv::waitKey(2);
#else
        cv::waitKey(mT);
#endif

        if (menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if (menuPause)
        {
            mpSystem->TogglePause();
            menuPause = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;

#if REAL_TIME_MODE
        milliseconds_to_sleep = (boost::posix_time::microsec_clock::local_time() - start).total_milliseconds() - mT;
        if (milliseconds_to_sleep > 0)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(milliseconds_to_sleep));
        }
#endif

    } /// end while
    
    if(mpPointCloudDrawer)
    {
        mpPointCloudDrawer.reset(); // destroy the point cloud drawer in the same drawing thread context 
    }    

    cv::destroyAllWindows();
    pangolin::DestroyWindow(kMapWindowName);
    
    SetFinish();
    
    std::cout << "Viewer::Run() - exit " << std::endl;    
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

bool Viewer::shouldFinished() {
  // return pangolin::ShouldQuit();
  return mbRequestQuit;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im, bool bColor)
{
    if(!im.empty())
    {
        if(bColor)
        {
            //use fast 4-byte alignment (default anyway) if possible
            glPixelStorei(GL_UNPACK_ALIGNMENT, (im.step & 3) ? 1 : 4);

            //imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
            imageTexture.Upload(im.data, 0, 0, im.cols, im.rows, GL_BGR,GL_UNSIGNED_BYTE);
        }
        else
        {
            imageTexture.Upload(im.data,GL_LUMINANCE,GL_UNSIGNED_BYTE);
        }
        imageTexture.RenderToViewportFlipY();
    }
}

void Viewer::SetCameraCalibration(float fx, float fy, float cx, float cy )
{
    unique_lock<mutex> lock(mMutexCamP);
    mfx = fx;
    mfy = fy;
    mcx = cx;
    mcy = cy;
    mCamP = pangolin::ProjectionMatrixRDF_TopLeft(mImageWidth,mImageHeight,mfx,mfy,mcx,mcy,0.001,1000);
}

}
