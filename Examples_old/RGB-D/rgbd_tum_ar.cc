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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "System.h"
#include "Tracking.h"
#include "Logger.h"
#include "ViewerAR.h"
#include "Converter.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void InitViewerAR(cv::FileStorage& fSettings, PLVS2::ViewerAR& viewerAR, cv::Mat& K, cv::Mat& DistCoef, bool& bRGB);
void updateViewerAR(PLVS2::ViewerAR& viewerAR, PLVS2::System& SLAM, cv::Mat& im, const cv::Mat& Tcw, const cv::Mat& K, const cv::Mat& DistCoef, bool bRGB);

int main(int argc, char **argv) 
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }
    
    cout << "running..." << endl;

    cout << "retrieve paths to images ..." << endl;
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    cout << "check consistency in the number of images and depthmaps ..." << endl;
    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bool bUseViewer = false;//static_cast<int> (PLVS2::Utils::GetParam(fSettings, "Viewer.on", 1)) != 0;
        
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    PLVS2::System SLAM(argv[1],argv[2],PLVS2::System::RGBD,bUseViewer);
    
    
    PLVS2::ViewerAR viewerAR;
    viewerAR.SetSLAM(&SLAM);        
    bool bRGB = true;
    cv::Mat K;
    cv::Mat DistCoef;     
    InitViewerAR(fSettings, viewerAR, K, DistCoef, bRGB);    
    thread tViewer = thread(&PLVS2::ViewerAR::Run,&viewerAR);        

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    
    int numImgsNoInit = 0; 
    int numImgsLost = 0; 
    
    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        Sophus::SE3f Tcw = SLAM.TrackRGBD(imRGB,imD,tframe);
        
        updateViewerAR(viewerAR, SLAM, imRGB, PLVS2::Converter::toCvSE3(Tcw), K, DistCoef, bRGB);        
        
        int trackingState = SLAM.GetTrackingState();
        if(trackingState == PLVS2::Tracking::NOT_INITIALIZED) numImgsNoInit++;
        if(trackingState == PLVS2::Tracking::LOST) numImgsLost++;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
#if 1
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
#else        
        std::cout<<"img " << ni << std::endl; 
        if(SLAM.GetTrackingState() != PLVS2::Tracking::NOT_INITIALIZED)
        {
            getchar(); // step by step
        }
#endif
        
    }
    
    if(bUseViewer)
    {
        std::cout << "\n******************\n" << std::endl;
        std::cout << "press a key to end" << std::endl;
        std::cout << "\n******************\n" << std::endl;
        getchar();
    }

    // Stop all threads
    SLAM.Shutdown();
        
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    
    cout << "perc images lost: " << (float(numImgsLost)/nImages)*100. << std::endl; 
    cout << "perc images no init: " << (float(numImgsNoInit)/nImages)*100. << std::endl;     

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   
    
    Logger logger("Performances.txt");
    logger << "perc images lost: " << (float(numImgsLost)/nImages)*100. << std::endl; 
    logger << "perc images no init: " << (float(numImgsNoInit)/nImages)*100. << std::endl; 
    logger << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    logger << "mean tracking time: " << totaltime/nImages << endl;

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    if (!fAssociation.is_open()) 
    { 
        std::cout << "cannot open file: " << strAssociationFilename << std::endl;
        quick_exit(-1);
    }

    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


void InitViewerAR(cv::FileStorage& fSettings, PLVS2::ViewerAR& viewerAR, cv::Mat& K, cv::Mat& DistCoef, bool& bRGB)
{
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    viewerAR.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    viewerAR.SetCameraCalibration(fx,fy,cx,cy);

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }    
}


void updateViewerAR(PLVS2::ViewerAR& viewerAR, PLVS2::System& SLAM, cv::Mat& imRGB, const cv::Mat& Tcw, const cv::Mat& K, const cv::Mat& DistCoef, bool bRGB)
{    
    cv::Mat im = imRGB.clone();
    cv::Mat imu;
    int state = SLAM.GetTrackingState();
    vector<PLVS2::MapPointPtr> vMPs = SLAM.GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = SLAM.GetTrackedKeyPointsUn();

    cv::undistort(im,imu,K,DistCoef);

    if(!bRGB)
    {
        cv::cvtColor(imu,imu,cv::COLOR_RGB2BGR);
    }    
    viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);    
}