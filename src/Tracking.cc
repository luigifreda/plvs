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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#ifdef USE_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

#include "LineExtractor.h"
#include "LineMatcher.h"
#include "MapLine.h"
#include "Utils.h"
#include "Stopwatch.h"
#include "MapObject.h"

#include <iostream>

#include <mutex>
#include <chrono>

#define CREATE_NEW_LINES_ON_TRACKING 1   // set this to zero in order to check how local mapping is able to triangulate lines 
#define USE_LINE_MATCHING_BY_KNN 0       // just for testing how search-by-projection is better w.r.t. KNN-based search 

using namespace std;

namespace PLVS2
{

int Tracking::skNumMinFeaturesStereoInitialization = 500; 
int Tracking::skNumMinFeaturesRGBDInitialization   = 500;
const int Tracking::kNumMinFeaturesMonoInitialization   = 100;

const int Tracking::kMaxNumOfKeyframesInLocalMap = 80; // originally it was 80
const int Tracking::kNumBestCovisibilityKeyFrames = 10; // originally it was 10

const double Tracking::kMaxTimeInSecsForRelocalizationFailureBeforeLost = 5.0; // [seconds] 

float Tracking::sknLineTrackWeigth = 2.; //  line weight for weighting line observations w.r.t. point observations in tracking 
                                        //   {3 points} or {1 line + 1 point} are enough to zero the DOF of a body => {3 points} "=" {1 line + 1 point} = > 1 line counts as 2 points

float Tracking::skLineStereoMaxDist = 20.; // [meters]
float Tracking::skMaxDistFovCenters = 0.5; // [meters]

bool Tracking::skUsePyramidPrecomputation = false; // use pyramid pre-computation in order to share image pyramid between ORBExtractor and LineExctor,
                                                   // this will force the use of the same scale on both feature extractors 
                                                   // (at present time, this does not actually seem beneficial) 

/*
// Tracking states
enum eTrackingState{
    SYSTEM_NOT_READY=-1,
    NO_IMAGES_YET=0,
    NOT_INITIALIZED=1,
    OK=2,
    RECENTLY_LOST=3,
    LOST=4,
    OK_KLT=5,
    RELOCALIZE_IN_LOADED_MAP=6   
    NUM_TRACKING_STATES_1
};*/
std::vector<std::string> Tracking::vTrackingStateStrings = {
    "SYSTEM_NOT_READY",
    "NO_IMAGES_YET",
    "NOT_INITIALIZED",
    "OK",
    "RECENTLY_LOST",
    "LOST",
    "OK_KLT",
    "RELOCALIZE_IN_LOADED_MAP"
}; // must be kept in sync with eTrackingState


Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings, const string &_nameSeq):
    mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFr(0), mbStep(false),
    mbOnlyTracking(false), mbMapUpdated(false), mbVO(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
    mbReadyToInitializate(false), 
    mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnLastRelocFrameId(0), time_recently_lost(5.0),
    mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0), mpCamera2(nullptr),
    mpLastKeyFrame(static_cast<KeyFrame*>(NULL))
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ); // TODO: Luigi fixme this must be an alternative to newParameterLoader() 

    // Load camera parameters from settings file
    if(settings){
        newParameterLoader(settings);
    }
    else{
        //cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool b_parse_cam = ParseCamParamFile(fSettings);
        if(!b_parse_cam)
        {
            std::cout << "*Error with the camera parameters in the config file*" << std::endl;
        }

        // Load feature parameters
        bool b_parse_orb = ParseFeaturesParamFile(fSettings);
        if(!b_parse_orb)
        {
            std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
        }

        bool b_parse_imu = true;
        if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO || sensor==System::IMU_RGBD)
        {
            b_parse_imu = ParseIMUParamFile(fSettings);
            if(!b_parse_imu)
            {
                std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
            }

            mnFramesToResetIMU = mMaxFrames;
        }

        if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
        {
            std::cerr << "**ERROR in the config file [Tracking], the format is not correct**" << std::endl;
            std::cout << "b_parse_cam: " << (int) b_parse_cam << std::endl; 
            std::cout << "b_parse_orb: " << (int) b_parse_orb << std::endl; 
            std::cout << "b_parse_imu: " << (int) b_parse_imu << std::endl;                         
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    initID = 0; lastID = 0;
    mbInitWith3KFs = false;

    // ---- ---- ---- 
    // Initialization parameters
    cout << endl  << "Initialization Parameters: " << endl;  
    if(sensor==System::RGBD)
        Tracking::skNumMinFeaturesRGBDInitialization = Utils::GetParam(fSettings, "Initialization.numMinFeaturesRGBD", Tracking::skNumMinFeaturesRGBDInitialization);  
    if(sensor==System::STEREO)
        Tracking::skNumMinFeaturesStereoInitialization = Utils::GetParam(fSettings, "Initialization.numMinFeaturesStereo", Tracking::skNumMinFeaturesStereoInitialization);     
        
    // KeyFrame generation based on FOV centers 
    cout << endl  << "KeyFrame Generation Parameters: " << endl;    
    mbUseFovCentersKfGenCriterion = static_cast<int> (Utils::GetParam(fSettings, "KeyFrame.fovCentersBasedGeneration.on", 0)) != 0;  
    Frame::mbUseFovCentersKfGenCriterion = mbUseFovCentersKfGenCriterion;
    skMaxDistFovCenters = Utils::GetParam(fSettings, "KeyFrame.maxFovCentersDistance", skMaxDistFovCenters);

    // ---- ---- ---- 
    // Depth Model parameters 
    
    cout << endl  << "Depth Model Parameters: " << endl;    
    Optimizer::skSigmaZFactor = Utils::GetParam(fSettings, "Depth.sigmaZfactor", Optimizer::skSigmaZFactor);

    mEnableDepthFilter = static_cast<int> (Utils::GetParam(fSettings, "DepthFilter.Morphological.on", 0)) != 0; 
    mDepthCutoff = Utils::GetParam(fSettings, "DepthFilter.Morphological.cutoff", 20);
#ifdef USE_CUDA
    if (mEnableDepthFilter) 
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        mpClosingFilter = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, CV_32F, kernel);
        mpOpeningFilter = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_32F, kernel);
    }
#endif    
    
    
    // ---- ---- ---- 
    
    mnNumDataset = 0;

    vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for(GeometricCamera* pCam : vpCams)
    {
        std::cout << "Camera " << pCam->GetId();
        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            std::cout << " is pinhole" << std::endl;
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            std::cout << " is fisheye" << std::endl;
        }
        else
        {
            std::cout << " is unknown" << std::endl;
        }
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif

    vnKeyFramesLM.clear();
    vnMapPointsLM.clear();
    vnMapLinesLM.clear();

    std::cout << "Tracking - check : DistCoef = " << mDistCoef.t() << std::endl;     
    std::cout << "Tracking - check : mpCamera2 = " << (mpCamera2 ? "true" : "false") << std::endl;       
}

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::LocalMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
    {
        f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
          << mpLocalMapper->vdKFCullingSync_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
    {
        f << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
          << mpLocalMapper->vnLBA_edges[i] << endl;
    }


    f.close();
}

void Tracking::TrackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << mpAtlas->GetAllKeyFrames().size() << endl;
    f << "Number of MPs: " << mpAtlas->GetAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

    for(int i=0; i<vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if(!vdRectStereo_ms.empty())
        {
            stereo_rect = vdRectStereo_ms[i];
        }

        double resize_image = 0.0;
        if(!vdResizeImage_ms.empty())
        {
            resize_image = vdResizeImage_ms[i];
        }

        double stereo_match = 0.0;
        if(!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if(!vdIMUInteg_ms.empty())
        {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
          << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
    }

    f.close();
}

void Tracking::PrintTimeStats()
{
    // Save data in files
    TrackStats2File();
    LocalMapStats2File();


    ofstream f;
    f.open("ExecMean.txt");
    f << fixed;
    //Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if(!vdRectStereo_ms.empty())
    {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdResizeImage_ms.empty())
    {
        average = calcAverage(vdResizeImage_ms);
        deviation = calcDeviation(vdResizeImage_ms, average);
        std::cout << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if(!vdStereoMatch_ms.empty())
    {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation << std::endl;
    }

    if(!vdIMUInteg_ms.empty())
    {
        average = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation << std::endl;
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLBA_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdKFCulling_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFCulling_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_edges);
    deviation = calcDeviation(mpLocalMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFopt);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_KFfixed);
    deviation = calcDeviation(mpLocalMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpLocalMapper->vnLBA_MPs);
    deviation = calcDeviation(mpLocalMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    std::cout << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << mpLocalMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << mpLocalMapper->nLBA_abort << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "KFs in map: " << mpAtlas->GetAllKeyFrames().size() << std::endl;
    std::cout << "MPs in map: " << mpAtlas->GetAllMapPoints().size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBestMap = vpMaps[0];
    for(int i=1; i<vpMaps.size(); ++i)
    {
        if(pBestMap->GetAllKeyFrames().size() < vpMaps[i]->GetAllKeyFrames().size())
        {
            pBestMap = vpMaps[i];
        }
    }

    f << "KFs in map: " << pBestMap->GetAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << pBestMap->GetAllMapPoints().size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdDataQuery_ms);
    deviation = calcDeviation(mpLoopClosing->vdDataQuery_ms, average);
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdEstSim3_ms);
    deviation = calcDeviation(mpLoopClosing->vdEstSim3_ms, average);
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdPRTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdPRTotal_ms, average);
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopFusion_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopFusion_ms, average);
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopOptEss_ms, average);
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdLoopTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdLoopTotal_ms, average);
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nLoop << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nLoop << std::endl;
    average = calcAverage(mpLoopClosing->vnLoopKFs);
    deviation = calcDeviation(mpLoopClosing->vnLoopKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeMaps_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeMaps_ms, average);
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdWeldingBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdWeldingBA_ms, average);
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeOptEss_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeOptEss_ms, average);
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdMergeTotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdMergeTotal_ms, average);
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nMerges << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nMerges << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeKFs);
    deviation = calcDeviation(mpLoopClosing->vnMergeKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnMergeMPs);
    deviation = calcDeviation(mpLoopClosing->vnMergeMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    average = calcAverage(mpLoopClosing->vdGBA_ms);
    deviation = calcDeviation(mpLoopClosing->vdGBA_ms, average);
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdUpdateMap_ms);
    deviation = calcDeviation(mpLoopClosing->vdUpdateMap_ms, average);
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vdFGBATotal_ms);
    deviation = calcDeviation(mpLoopClosing->vdFGBATotal_ms, average);
    f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;
    std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl << std::endl;

    f << "Numb exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    std::cout << "Num exec: " << mpLoopClosing->nFGBA_exec << std::endl;
    f << "Numb abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    std::cout << "Num abort: " << mpLoopClosing->nFGBA_abort << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAKFs);
    deviation = calcDeviation(mpLoopClosing->vnGBAKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpLoopClosing->vnGBAMPs);
    deviation = calcDeviation(mpLoopClosing->vnGBAMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;

    f.close();

}

#endif

Tracking::~Tracking()
{
    //f_track_stats.close();

}

void Tracking::newParameterLoader(Settings *settings) {
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if(settings->needToUndistort()){
        mDistCoef = settings->camera1DistortionCoef();
    }
    else{
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    }

    //TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = mpCamera->toLinearK();

    mK_ = mpCamera->toLinearK_();

    if((mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD) &&
        settings->cameraType() == Settings::KannalaBrandt){
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();

        mpFrameDrawer->both = true;
    }  

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD ){
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD){
        mDepthMapFactor = settings->depthMapFactor();
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    //ORB parameters
    int nFeatures = settings->nFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    //Line parameters 
    const SettingsLines& linesSettings = settings->linesSettings();    
    mbLineTrackerOn = linesSettings.bLineTrackerOn;      
    int numLineFeatures = linesSettings.nNumLineFeatures;

    cv::line_descriptor_c::LSDDetectorC::LSDOptions lsdOptions;        
    lsdOptions.numOctaves = linesSettings.lsdOptions.numOctaves; 
    lsdOptions.scale = linesSettings.lsdOptions.scale;     
    lsdOptions.refine = linesSettings.lsdOptions.refine;   
    lsdOptions.sigma_scale = linesSettings.lsdOptions.sigma_scale;
    lsdOptions.quant = linesSettings.lsdOptions.quant;
    lsdOptions.ang_th = linesSettings.lsdOptions.ang_th;
    lsdOptions.log_eps = linesSettings.lsdOptions.log_eps;
    lsdOptions.density_th = linesSettings.lsdOptions.density_th;
    lsdOptions.n_bins = linesSettings.lsdOptions.n_bins;
    lsdOptions.min_length = linesSettings.lsdOptions.min_length;
    if(mbLineTrackerOn)
    {
        cout  << endl << "Using Line tracking" << endl; 
        mpLineExtractorLeft =  std::make_shared<LineExtractor>(numLineFeatures, lsdOptions); 
        
        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)            
            mpLineExtractorRight =  std::make_shared<LineExtractor>(numLineFeatures, lsdOptions);        
    }    

    //IMU parameters
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001; //1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);      
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        if(mImageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * mImageScale;
            fy = fy * mImageScale;
            cx = cx * mImageScale;
            cy = cy * mImageScale;
        }
        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        mpCamera = mpAtlas->AddCamera(mpCamera);

        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- Image scale: " << mImageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;

        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

        mK_.setIdentity();
        mK_(0,0) = fx;
        mK_(1,1) = fy;
        mK_(0,2) = cx;
        mK_(1,2) = cy;
    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if(!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if(!b_miss_params)
        {
            if(mImageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);
            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- Image scale: " << mImageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;

            mK_.setIdentity();
            mK_(0,0) = fx;
            mK_(1,1) = fy;
            mK_(0,2) = cx;
            mK_(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            cv::Mat cvTlr;
            if(!node.empty())
            {
                cvTlr = node.mat();
                if(cvTlr.rows != 3 || cvTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                if(mImageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;

                    leftLappingBegin = leftLappingBegin * mImageScale;
                    leftLappingEnd = leftLappingEnd * mImageScale;
                    rightLappingBegin = rightLappingBegin * mImageScale;
                    rightLappingEnd = rightLappingEnd * mImageScale;
                }

                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);
                mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                mTlr = Converter::toSophus(cvTlr);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << mImageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << cvTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD )
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
            if(mImageScale != 1.f)
            {
                mbf *= mImageScale;
            }
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
    {
        float fx = mpCamera->getLinearParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseFeaturesParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node;
    
    cout << endl  << "Features Parameters: " << endl;    
    bool bBalanceTotalNumFeatures = static_cast<int> (Utils::GetParam(fSettings, "Features.balanceTotalNumFeatures", 0)) != 0;   
    
    nFeatures = Utils::GetParam(fSettings, "ORBextractor.nFeatures", 1000, false);    
    
    mbLineTrackerOn = static_cast<int> (Utils::GetParam(fSettings, "Line.on", 0, false)) != 0;    
    int numLineFeatures = Utils::GetParam(fSettings, "Line.nfeatures", 100, false);     
    if( bBalanceTotalNumFeatures && mbLineTrackerOn )
    {
        nFeatures = std::max( 500, nFeatures - 2 * numLineFeatures);
        std::cout << "balancing num ORB features: " << nFeatures << std::endl;
    }

#if 0
    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }
#endif 
    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST); //NOTE: in ORBSLAM2 it was 2*nFeatures and NOT 5*nFeatures !!!!!

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
    
    // Line extractor
    cout << endl  << "Line Extractor Parameters: " << endl;    
    
    cv::line_descriptor_c::LSDDetectorC::LSDOptions lsdOptions;
    
    mbLineTrackerOn = Utils::GetParam(fSettings, "Line.on", false);      
    numLineFeatures = Utils::GetParam(fSettings, "Line.nfeatures", 100); // read again just for printing out
        
    Tracking::skUsePyramidPrecomputation = Utils::GetParam(fSettings, "Line.pyramidPrecomputation", false);   
     
    lsdOptions.numOctaves = Utils::GetParam(fSettings, "Line.nLevels", LineExtractor::kNumOctavesForBdDefault); 
    
    lsdOptions.scale = Utils::GetParam(fSettings, "Line.scaleFactor", LineExtractor::kScaleFactorDefault);  
    if( Tracking::skUsePyramidPrecomputation )
    {
        lsdOptions.scale = fScaleFactor;
    }
    
    LineExtractor::skSigma0 = Utils::GetParam(fSettings, "Line.sigma", LineExtractor::skSigma0);    
        
    // LSD options  http://docs.opencv.org/3.0-beta/modules/imgproc/doc/feature_detection.html
    LineExtractor::skUseLsdExtractor = static_cast<int> (Utils::GetParam(fSettings, "Line.LSD.on", (int)LineExtractor::skUseLsdExtractor)) != 0;    
    lsdOptions.refine = Utils::GetParam(fSettings, "Line.LSD.refine", 1);   
    lsdOptions.sigma_scale = Utils::GetParam(fSettings, "Line.LSD.sigmaScale", 0.6);
    lsdOptions.quant = Utils::GetParam(fSettings, "Line.LSD.quant", 2.0);
    lsdOptions.ang_th = Utils::GetParam(fSettings, "Line.LSD.angTh", 22.5);
    lsdOptions.log_eps = Utils::GetParam(fSettings, "Line.LSD.logEps", 1.0);
    lsdOptions.density_th = Utils::GetParam(fSettings, "Line.LSD.densityTh", 0.6);
    lsdOptions.n_bins = Utils::GetParam(fSettings, "Line.LSD.nbins", 1024);
    lsdOptions.min_length = Utils::GetParam(fSettings, "Line.minLineLength", 0.025);
    lsdOptions.lineFitErrThreshold = Utils::GetParam(fSettings, "Line.lineFitErrThreshold", lsdOptions.lineFitErrThreshold);
        
    Tracking::sknLineTrackWeigth = 0;
    if(mbLineTrackerOn) 
    {
        Tracking::sknLineTrackWeigth = Utils::GetParam(fSettings, "Line.lineTrackWeigth", Tracking::sknLineTrackWeigth); 
    }
    else
    {
        std::cout << "Line.lineTrackWeigth" << ": " << Tracking::sknLineTrackWeigth << std::endl;
    }
    
    Tracking::skLineStereoMaxDist = Utils::GetParam(fSettings, "Line.Stereo.maxDist", Tracking::skLineStereoMaxDist);    
    
    Frame::skMinLineLength3D = Utils::GetParam(fSettings, "Line.minLineLength3D", Frame::skMinLineLength3D);
    Optimizer::skMuWeightForLine3dDist = Utils::GetParam(fSettings, "Line.muWeightForLine3dDist", Optimizer::skMuWeightForLine3dDist);    
    
    // update parameters which depend on Optimizer::skMuWeightForLine3dDist
    //Optimizer::skSigmaLineError3D = (1 + Optimizer::skMuWeightForLine3dDist)*Optimizer::kSigmaPointLineDistance;    
    //Optimizer::skInvSigma2LineError3D = 1.0/(Optimizer::skSigmaLineError3D * Optimizer::skSigmaLineError3D );     
    

    // ---- ---- ----     
    // Map Object 
    cout << endl  << "Map Objects Parameters: " << endl;  
    mbObjectTrackerOn = static_cast<int> (Utils::GetParam(fSettings, "MapObject.on", 0)) != 0;          
    if(mbObjectTrackerOn)
    {
        std::string imgFilenamesString = Utils::GetParam(fSettings, "MapObject.imgs", std::string("object.png"));    
        std::vector<std::string> imgFilenames;
        Utils::TokenizeString(imgFilenamesString, imgFilenames, " ");
        MapObject::skMatchRatio = Utils::GetParam(fSettings, "MapObject.matchRatio", MapObject::skMatchRatio);        
        MapObject::skNumMinInliers = Utils::GetParam(fSettings, "MapObject.numMinInliers", MapObject::skNumMinInliers);
        MapObject::skMaxImgReprojectionError = Utils::GetParam(fSettings, "MapObject.maxReprojectionError", MapObject::skMaxImgReprojectionError);    
        MapObject::skMaxSim3Error = Utils::GetParam(fSettings, "MapObject.maxSim3Error", MapObject::skMaxSim3Error);                    
        
        int numImgsOpened = 0;
        for(size_t ii=0, iiend=imgFilenames.size(); ii<iiend; ii++)
        {        
            cv::Mat imgObject = cv::imread(imgFilenames[ii], cv::IMREAD_GRAYSCALE);
            if(!imgObject.empty())
            {
                std::cout << "loaded image file: " << imgFilenames[ii] << std::endl;                 
                
                //MapObjectPtr pMapObject( std::make_shared<MapObject>(pMap, imgObject, mK, mDistCoef) );
                MapObjectPtr pMapObject = MapObjectNewPtr(mpAtlas->GetCurrentMap(), imgObject, mK, mDistCoef);
                
                pMapObject->InitFeatures(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
                mpAtlas->AddMapObject(pMapObject);
                numImgsOpened++;
            }
            else
            {
                std::cerr << "could not open object image file: " << imgFilenames[ii] << std::endl; 
            }
        }
        mbObjectTrackerOn = numImgsOpened > 0;
        
        mvpLocalMapObjects = mpAtlas->GetAllMapObjects();
        mpAtlas->SetReferenceMapObjects(mvpLocalMapObjects);
    }

    // ---- ---- ---- 
    
    if(mbLineTrackerOn)
    {
        cout  << endl << "Using Line tracking" << endl; 
        mpLineExtractorLeft =  std::make_shared<LineExtractor>(numLineFeatures, lsdOptions); 
        
        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
            mpLineExtractorRight =  std::make_shared<LineExtractor>(numLineFeatures, lsdOptions);        
    }

    return true; 
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        cvTbc = node.mat();
        if(cvTbc.rows != 4 || cvTbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }
    cout << endl;
    cout << "Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
    Eigen::Matrix<float,4,4,Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f Tbc(eigTbc);

    node = fSettings["InsertKFsWhenLost"];
    mInsertKFsLost = true;
    if(!node.empty() && node.isInt())
    {
        mInsertKFsLost = (bool) node.operator int();
    }

    if(!mInsertKFsLost)
        cout << "Do not insert keyframes when lost visual tracking " << endl;



    float Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        mImuFreq = node.operator int();
        mImuPer = 0.001; //1.0 / (double) mImuFreq;
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.fastInit"];
    mFastInit = false;
    if(!node.empty())
    {
        mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
    }

    if(mFastInit)
        cout << "Fast IMU initialization. Acceleration is not checked \n";

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(mImuFreq);
    cout << endl;
    cout << "IMU frequency: " << mImuFreq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

void Tracking::SetPointCloudMapping(std::shared_ptr<PointCloudMapping>& pPointCloudMapping)
{
    mpPointCloudMapping = pPointCloudMapping;
}

void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;
}

bool Tracking::GetStepByStep()
{
    return bStepByStep;
}



Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename)
{
    //cout << "GrabImageStereo" << endl;

    mImGray = imRectLeft;
    mImgGrayRight = imRectRight;
    mImRight = imRectRight;

    if(mImGray.channels()==3)
    {
        //cout << "Image with 3 channels" << endl;
        mImRGB = mImGray.clone();
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
            cvtColor(mImgGrayRight,mImgGrayRight,cv::COLOR_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
            cvtColor(mImgGrayRight,mImgGrayRight,cv::COLOR_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        //cout << "Image with 4 channels" << endl;
        mImRGB = mImGray.clone();
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
            cvtColor(mImgGrayRight,mImgGrayRight,cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
            cvtColor(mImgGrayRight,mImgGrayRight,cv::COLOR_BGRA2GRAY);
        }
    }

    TICKTRACK("Track");     
    //cout << "Incoming frame creation" << endl;

    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,mImgGrayRight,timestamp,mpLineExtractorLeft,mpLineExtractorRight,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,mImgGrayRight,timestamp,mpLineExtractorLeft,mpLineExtractorRight,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr);
    else if(mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,mImgGrayRight,timestamp,mpLineExtractorLeft,mpLineExtractorRight,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,mImgGrayRight,timestamp,mpLineExtractorLeft,mpLineExtractorRight,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib);

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
#endif

    //cout << "Tracking start" << endl;
    Track();
    //cout << "Tracking end" << endl;

    
    TOCKTRACK("Track");
    
    SENDALLTRACK;    

    return mCurrentFrame.GetPose();
}


Sophus::SE3f Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename)
{
    mImRGB   = imRGB;
    mImGray  = imRGB;
    mImDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || mImDepth.type()!=CV_32F)
        mImDepth.convertTo(mImDepth,CV_32F,mDepthMapFactor);
    
#ifdef USE_CUDA
    if (mEnableDepthFilter) 
    {
        cv::cuda::GpuMat gpuInput_gray(mImDepth);
        cv::cuda::threshold(gpuInput_gray, gpuInput_gray, mDepthCutoff, 0.0, cv::THRESH_TOZERO_INV);
        mpOpeningFilter->apply(gpuInput_gray, gpuInput_gray);
        mpClosingFilter->apply(gpuInput_gray, gpuInput_gray);
        gpuInput_gray.download(mImDepth);
    }
#endif
    
    /*
    double min, max;
    cv::minMaxLoc(mImDepth, &min, &max);
    std::cout << "depth - min: " << min << " max: " << max << std::endl; 
    */    

    TICKTRACK("Track"); 

    if (mSensor == System::RGBD)
    {
        //mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
        mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpLineExtractorLeft,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    }
    else if(mSensor == System::IMU_RGBD)
    {
        //mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
        mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpLineExtractorLeft,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    }

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    Track();

    TOCKTRACK("Track");
    
    SENDALLTRACK;
    return mCurrentFrame.GetPose();
}


Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,cv::COLOR_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,cv::COLOR_BGRA2GRAY);
    }

    if (mSensor == System::MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
#endif

    lastID = mCurrentFrame.mnId;
    Track();

    return mCurrentFrame.GetPose();
}


void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU()
{

    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if(mlQueueImuData.size() == 0)
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while(true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if(!mlQueueImuData.empty())
            {
                IMU::Point* m = &mlQueueImuData.front();
                cout.precision(17);
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-mImuPer)
                {
                    mlQueueImuData.pop_front();
                }
                else if(m->t<mCurrentFrame.mTimeStamp-mImuPer)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }

    const int n = mvImuFromLastFrame.size()-1;
    if(n==0){
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);

    for(int i=0; i<n; i++)
    {
        //float tstep; // [Luigi] BugFix timestamps should always be represented with double!
        double tstep;
        Eigen::Vector3f acc, angVel;
        if((i==0) && (i<(n-1)))
        {
            double tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;                     // Luigi: changed to double 
            double tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;        // Luigi: changed to double 
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        else if((i>0) && (i==(n-1)))
        {
            double tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;                        // Luigi: changed to double 
            double tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;                      // Luigi: changed to double 
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    //Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}


bool Tracking::PredictStateIMU()
{
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if(mbMapUpdated && mpLastKeyFrame)
    {
        // NOTE: [Luigi] probably, to increase precision, convert matrices to double and keep time as double, 
        //       then reconvert to float. However, this is just a prediction. 

        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;  // NOTE: [Luigi] this is conversion of time from double to float!

        // preintegrated IMU measurements (Sect VI.A in "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry")
        // combining equation (2) and (3) from "Visual-Inertial Monocular SLAM with map reuse": using the bias coming from the last keyframe
        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else if(!mbMapUpdated)
    {
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT; // NOTE: [Luigi] this is conversion of time from double to float!

        // preintegrated IMU measurements (Sect VI.A in "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry") 
        // combining equation (2) and (3) from "Visual-Inertial Monocular SLAM with map reuse": using the bias coming from the last frame  
        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        Eigen::Vector3f twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 = Vwb1 + t12*Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);

        mCurrentFrame.mImuBias = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}


void Tracking::Track()
{
#if 0    
    std::cout << "Tracking::Track() - frame id " << mCurrentFrame.mnId << endl;
#endif 
    
    if (bStepByStep)
    {
        std::cout << "Tracking: Waiting to the next step" << std::endl;
        while(!mbStep && bStepByStep)
            usleep(500);
        mbStep = false;
    }

    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    if(!pCurrentMap)
    {
        cout << "ERROR: There is not an active map in the atlas" << endl;
    }

    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            // cout << mCurrentFrame.mTimeStamp << ", " << mLastFrame.mTimeStamp << endl;
            cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            if(mpAtlas->isInertial())
            {

                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Resetting IMU integration..." << endl;
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        std::cout << "Resetting the active map" << std::endl; 
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        std::cout << "Creating a new map in Atlas" << std::endl; 
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Resetting..." << endl;
                    mpSystem->ResetActiveMap();
                }
                
                return;
            }
            else
            {
                // NOTE: [Luigi] added this for avoiding that during ROS play-back the algorithm completely stops and does not try to recover 
                cout << "Timestamp jump detected"  << endl; 
#if 0                
                cout << "Setting LOST..." << endl;
                mState = LOST;
#endif 
            }


        }
    }


    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mbCreatedMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU = std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU = std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPreIMU - time_StartPreIMU).count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif

    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        // cout << "Map update detected" << endl;
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

    if(mState==RELOCALIZE_IN_LOADED_MAP)
    {
        InitForRelocalizationInMap();
    }

#if 0
    std::cout << "State: " << GetTrackingStateString() << std::endl; //"(" << mState << ")" << std::endl; 
#endif 
    
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO || mSensor==System::IMU_RGBD)
        {
            StereoInitialization();
        }
        else
        {
            MonocularInitialization();  /// < TODO: integrate line initialization ?
        }

        mpFrameDrawer->Update(this);  // Luigi: enabled back in order to visualize the image flow even when tracking is not OK

        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            /// < SLAM Mode: Local Mapping is active

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if(mState==OK)
            {

                // Local Mapping might have changed some Map features tracked in last frame
                CheckReplacedInLastFrame(); /// < OKL

                if((!mbVelocity && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackReferenceKeyFrame(); /// < OKL
                }
                else
                {
                    Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    bOK = TrackWithMotionModel();  /// < OKL
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame(); /// < OKL
                }


                if (!bOK)
                {
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        // cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else
            {

                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))
                    {
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else
                    {
                        // Relocalization
                        bOK = Relocalization();
                        //std::cout << "mCurrentFrame.mTimeStamp:" << to_string(mCurrentFrame.mTimeStamp) << std::endl;
                        //std::cout << "mTimeStampLost:" << to_string(mTimeStampLost) << std::endl;
                        if(mCurrentFrame.mTimeStamp-mTimeStampLost>3.0f && !bOK)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)
                {

                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        Verbose::PrintMess("Resetting current map...", Verbose::VERBOSITY_NORMAL);
                    }else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFramePtr>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }

        }
        else
        {
            /// < Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints and MapLines in the map
                    if(mbVelocity)
                    {
                        bOK = TrackWithMotionModel(); /// < OKL
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame(); /// < OKL
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is successful we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPointPtr> vpMPsMM;
                    vector<MapLinePtr> vpMLsMM;
                    vector<bool> vbOutMM;
                    vector<bool> vbOutLinesMM;
                    Sophus::SE3f TcwMM;
                    if(mbVelocity)
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        if(mbLineTrackerOn)
                        {
                            vpMLsMM = mCurrentFrame.mvpMapLines;                        
                            vbOutLinesMM = mCurrentFrame.mvbLineOutlier;
                        }
                        TcwMM = mCurrentFrame.GetPose();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;
                        if(mbLineTrackerOn)
                        {
                            mCurrentFrame.mvpMapLines = vpMLsMM;
                            mCurrentFrame.mvbLineOutlier = vbOutLinesMM;
                        }

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                            
                            if(mbLineTrackerOn)
                            {
                                for(int i =0; i<mCurrentFrame.Nlines; i++)
                                {
                                    if(mCurrentFrame.mvpMapLines[i] && !mCurrentFrame.mvbLineOutlier[i])
                                    {
                                        mCurrentFrame.mvpMapLines[i]->IncreaseFound();
                                    }
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        } /// end if(!mbOnlyTracking)
        

        if(!mCurrentFrame.mpReferenceKF) // Luigi: in ORBSLAM2, this check was not present and the reference KF was always updated
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
        vdPosePred_ms.push_back(timePosePred);
#endif


#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack = std::chrono::steady_clock::now();
#endif
        
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                bOK = TrackLocalMap(); /// < OKL
            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints and MapLines in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap(); /// < OKL
        }

        // < Detect objects 
        if(bOK && mbObjectTrackerOn)
        {        
            const std::vector<MapObjectPtr > &vpMObjs = mpAtlas->GetAllMapObjects();  // check all objects in the map          
            // < TODO: add parallel threads (or move everthing in a new thread dedicated to object tracking)
            for(size_t ii=0;ii<vpMObjs.size(); ii++)
            {
                const MapObjectPtr& pMObj = vpMObjs[ii];
                pMObj->Detect(&mCurrentFrame); 
                if( pMObj->IsLocalizedInCurrentFrame() || pMObj->IsOjectVisibleInCurrentFrame() )
                {
                    mCurrentFrame.mvpMapObjects.push_back(MapObjectPtr(pMObj)); // this is also used by the FrameDrawer
                }
            }
        }

        if(bOK)
        {
            mState = OK;
        }
        else if (mState == OK)
        {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Resetting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState = RECENTLY_LOST;
            }
            else
            {
                mState = RECENTLY_LOST; // visual to lost  // LOST 
            }

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            //}
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) &&
           (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETTING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_StartLMTrack).count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        mpFrameDrawer->Update(this);  /// < OKL
        if(mCurrentFrame.isSet())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            if(mLastFrame.isSet() && mCurrentFrame.isSet())
            {
                Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                mVelocity = mCurrentFrame.GetPose() * LastTwc;
                mbVelocity = true;
            }
            else {
                mbVelocity = false;
            }

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

            /// < Clean VO point matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPointPtr>(NULL);
                    }
            }
            
            /// < Clean VO line matches
            if(mbLineTrackerOn)
            {
                for(int i=0; i<mCurrentFrame.Nlines; i++)
                {
                    MapLinePtr pML = mCurrentFrame.mvpMapLines[i];
                    if(pML)
                        if(pML->Observations()<1)
                        {
                            mCurrentFrame.mvbLineOutlier[i] = false;
                            mCurrentFrame.mvpMapLines[i]=static_cast<MapLinePtr>(NULL);
                        }
                }
            }

            /// < Delete temporal MapPoints
            for(list<MapPointPtr>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPointPtr& pMP = *lit;
               
                //delete pMP;
                DeletePtr(pMP);
            }
            mlpTemporalPoints.clear();
            
            /// < Delete temporal MapLines
            if(mbLineTrackerOn)
            {
                for(list<MapLinePtr>::iterator lit = mlpTemporalLines.begin(), lend =  mlpTemporalLines.end(); lit!=lend; lit++)
                {
                    MapLinePtr& pML = *lit;
                    
                    //delete pML;
                    DeletePtr(pML);
                }
                mlpTemporalLines.clear();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();
#endif
            bool bNeedKF = NeedNewKeyFrame();

            /// < Check if we need to insert a new keyframe
            // if(bNeedKF && bOK)
            if(bNeedKF && (bOK || (mInsertKFsLost && mState==RECENTLY_LOST &&
                                   (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD))))
            {
                CreateNewKeyFrame();  /// < OKL
                
                // TODO: Luigi move it in an async task 
                if(mbObjectTrackerOn)
                {
                    const std::vector<MapObjectPtr > &vpMObjs = mCurrentFrame.mvpMapObjects;  // check all objects in the map                          
                    for(size_t ii=0;ii<vpMObjs.size(); ii++)
                    {
                        const MapObjectPtr& pMObj = vpMObjs[ii];
                        if( pMObj->IsLocalizedInCurrentFrame() )
                        {
                            pMObj->AddKeyFrameObservation(mpReferenceKF); 
                            mpReferenceKF->AddMapObject(pMObj);                        
                        }
                    }                    
                }
            }
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            /// < Clean outliers once keyframe generation has been managed 
            // We allow points and lines with high innovation (considered outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPointPtr>(NULL);
            }
            
            if(mbLineTrackerOn)
            {
                for(int i=0; i<mCurrentFrame.Nlines;i++)
                {
                    if(mCurrentFrame.mvpMapLines[i] && mCurrentFrame.mvbLineOutlier[i])
                        mCurrentFrame.mvpMapLines[i]=static_cast<MapLinePtr>(NULL);
                }                
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(pCurrentMap->KeyFramesInMap()<=10)
            {
                cout << "Track lost soon after initialisation, resetting active map..." << endl;
                mpSystem->ResetActiveMap();
                return;
            }
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::PrintMess("Track lost before IMU initialisation, resetting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            CreateMapInAtlas();

            return;  // NOTE: new exit point!
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }




    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(mCurrentFrame.isSet())
        {
            Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }

#ifdef REGISTER_LOOP
    if (Stop()) {

        // Safe area to stop
        while(isStopped())
        {
            usleep(3000);
        }
    }
#endif
}


void Tracking::StereoInitialization()
{
    const int numMinFeaturesForInit = (mSensor==System::RGBD)? skNumMinFeaturesRGBDInitialization : skNumMinFeaturesStereoInitialization;
    
    if( (mCurrentFrame.N + mCurrentFrame.Nlines) > numMinFeaturesForInit)
    {
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (!mFastInit && (mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA).norm()<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
        }
        else
            mCurrentFrame.SetPose(Sophus::SE3f());

        // Create KeyFrame
        //KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
        KeyFramePtr pKFini = KeyFrameNewPtr(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpAtlas->AddKeyFrame(pKFini);

        PushKeyFrameInPointCloudMapping(pKFini);

        // Create MapPoints and associate to KeyFrame
        if(!mpCamera2)
	    {
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    Eigen::Vector3f x3D;
                    mCurrentFrame.UnprojectStereo(i, x3D);
                    //MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
		            MapPointPtr pNewMP = MapPointNewPtr(x3D,pKFini,mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }

#if CREATE_NEW_LINES_ON_TRACKING  
            if(mbLineTrackerOn)
            {                  
                // Create MapLines and associate to KeyFrame
                for(int i=0; i<mCurrentFrame.Nlines;i++)
                {
                    const float& zS = mCurrentFrame.mvDepthLineStart[i];
                    const float& zE = mCurrentFrame.mvDepthLineEnd[i];
                    if( (zS>0) && (zE>0) )                
                    {                         
                        Eigen::Vector3f xs3D, xe3D; // start and end points 
                        if(mCurrentFrame.UnprojectStereoLine(i,xs3D,xe3D))
                        {                       
                            //MapLinePtr pNewLine = new MapLine(xs3D,xe3D,pKFini,mpAtlas->GetCurrentMap());
                            MapLinePtr pNewLine = MapLineNewPtr(xs3D,xe3D,pKFini,mpAtlas->GetCurrentMap());
                            
                            pNewLine->AddObservation(pKFini,i);
                            pKFini->AddMapLine(pNewLine,i);
                            pNewLine->ComputeDistinctiveDescriptors();
                            pNewLine->UpdateNormalAndDepth();
                            mpAtlas->AddMapLine(pNewLine);

                            mCurrentFrame.mvpMapLines[i]=pNewLine;          
                        }                                        
                    }
                }                     
            }    
#endif                  

        } else{

            if(mCurrentFrame.Nright>0)
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    Eigen::Vector3f x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    //MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
		            MapPointPtr pNewMP = MapPointNewPtr(x3D,pKFini,mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }

#if CREATE_NEW_LINES_ON_TRACKING  
            if(mbLineTrackerOn)
            {              
                // Create MapLines and associate to KeyFrame
                if(mCurrentFrame.NlinesRight>0)
                for(int i=0; i<mCurrentFrame.NlinesLeft;i++)
                {
                    int rightIndex = mCurrentFrame.mvLeftToRightLinesMatch[i];
                    if(rightIndex != -1)             
                    {                         
                        // at initialization time we don't need to transform with Twc
                        const Eigen::Vector3f xs3D = mCurrentFrame.mvStereo3DLineStartPoints[i]; 
                        const Eigen::Vector3f xe3D = mCurrentFrame.mvStereo3DLineEndPoints[i];                       
              
                        //MapLinePtr pNewLine = new MapLine(xs3D,xe3D,pKFini,mpAtlas->GetCurrentMap());
                        MapLinePtr pNewLine = MapLineNewPtr(xs3D,xe3D,pKFini,mpAtlas->GetCurrentMap());
                        
                        pNewLine->AddObservation(pKFini,i);
                        pNewLine->AddObservation(pKFini,rightIndex + mCurrentFrame.NlinesLeft);    

                        pKFini->AddMapLine(pNewLine,i);
                        pKFini->AddMapLine(pNewLine,rightIndex + mCurrentFrame.NlinesLeft);                        

                        pNewLine->ComputeDistinctiveDescriptors();
                        pNewLine->UpdateNormalAndDepth();
                        mpAtlas->AddMapLine(pNewLine);

                        mCurrentFrame.mvpMapLines[i]=pNewLine;         
                        mCurrentFrame.mvpMapLines[rightIndex + mCurrentFrame.NlinesLeft]=pNewLine;                                                              
                    }
                }                     
            }    
#endif                     
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points and " + to_string( mpAtlas->MapLinesInMap()) + " lines", Verbose::VERBOSITY_NORMAL);

        //cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        //mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        if(mbLineTrackerOn)
            mvpLocalMapLines = mpAtlas->GetAllMapLines();

        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
        if(mbLineTrackerOn)
            mpAtlas->SetReferenceMapLines(mvpLocalMapLines);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

        mState=OK;
    }
}


void Tracking::MonocularInitialization()
{
    /// < TODO: add support for line segments init 
    
    if(!mbReadyToInitializate)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            }

            mbReadyToInitializate = true;

            return;
        }
    }
    else
    {
        if (((int)mCurrentFrame.mvKeys.size()<=kNumMinFeaturesMonoInitialization)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
        {
            mbReadyToInitializate = false;

            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            mbReadyToInitializate = false;
            return;
        }

        Sophus::SE3f Tcw;
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Tcw,mvIniP3D,vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(Sophus::SE3f());
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}



void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    //KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFramePtr pKFini = KeyFrameNewPtr(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    //KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFramePtr pKFcur = KeyFrameNewPtr(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mSensor == System::IMU_MONOCULAR)
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);
    
    PushKeyFrameInPointCloudMapping(pKFini);
    PushKeyFrameInPointCloudMapping(pKFcur);

    // Create MapPoints and associate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        //MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());
	MapPointPtr pMP = MapPointNewPtr(worldPos,pKFcur,mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPointPtr> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    // Set median depth to a pre-fixed value 
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, resetting...", Verbose::VERBOSITY_NORMAL);
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPointPtr> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPointPtr pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }


    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    //mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    if(mbLineTrackerOn)
        mvpLocalMapLines = mpAtlas->GetAllMapLines();    
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFramePtr> vKFs = mpAtlas->GetAllKeyFrames();

    Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
    mbVelocity = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
    if(mbLineTrackerOn)
        mpAtlas->SetReferenceMapLines(mvpLocalMapLines);   

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;

    initID = pKFcur->mnId;
}

void Tracking::InitForRelocalizationInMap()
{
    // Set Frame pose to the origin
    //mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F)); 
    mCurrentFrame.SetPose(Sophus::SE3f()); 

    std::vector<KeyFramePtr> keyframes = mpAtlas->GetAllKeyFrames();

    KeyFramePtr pKFini = keyframes[0];
    //mpLocalMapper->InsertKeyFrame(pKFini);

    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;

    //mvpLocalKeyFrames.push_back(pKFini);

    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    if(mbLineTrackerOn)
        mvpLocalMapLines = mpAtlas->GetAllMapLines();

    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
    if(mbLineTrackerOn)
        mpAtlas->SetReferenceMapLines(mvpLocalMapLines);

    //mpAtlas->mvpKeyFrameOrigins.push_back(pKFini);

    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());  
    
    // setting we are recently lost to force relocalization 
    mState = RECENTLY_LOST;    
    mTimeStampLost = mCurrentFrame.mTimeStamp;
}

void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mbSetInit=false;

    mnInitialFrameId = mCurrentFrame.mnId+1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    //mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
    mbVO = false; // Init value for know if there are enough MapPoints in the last KF
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mbReadyToInitializate = false;
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFramePtr>(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFramePtr>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPointPtr pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPointPtr pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
    
    if(mbLineTrackerOn)
    {
        for(int i =0; i<mLastFrame.Nlines; i++)
        {
            MapLinePtr pML = mLastFrame.mvpMapLines[i];

            if(pML)
            {
                MapLinePtr pRep = pML->GetReplaced();
                if(pRep)
                {
                    mLastFrame.mvpMapLines[i] = pRep;
                }
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    std::cout << "Tracking::TrackReferenceKeyFrame()" << std::endl; 
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPointPtr> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    //if(nmatches<15)
    //{
    //    cout << "TRACK_REF_KF: Less than 15 matches!!\n";
    //    return false;
    //}

    int numLineMatches = 0;
    vector<MapLinePtr> vpMapLineMatches;
    
    // Line segments tracking 
    if(mbLineTrackerOn)
    {
        LineMatcher lineMatcher(0.7);

        numLineMatches = lineMatcher.SearchByKnn(mpReferenceKF, mCurrentFrame, vpMapLineMatches);
        
    }
    
    if(nmatches<15)
    {
        std::cout << "Tracking::TrackReferenceKeyFrame() - nmatches<15: relying on " << nmatches << " point matches and " << numLineMatches << " line matches " << std::endl; 
    }
    
    if( ( nmatches + sknLineTrackWeigth*numLineMatches ) < 15)
    {
        std::cout << "Tracking::TrackReferenceKeyFrame() - FAILURE - (nmatches + mnLineTrackWeigth*numLineMatches)<15: " << (nmatches + sknLineTrackWeigth*numLineMatches) << std::endl;
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.mvpMapLines = vpMapLineMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());

    //mCurrentFrame.PrintPointDistribution();


    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        //if(i >= mCurrentFrame.Nleft) break;
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPointPtr>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

   
    // Discard line outliers
    int nmatchesMapLines = 0;
    if(mbLineTrackerOn)
    {
        for(int i =0; i<mCurrentFrame.Nlines; i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLinePtr pML = mCurrentFrame.mvpMapLines[i];

                    mCurrentFrame.mvpMapLines[i]=static_cast<MapLinePtr>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView = false;
                    pML->mnLastFrameSeen = mCurrentFrame.mnId;
                    numLineMatches--;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nmatchesMapLines++;
            }
        }        
    }

    const int nmatchesMapFeatures = ( nmatchesMap + sknLineTrackWeigth*nmatchesMapLines );

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        return true;
    }
    else
    {
        //return nmatchesMap>=10;
    if(  nmatchesMapFeatures < 10 )
    {
        std::cout << "Tracking::TrackReferenceKeyFrame() - relying on " << nmatchesMap << " map point matches and " << nmatchesMapLines << " map line matches " << std::endl; 
        std::cout << "Tracking::TrackReferenceKeyFrame() - FAILURE - (nmatchesMap + mnLineTrackWeigth*nmatchesMapLines)<10: " << nmatchesMapFeatures << std::endl;
        return false;
    }
    
    return ( nmatchesMapFeatures >= 10 );
    }
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFramePtr pRef = mLastFrame.mpReferenceKF;
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for(int i=0; i<Nfeat;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPointPtr pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            Eigen::Vector3f x3D;

            if(mLastFrame.Nleft == -1)
            {
                mLastFrame.UnprojectStereo(i, x3D);
            }
            else
            {
                x3D = mLastFrame.UnprojectStereoFishEye(i);
            }

            //MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
	    MapPointPtr pNewMP = MapPointNewPtr(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
        {
            break;
        }
    }
}

void Tracking::UpdateLastFrameLines(bool updatePose)
{
    if(!mbLineTrackerOn) return; 
    
    if(updatePose)
    {
        // This is disabled by default since it is already performed in UpdateLastFrame()
        
        // Update pose according to reference keyframe
        KeyFramePtr pRef = mLastFrame.mpReferenceKF;
        Sophus::SE3f Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr*pRef->GetPose());
    }
    
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;
    
    // Create "visual odometry" MapLines
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.Nlines);
    for(int i=0; i<mLastFrame.Nlines;i++)
    {
        const float& zS = mLastFrame.mvDepthLineStart[i];
        const float& zE = mLastFrame.mvDepthLineEnd[i];
        if( (zS>0) && (zE>0) )
        {
            //const float z  = 0.5*(zS + zE);
            const float z  = std::max(zS,zE);
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close lines (depth<mThDepth)
    // If less than 100 close lines, we insert the 100 closest ones.
    int nLines = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapLinePtr pML = mLastFrame.mvpMapLines[i];
        if(!pML)
            bCreateNew = true;
        else if(pML->Observations()<1)
        {
            bCreateNew = true;
        }

#if CREATE_NEW_LINES_ON_TRACKING 
        if(bCreateNew)
        {             
            Eigen::Vector3f x3DStart,x3DEnd;
            bool unproject = false; 
            if(mLastFrame.NlinesLeft == -1)
            {
                unproject = mLastFrame.UnprojectStereoLine(i,x3DStart,x3DEnd);
            }
            else
            {
                unproject = mLastFrame.UnprojectStereoLineFishEye(i,x3DStart,x3DEnd);
            }
            if(unproject)
            {
                //MapLinePtr pNewML = new MapLine(x3DStart,x3DEnd,mpAtlas->GetCurrentMap(),&mLastFrame,i);
                MapLinePtr pNewML = MapLineNewPtr(x3DStart,x3DEnd,mpAtlas->GetCurrentMap(),&mLastFrame,i);

                mLastFrame.mvpMapLines[i]=pNewML;

                mlpTemporalLines.push_back(pNewML);
                nLines++;
            }      
        }
        else
#endif               
        {
            nLines++;
        }

        if(vDepthIdx[j].first>mThDepth && nLines>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();
    
    if(mbLineTrackerOn)
        UpdateLastFrameLines();


    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesn't need reset
        PredictStateIMU();
        return true;
    }
    else
    {
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }


    // track points 
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPointPtr>(NULL));

    // Project points seen in previous frame
    int th;

    if(mSensor==System::STEREO)
        th=7;
    else
        th=15;

    int nPointmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    bool bLargerLineSearch = false; 

    // If few matches, uses a wider window search
    if(nPointmatches<20)
    {
        bLargerLineSearch = true; 

        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPointPtr>(NULL));

        nPointmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nPointmatches), Verbose::VERBOSITY_NORMAL);
    }

    // track line segments
    int nLineMatches = 0;
    if(mbLineTrackerOn)
    {
        fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(),static_cast<MapLinePtr>(NULL));  
               
        LineMatcher lineMatcher(0.9);
        
#if USE_LINE_MATCHING_BY_KNN        
        nLineMatches = lineMatcher.SearchByKnn(mCurrentFrame, mLastFrame);
#else
        nLineMatches = lineMatcher.SearchByProjection(mCurrentFrame, mLastFrame, bLargerLineSearch/*bLarger*/,mSensor==System::MONOCULAR);
        if(nLineMatches<10 && !bLargerLineSearch)
        {
            //Verbose::PrintMess("Not enough line matches, larger threshold search!!", Verbose::VERBOSITY_NORMAL);
            fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(),static_cast<MapLinePtr>(NULL));  

            nLineMatches = lineMatcher.SearchByProjection(mCurrentFrame, mLastFrame,true/*bLarger*/,mSensor==System::MONOCULAR);
            //Verbose::PrintMess("Line Matches with larger threshold search: " + to_string(nLineMatches), Verbose::VERBOSITY_NORMAL);

        }
#endif        
    }

    if(nPointmatches<20)
    {
        std::cout << "Tracking::TrackWithMotionModel() - nPointmatches<20: relying on " << nPointmatches << " point matches and " << nLineMatches << " line matches " << std::endl; 
    }

    if(nPointmatches + sknLineTrackWeigth*nLineMatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
	    {
            return true;
        }
        else
        {
            std::cout << "Tracking::TrackWithMotionModel() - FAILURE - nPointmatches + sknLineTrackWeigth*nLineMatches<20: " << nPointmatches + sknLineTrackWeigth*nLineMatches << std::endl; 
            return false;
        }
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nMatchesMapPoints = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPointPtr>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nPointmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nMatchesMapPoints++;
        }
    }
 
    int nMatchesMapLines = 0;
    if(mbLineTrackerOn)
    {
        for(int i =0; i<mCurrentFrame.Nlines; i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLinePtr pML = mCurrentFrame.mvpMapLines[i];

                    mCurrentFrame.mvpMapLines[i]=static_cast<MapLinePtr>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView = false;
                    pML->mnLastFrameSeen = mCurrentFrame.mnId;
                    nLineMatches--;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nMatchesMapLines++;
            }
        }        
    }
    
    const int nMatchesMapFeatures = nMatchesMapPoints + sknLineTrackWeigth*nMatchesMapLines;
    if(mbOnlyTracking)
    {
        mbVO = nMatchesMapFeatures<10; // we have not tracked enough MapPoints and MapLines in the map
        return (nPointmatches + sknLineTrackWeigth*nLineMatches)>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        return true;
    }
    else
    {
        //return nmatchesMap>=10;
        if( nMatchesMapFeatures <10 )
        {
            std::cout << "Tracking::TrackWithMotionModel() - FAILURE - (nMatchesMapPoints + sknLineTrackWeigth*nMatchesMapLines)<10: " << nMatchesMapPoints + sknLineTrackWeigth*nMatchesMapLines << std::endl; 
            return false;
        }
        
        return nMatchesMapFeatures>=10;
    }
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points and map lines tracked in the frame.
    // We retrieve the local map and try to find matches to points and lines in the local map.

    mTrackedFr++;

    UpdateLocalMap();
    SearchLocalPoints();
    if(mbLineTrackerOn)
        SearchLocalLines();


#if 0    
    // TODO Luigi: add lines here?
    // TODO check outliers before PO
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }
#endif 
    
    int inliers;
    if (!mpAtlas->isImuInitialized())
    {
        Optimizer::PoseOptimization(&mCurrentFrame);
    }
    else
    {
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

#if 0   
    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }
#endif 
    
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPointPtr>(NULL);

        }
    }
    
    mnLineMatchesInliers = 0;
    
    // Update MapLines Statistics
    if(mbLineTrackerOn)
    {
        for(int i=0; i<mCurrentFrame.Nlines; i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(!mCurrentFrame.mvbLineOutlier[i])
                {
                    mCurrentFrame.mvpMapLines[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                            mnLineMatchesInliers++;
                    }
                    else
                        mnLineMatchesInliers++;
                }
                else if(mSensor==System::STEREO)                
                    mCurrentFrame.mvpMapLines[i] = static_cast<MapLinePtr>(NULL);
        
            }
        }
    }

    //std::cout << "track local map: point inliers: " << mnMatchesInliers << ", line inliers: " << mnLineMatchesInliers << std::endl; 
                
    const int nFeatureMatchesInliers = mnMatchesInliers + sknLineTrackWeigth*mnLineMatchesInliers;
    
    // Decide if the tracking was successful
    // More restrictive if there was a relocalization recently
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;

    //if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    //    return false;
    if( ( mCurrentFrame.mnId < mnLastRelocFrameId+mMaxFrames ) && ( nFeatureMatchesInliers < 50 ) )
    {
        std::cout << "Tracking::TrackLocalMap() - FAILURE after relocalization - mnMatchesInliers+mnLineTrackWeigth*sknLineTrackWeigth<50): " << nFeatureMatchesInliers << std::endl;        
        std::cout << "Tracking::TrackLocalMap() - FAILURE after relocalization - relying on mnMatchesInliers: " << mnMatchesInliers << ", mnLineMatchesInliers: " << mnLineMatchesInliers << std::endl;
        return false;
    }

    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;


    if (mSensor == System::IMU_MONOCULAR)
    {
        if((nFeatureMatchesInliers<15 && mpAtlas->isImuInitialized())||(nFeatureMatchesInliers<50 && !mpAtlas->isImuInitialized()))
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        if(nFeatureMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        //if(mnMatchesInliers<30)
        //    return false;
        //else
        //    return true;
        if( nFeatureMatchesInliers < 30 )
        {
            std::cout << "Tracking::TrackLocalMap() - FAILURE - mnMatchesInliers+mnLineTrackWeigth*mnLineMatchesInliers<30): " << nFeatureMatchesInliers << std::endl;         
            std::cout << "Tracking::TrackLocalMap() - FAILURE - relying on mnMatchesInliers: " << mnMatchesInliers << ", mnLineMatchesInliers: " << mnLineMatchesInliers << std::endl;
            return false;
        }
        else
        {
            return true;
        }
    }
}

#define USE_LINES_FOR_NEW_KEYFRAMES_GEN 1
#define USE_FOV_CENTER_CRITERION 1

bool Tracking::NeedNewKeyFrame()
{
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if ((mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefPointsMatches = mpReferenceKF->TrackedMapPoints(nMinObs);
    
#if USE_LINES_FOR_NEW_KEYFRAMES_GEN    
    int nRefLinesMatches = 0;
    if(mbLineTrackerOn) 
    {
        nRefLinesMatches = mpReferenceKF->TrackedMapLines(nMinObs);
    }
#else
    int nRefLinesMatches = 0; 
#endif
    
    const int nRefFeaturesMatches = nRefPointsMatches + Tracking::sknLineTrackWeigth*nRefLinesMatches;

    // Local Mapping accept keyframes?  
    //bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points and lines are being tracked and how many could be potentially created.
    int nPointsNonTrackedClose = 0;
    int nPointsTrackedClose= 0;    
    int nLinesNonTrackedClose = 0;
    int nLinesTrackedClose= 0;

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nPointsTrackedClose++;
                else
                    nPointsNonTrackedClose++;

            }
        }
        //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);
        
#if USE_LINES_FOR_NEW_KEYFRAMES_GEN          
        if(mbLineTrackerOn) 
        for(int i =0; i<mCurrentFrame.Nlines; i++)
        {
            if( 
                    (mCurrentFrame.mvDepthLineStart[i]>0 && mCurrentFrame.mvDepthLineStart[i]<mThDepth) &&
                    (mCurrentFrame.mvDepthLineEnd[i]>0 && mCurrentFrame.mvDepthLineEnd[i]<mThDepth) 
                    
              )      
            {
                if(mCurrentFrame.mvpMapLines[i] && !mCurrentFrame.mvbLineOutlier[i])
                    nLinesTrackedClose++;
                else
                    nLinesNonTrackedClose++;
            }
        }        
#endif
    }

    const int nFeaturesTrackedClose    = nPointsTrackedClose + Tracking::sknLineTrackWeigth*nLinesTrackedClose;
    const int nFeaturesNonTrackedClose = nPointsNonTrackedClose + Tracking::sknLineTrackWeigth*nLinesNonTrackedClose;
    bool bNeedToInsertClose = (  nFeaturesTrackedClose < 100 ) &&   // not enough tracked close features 
                              (  nFeaturesNonTrackedClose > 70 );   // enough close features which could be potentially tracked

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    /*int nClosedPoints = nTrackedClose + nNonTrackedClose;
    const int thStereoClosedPoints = 15;
    if(nClosedPoints < thStereoClosedPoints && (mSensor==System::STEREO || mSensor==System::IMU_STEREO))
    {
        //Pseudo-monocular, there are not enough close points to be confident about the stereo observations.
        thRefRatio = 0.9f;
    }*/

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;
    
#if USE_LINES_FOR_NEW_KEYFRAMES_GEN   
    if(!mbLineTrackerOn) mnLineMatchesInliers = 0;
    const int nFeaturesMatchesInliers = mnMatchesInliers + Tracking::sknLineTrackWeigth*mnLineMatchesInliers;
#else
    const int nFeaturesMatchesInliers = mnMatchesInliers;
#endif

    // Local Mapping accept keyframes?  
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

      // ORBSLAM2 
//    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
//    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
//    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
//    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
//    //Condition 1c: tracking is weak
//    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
//    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
//    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);
    
    // ORBSLAM3
    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    //const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    //const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR && mSensor!=System::IMU_STEREO && mSensor!=System::IMU_RGBD && (nFeaturesMatchesInliers<nRefFeaturesMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked features compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = (((nFeaturesMatchesInliers<nRefFeaturesMatches*thRefRatio || bNeedToInsertClose)) && nFeaturesMatchesInliers>15);

#if 1    
    const int maxNumOfKeyFramesInLocalMapperQueue = ( mbLineTrackerOn || mpPointCloudMapping ) ? 5 : 3; // 3 was originally used without lines and volumetric mapping; 
                                                                                       // now, we use an higher number (5>3) 
                                                                                       // since lines and volumetric mapping bring more computations and local mapper is always busier 
                                                                                       // when dealing with both points and lines  
#else
    const int maxNumOfKeyFramesInLocalMapperQueue = 3;
#endif 

    // Condition cFovCs: Current FOV center is distant more than "skMaxDistFovCenters" from last keyframe FOV center and we have a decent number of inliers 
    bool cFovCs = false;
#if USE_FOV_CENTER_CRITERION    
    if(mbUseFovCentersKfGenCriterion)
    {
        const Eigen::Vector3f currentFovCenter = mCurrentFrame.GetFovCenter();
        const Eigen::Vector3f lastKfFovCenter = mpLastKeyFrame->GetFovCenter();
        float distanceFovCs = (currentFovCenter - lastKfFovCenter).norm();
        cFovCs = (distanceFovCs > skMaxDistFovCenters) && (nFeaturesMatchesInliers>15);
    }
#endif    

    //std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    //std::cout << "F: " << mCurrentFrame.mnId << ", nFeaturesMatchesInliers: " << nFeaturesMatchesInliers << ", KF: " << mpReferenceKF->mnFrameId << ", RefFeaturesMatches: " << nRefFeaturesMatches << std::endl; 

    bool c4 = false;
    //if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
    //    c4=true;
    //else
    //    c4=false;
    if ((((nFeaturesMatchesInliers<75) && (nFeaturesMatchesInliers>15)) || mState==RECENTLY_LOST) && (mSensor == System::IMU_MONOCULAR)) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4 ||cFovCs)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle || mpLocalMapper->IsInitializing())
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                //if(mpLocalMapper->KeyframesInQueue()<3)
                if(mpLocalMapper->KeyframesInQueue()<maxNumOfKeyFramesInLocalMapperQueue)
                    return true;
                else
                    return false;
            }
            else
            {
                //std::cout << "NeedNewKeyFrame: localmap is busy" << std::endl;
                return false;
            }
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
        return;

    if(!mpLocalMapper->SetNotStop(true))
        return;

    //KeyFramePtr pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);
    KeyFramePtr pKF = KeyFrameNewPtr(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    //std::cout << "new KF " << pKF->mnFrameId << std::endl; 

    if(mpAtlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if include imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                {
                    bCreateNew = true;
                }
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPointPtr>(NULL);
                }

                if(bCreateNew)
                {
                    Eigen::Vector3f x3D;

                    if(mCurrentFrame.Nleft == -1){
                        mCurrentFrame.UnprojectStereo(i, x3D);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    //MapPointPtr pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    MapPointPtr pNewMP = MapPointNewPtr(x3D,pKF,mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }
            //Verbose::PrintMess("new MPs for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
        }

        if(mbLineTrackerOn)
        {
            // We sort lines by the measured depth by the stereo/RGBD sensor.
            // We create all those MapLines whose depth < mThDepth.
            // If there are more than 100 close points we create the 100 closest.
            vector<pair<float,int> > vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.Nlines);
            for(int i=0; i<mCurrentFrame.Nlines; i++)
            {
                const float& zS = mCurrentFrame.mvDepthLineStart[i];
                const float& zE = mCurrentFrame.mvDepthLineEnd[i];
                if( (zS>0) && (zE>0) )
                {
                    //const float z  = 0.5*(zS + zE);
                    const float z  = std::max(zS,zE);
                    vDepthIdx.push_back(make_pair(z,i));
                }
            }

            if(!vDepthIdx.empty())
            {
                sort(vDepthIdx.begin(),vDepthIdx.end());

                int nLines = 0;
                for(size_t j=0; j<vDepthIdx.size();j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapLinePtr pML = mCurrentFrame.mvpMapLines[i];
                    if(!pML)
                    {
                        bCreateNew = true;
                    }
                    else if(pML->Observations()<1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapLines[i] = static_cast<MapLinePtr>(NULL);
                    }

#if CREATE_NEW_LINES_ON_TRACKING 
                    if(bCreateNew)
                    {
                        Eigen::Vector3f x3DStart,x3DEnd;
                        bool unproject = false;
                        if(mCurrentFrame.NlinesLeft == -1)
                        {
                            unproject = mCurrentFrame.UnprojectStereoLine(i,x3DStart,x3DEnd);
                        }
                        else
                        {
                            unproject = mCurrentFrame.UnprojectStereoLineFishEye(i,x3DStart,x3DEnd);
                        }
                        if(unproject)
                        {
                            //MapLinePtr pNewML = new MapLine(x3DStart,x3DEnd,pKF,mpAtlas->GetCurrentMap());
                            MapLinePtr pNewML = MapLineNewPtr(x3DStart,x3DEnd,pKF,mpAtlas->GetCurrentMap());
                            
                            pNewML->AddObservation(pKF,i);
                            pKF->AddMapLine(pNewML,i);
                            pNewML->ComputeDistinctiveDescriptors();
                            pNewML->UpdateNormalAndDepth();
                            mpAtlas->AddMapLine(pNewML);

                            mCurrentFrame.mvpMapLines[i]=pNewML;
                            nLines++;
                        }
                    }
                    else
#endif                     
                    {
                        nLines++;
                    }

                    if(vDepthIdx[j].first>mThDepth && nLines>100)
                        break;
                }
            }
            
        }
    }

    //std::cout << "Tracking::CreateNewKeyFrame() - created new KF: "<< pKF->mnId << std::endl;    
    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);
    
    /// < Push new frame in PointCloudMapping
    PushKeyFrameInPointCloudMapping(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}


// push new frame in PointCloudMapping
void Tracking::PushKeyFrameInPointCloudMapping(KeyFramePtr& pKF)
{
    if(mpPointCloudMapping)
    {
        switch(mSensor) 
        {
            case System::RGBD:
            case System::IMU_RGBD:      
                {
                PointCloudKeyFrame<PointCloudMapping::PointT>::Ptr pcKeyframe(new PointCloudKeyFrame<PointCloudMapping::PointT>(pKF, this->mImRGB, this->mImDepth));
                mpPointCloudMapping->InsertKeyFrame(pcKeyframe);
                }
                break;
            case System::STEREO:
            case System::IMU_STEREO:                
                {
                PointCloudKeyFrame<PointCloudMapping::PointT>::Ptr pcKeyframe(new PointCloudKeyFrame<PointCloudMapping::PointT>(pKF, this->mImRGB, this->mImGray, this->mImgGrayRight));
                mpPointCloudMapping->InsertKeyFrame(pcKeyframe);
                }
                break;
            
            default:
                ; // nop
            
        }
    }    
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPointPtr>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPointPtr pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPointPtr>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPointPtr>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPointPtr pMP = *vit;

        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        // if(pMP->mbTrackInView)
        // {
        //     mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        // }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD || mSensor==System::IMU_RGBD)
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=6;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

// TODO: Luigi update according to the new changes above in SearchLocalPoints
void Tracking::SearchLocalLines()
{
    if(!mbLineTrackerOn) return; 
        
    // Do not search map lines already matched in current frame 
    for(vector<MapLinePtr>::iterator vit=mCurrentFrame.mvpMapLines.begin(), vend=mCurrentFrame.mvpMapLines.end(); vit!=vend; vit++)
    {
        MapLinePtr pML = *vit;
        if(pML)
        {
            if(pML->isBad())
            {
                *vit = static_cast<MapLinePtr>(NULL);
            }
            else
            {
                pML->IncreaseVisible();
                pML->mnLastFrameSeen = mCurrentFrame.mnId;
                pML->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project lines in frame and check its visibility
    for(vector<MapLinePtr>::iterator vit=mvpLocalMapLines.begin(), vend=mvpLocalMapLines.end(); vit!=vend; vit++)
    {
        MapLinePtr pML = *vit;
        if(pML->mnLastFrameSeen == mCurrentFrame.mnId) // lines already checked 
            continue;
        if(pML->isBad())
            continue;
        // Project (this fills MapLine variables for matching)
        if( mCurrentFrame.isInFrustum(pML,0.5) ) 
        {
            pML->IncreaseVisible();
            nToMatch++;
        }
        // if(pML->mbTrackInView)
        // {
        //     mCurrentFrame.mmProjectLines[pML->mnId] = std::make_pair(cv::Point2f(pML->mTrackProjStartX, pML->mTrackProjStartY),
        //                                                              cv::Point2f(pML->mTrackProjEndX, pML->mTrackProjEndY));        
        // }        
    }

    bool bLargerLineSearch = false; 

    if(mpAtlas->isImuInitialized())
    {
        if(!mpAtlas->GetCurrentMap()->GetIniertialBA2()) bLargerLineSearch=true;
    }
    else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO || mSensor == System::IMU_RGBD))
    {
        bLargerLineSearch=true;
    }

    // If the camera has been relocalised recently, perform a coarser search
    if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
        bLargerLineSearch=true;

    if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
        bLargerLineSearch=true;     

    if(nToMatch>0)
    {
        LineMatcher matcher(0.8);
        
#if USE_LINE_MATCHING_BY_KNN        
        matcher.SearchByKnn(mCurrentFrame, mvpLocalMapLines);
#else
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapLines, bLargerLineSearch);
#endif
    }
}


void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
    
    if(mbLineTrackerOn)
        mpAtlas->SetReferenceMapLines(mvpLocalMapLines);
    
    if(mbObjectTrackerOn)
        mpAtlas->SetReferenceMapObjects(mvpLocalMapObjects);    

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalFeatures();
    
}

void Tracking::UpdateLocalFeatures()
{
    mvpLocalMapPoints.clear();
    
    if(mbLineTrackerOn) mvpLocalMapLines.clear();
    if(mbObjectTrackerOn) mvpLocalMapObjects.clear();

    int count_pts = 0;

    for(vector<KeyFramePtr>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFramePtr pKF = *itKF;
        
        const vector<MapPointPtr> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPointPtr>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPointPtr pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
        
        if(mbLineTrackerOn) 
        {
            const vector<MapLinePtr> vpMLs = pKF->GetMapLineMatches();

            for(vector<MapLinePtr>::const_iterator itML=vpMLs.begin(), itEndML=vpMLs.end(); itML!=itEndML; itML++)
            {
                MapLinePtr pML = *itML;
                if(!pML)
                    continue;
                if(pML->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pML->isBad())
                {
                    mvpLocalMapLines.push_back(pML);
                    pML->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }            
        }
        
        if(mbObjectTrackerOn)
        {
            const vector<MapObjectPtr > vpMObjs = pKF->GetMapObjectMatches();

            for(vector<MapObjectPtr >::const_iterator itML=vpMObjs.begin(), itEndML=vpMObjs.end(); itML!=itEndML; itML++)
            {
                MapObjectPtr pMObj = *itML;
                if(!pMObj)
                    continue;
                if(pMObj->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pMObj->isBad())
                {
                    mvpLocalMapObjects.push_back(pMObj);
                    pMObj->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }             
        }
        
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point (of the current frame) votes for the keyframes in which it has been observed
    //map<KeyFramePtr,int> keyframeCounter;
    map<KeyFramePtr,float> keyframeCounter;
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    const map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFramePtr,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
	    // Each map line (of the current frame) votes for the keyframes in which it has been observed
	    const float lineWeight = Tracking::sknLineTrackWeigth;
	    if(mbLineTrackerOn)
	    {
		for(int i=0; i<mCurrentFrame.Nlines; i++)
		{
		    if(mCurrentFrame.mvpMapLines[i]) 
		    {
		        MapLinePtr pML = mCurrentFrame.mvpMapLines[i];
                        if(!pML) continue; 
		        if(!pML->isBad())
		        {
		            const map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();
		            for(map<KeyFramePtr,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
		                //keyframeCounter[it->first]++;
		                keyframeCounter[it->first] += lineWeight;
		        }
		        else
		        {
		            mCurrentFrame.mvpMapLines[i]=NULL;
		        }
		    }
		}
	    }
    }
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPointPtr pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFramePtr,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
	    // Each map line (of the last frame) votes for the keyframes in which it has been observed
	    const float lineWeight = Tracking::sknLineTrackWeigth;
	    if(mbLineTrackerOn)
	    {
		for(int i=0; i<mLastFrame.Nlines; i++)
		{
		    if(mLastFrame.mvpMapLines[i]) 
		    {
		        MapLinePtr pML = mLastFrame.mvpMapLines[i];
                        if(!pML) continue; 
		        if(!pML->isBad())
		        {
		            const map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();
		            for(map<KeyFramePtr,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
		                //keyframeCounter[it->first]++;
		                keyframeCounter[it->first] += lineWeight;
		        }
		        else
		        {
		            mLastFrame.mvpMapLines[i]=NULL;
		        }
		    }
		}
	    }
    }

    // < TODO: add objects votes?

    int max=0;
    KeyFramePtr pKFmax= static_cast<KeyFramePtr>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point or a map line are included in the local map. Also check which keyframe shares most points and lines
    for(map<KeyFramePtr,float>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFramePtr pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFramePtr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size() > kMaxNumOfKeyframesInLocalMap)
            break;

        KeyFramePtr pKF = *itKF;

        const vector<KeyFramePtr> vNeighs = pKF->GetBestCovisibilityKeyFrames( kNumBestCovisibilityKeyFrames );
        for(vector<KeyFramePtr>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFramePtr pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFramePtr> spChilds = pKF->GetChilds();
        for(set<KeyFramePtr>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFramePtr pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFramePtr pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    // Add 10 last temporal KFs (mainly for IMU)
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD) &&mvpLocalKeyFrames.size()<80)
    {
        KeyFramePtr tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

/// < TODO: add line processing here 
bool Tracking::Relocalization()
{
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFramePtr> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    if(vpCandidateKFs.empty()) 
    {
        Verbose::PrintMess("Relocalized FAILURE - There are no candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector<vector<MapPointPtr> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFramePtr pKF = vpCandidateKFs[i];
        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
        }
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,6,0.5,5.991);  //This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool bTcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(bTcw)
            {
                Sophus::SE3f Tcw(eigTcw);
                mCurrentFrame.SetPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPointPtr> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPointPtr>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        std::cout << "Relocalization FAILURE!" << std::endl; 
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }

}

void Tracking::Reset(bool bLocMap)
{
    cout << "\nTracking Resetting ->" << endl;
    Verbose::PrintMess("System Resetting", Verbose::VERBOSITY_NORMAL);

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Resetting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Resetting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Resetting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_RGBD)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFramePtr>(NULL);
    mpLastKeyFrame = static_cast<KeyFramePtr>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End resetting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    Verbose::PrintMess("Active map Resetting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Resetting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Resetting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Resetting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    //Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    //mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; //NOT_INITIALIZED;

    mbReadyToInitializate = false;

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    //cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFramePtr>(NULL);
    mpLastKeyFrame = static_cast<KeyFramePtr>(NULL);
    mvIniMatches.clear();

#if 1
    // Luigi: added to avoid segmentation fault on map Stereo-Intertial map reloading when map resets are called. 
    // Without these clear() calls we get a segmentation fault in UpdateFrameIMU(): some of the KFs get lost in the resets above.  
    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
#endif 

    mbVelocity = false;

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End resetting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPointPtr> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

vector<MapLinePtr> Tracking::GetLocalMapMLS()
{
    return mvpLocalMapLines;
}

//TODO: [Luigi] do we need to update the adopted camera model too? 
void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    mK_.setIdentity();
    mK_(0,0) = fx;
    mK_(1,1) = fy;
    mK_(0,2) = cx;
    mK_(1,2) = cy;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
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
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
    
    if(mpCamera->GetType() == GeometricCamera::CAM_PINHOLE)
    {
        vector<float> vCamCalib{fx,fy,cx,cy};
        mpCamera->setParameters(vCamCalib);
    }
    else if (mpCamera->GetType() == GeometricCamera::CAM_FISHEYE)
    {
        std::cout << "TODO: this has not been implemented yet!" << std::endl;
        assert(false);
    }
}

//TODO: [Luigi] do we need to update the adopted camera model too? 
void Tracking::SetCameraCalibration(const float fx, const float fy, const float cx, const float cy, const cv::Mat& DistCoef, const float bf)
{
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    /*cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }*/
    DistCoef.copyTo(mDistCoef);

    mbf = bf;

    Frame::mbInitialComputations = true;
    
    if(mpCamera->GetType() == GeometricCamera::CAM_PINHOLE)
    {
        vector<float> vCamCalib{fx,fy,cx,cy};
        mpCamera->setParameters(vCamCalib);
    }
    else if (mpCamera->GetType() == GeometricCamera::CAM_FISHEYE)
    {
        std::cout << "TODO: this has not been implemented yet!" << std::endl;
        assert(false);
    }
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFramePtr pCurrentKeyFrame)
{
    if(!pCurrentKeyFrame) return; 

    Map * pMap = pCurrentKeyFrame->GetMap();
    MSG_ASSERT((bool)pMap,"Current map must be non-null!");
    
    std::cout << "Tracking::UpdateFrameIMU() - update on map " << pMap->GetId() << std::endl;     
    //std::cout << "Tracking::UpdateFrameIMU() - start loop" << std::endl; 
    
    unsigned int index = mnFirstFrameId;
    list<PLVS2::KeyFramePtr>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    //std::cout << "mlbLost.size(): " << mlbLost.size() << std::endl; 
    //std::cout << "mlRelativeFramePoses.size(): " << mlRelativeFramePoses.size() << std::endl;
    //std::cout << "mlpReferences.size(): " << mlpReferences.size() << std::endl;
    MSG_ASSERT(mlbLost.size() == mlRelativeFramePoses.size(),"Must be of the same size!");
    MSG_ASSERT(mlbLost.size() == mlpReferences.size(),"Must be of the same size!");
    for(auto lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFramePtr pKF = *lRit;
        if(!pKF) 
        {
            std::cout << "WARNING: Tracking::UpdateFrameIMU() - null KF" << std::endl;             
            continue; 
        }

        while(pKF && pKF->isBad())
        {
            if(pKF == pKF->GetParent())
            {
                std::cout << "WARNING: Tracking::UpdateFrameIMU() - loop with parent" << std::endl;
                break;  
            }                
            pKF = pKF->GetParent();
            if(!pKF) 
            {
                std::cout << "WARNING: Tracking::UpdateFrameIMU() - null parent" << std::endl; 
            }            
        }

        if(pKF && pKF->GetMap() == pMap)
        {
            (*lit).translation() *= s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mLastFrame.mpImuPreintegrated->dT; // NOTE: [Luigi] this is conversion from double to float!

        mLastFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame.mpImuPreintegrated->dT; // NOTE: [Luigi] this is conversion from double to float!

        mCurrentFrame.SetImuPoseVelocity(IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder)
{
    mpSystem->SaveTrajectoryEuRoC(strFolder + strNameFile_frames);
    //mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap)
{
    mpSystem->SaveTrajectoryEuRoC(strNameFile_frames, pMap);
    if(!strNameFile_kf.empty())
        mpSystem->SaveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

const float Tracking::GetImageScale() const
{
    return mImageScale;
}

#ifdef REGISTER_LOOP
void Tracking::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
}

bool Tracking::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Tracking STOP" << endl;
        return true;
    }

    return false;
}

bool Tracking::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool Tracking::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

void Tracking::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
    mbStopRequested = false;
}
#endif

} // namespace PLVS2
