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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "Viewer.h"
#include "Utils.h" 

#include "PointCloudMapping.h"
#include "PointCloudDrawer.h"
#include "Stopwatch.h"

#define ENABLE_LOOP_CLOSURE 1


namespace PLVS
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer):
mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
mbDeactivateLocalizationMode(false), mnSaveMapCount(0), mbSaveMap(true), mbPaused(false)
{
    // Output welcome message
    cout << endl <<
    "PLVS" << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }
    
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bBinVocLoaded = false;    
#if ENABLE_LOOP_CLOSURE
    bool bVocLoad = false;  // chose loading method based on file extension
    std::string vocabularyFileNameBin = Utils::getFileNameWithouExtension(strVocFile) + ".bin";
    if( Utils::fileExist(vocabularyFileNameBin) )
    {
        bVocLoad = mpVocabulary->loadFromBinaryFile(vocabularyFileNameBin);
        bBinVocLoaded = true;
    }
    else
    {
        if (Utils::hasSuffix(strVocFile, ".txt"))
        {
            bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        }
        else if (Utils::hasSuffix(strVocFile, ".bin"))
        {
            bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
            bBinVocLoaded = true;            
        }
        else
        {
            bVocLoad = false;
        }
    }
    
    if(!bVocLoad)   
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
#endif
    std::string binString = bBinVocLoaded ? "(binary)" : "";
    cout << "Vocabulary " << binString << " loaded!" << endl << endl;

    // load/save map parameters 
    mStrMapfile = Utils::GetParam(fsSettings, "SparseMapping.filename", std::string());     
    mbSaveMap = static_cast<int> (Utils::GetParam(fsSettings, "SparseMapping.saveMap", 0)) != 0;
    bool bReuseMap = static_cast<int> (Utils::GetParam(fsSettings, "SparseMapping.reuseMap", 0)) != 0;
    bool bForceRelocalizationInMap = static_cast<int> (Utils::GetParam(fsSettings, "SparseMapping.forceRelocalization", 0)) != 0;    
    bool bFreezeMap = static_cast<int> (Utils::GetParam(fsSettings, "SparseMapping.freezeMap", 0)) != 0;      
    bool bMapLoaded = false;
    if( !mStrMapfile.empty() && bReuseMap && LoadMap(mStrMapfile) ) 
    {
        if(bFreezeMap)
        {
            std::vector<KeyFramePtr> keyframes = mpMap->GetAllKeyFrames();
            for(size_t ii=0, iiEnd=keyframes.size(); ii<iiEnd; ii++) keyframes[ii]->mbFixed = true;        
        }
        
        cout << "Loading available map: " << mStrMapfile << endl; 
        bMapLoaded = true; 
    } 
    else 
    {
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Map
        mpMap = new Map();
    }
    
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&PLVS::LocalMapping::Run,mpLocalMapper);

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    
    bool bPointCloudMappingActive = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.on", 0, false)) != 0;
    if( bPointCloudMappingActive )
    {
        mpPointCloudMapping = make_shared<PointCloudMapping>(strSettingsFile, mpMap, mpLocalMapper);
        //mpPointCloudMapping->setDepthFactor(fsSettings["DepthMapFactor"]);

        mpPointCloudDrawer = make_shared<PointCloudDrawer>();
        mpPointCloudDrawer->SetPointCloudMapping(mpPointCloudMapping);
    }

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
    
    if( bMapLoaded && bForceRelocalizationInMap )
    {
        mpTracker->mState = Tracking::RELOCALIZE_IN_LOADED_MAP; // force relocalization in the loaded map         
    }
    
    if( bPointCloudMappingActive )
    {    
        mpTracker->SetPointCloudMapping(mpPointCloudMapping);
    }
    

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
#if ENABLE_LOOP_CLOSURE    
    mptLoopClosing = new thread(&PLVS::LoopClosing::Run, mpLoopCloser);
#endif

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        if( bPointCloudMappingActive )
        {           
            mpViewer->SetPointCloudDrawer(mpPointCloudDrawer);
        }
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
    
    /// < signals connection 
    
    if(mpPointCloudMapping) 
    {
        mpLoopCloser->mSignalGlobalBundleAdjustmentFinished.connectMember(mpPointCloudMapping.get(), &PointCloudMapping::RebuildMap);
    }
    
}

System::~System()
{
    //std::cout << "System::~System() - start" << std::endl;       
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    if (mbPaused) {
        cv::Mat pause_mat;
        return pause_mat;
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    
    mTrackedMapLines = mpTracker->mCurrentFrame.mvpMapLines;
    mTrackedKeyLinesUn = mpTracker->mCurrentFrame.mvKeyLinesUn;    
    
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    if (mbPaused) {
        cv::Mat pause_mat;
        return pause_mat;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        ResetPointCloudMapping();
        
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);
    
    /*if(MapChanged())
    {
        cout << "MapChanged() - resetting the point cloud" << endl;
        //RebuildPointCloudMapping(); // replaced by signal connection of mSignalGlobalBundleAdjustmentFinished
    }*/

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    
    mTrackedMapLines = mpTracker->mCurrentFrame.mvpMapLines;
    mTrackedKeyLinesUn = mpTracker->mCurrentFrame.mvKeyLinesUn;    
    
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }
    
    if (mbPaused) {
        cv::Mat pause_mat;
        return pause_mat;
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    std::cout << "System::Reset()" << std::endl;     
    mbReset = true;
}

void System::Shutdown()
{
    std::cout << "System::Shutdown() - start" << std::endl; 
    
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    
    mpPointCloudDrawer.reset(); // let the viewer destroy the point cloud drawer 
    
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
        
        delete mpViewer;
        mpViewer = static_cast<Viewer*>(NULL);
    }
        
    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
    
    std::cout << "System::Shutdown() - point cloud mapping shutdown..." << std::endl;
    if(mpPointCloudMapping)
    {
        mpPointCloudMapping->Shutdown(); /// < TODO: check if this is the right point where to shutdown the point mapper 
        while(!mpPointCloudMapping->IsFinished()) 
            usleep(1000);
    }
    std::cout << "System::Shutdown() - done" << std::endl;

    if(mpViewer)
    {
        std::cout << "System::Shutdown() - binding to context" << std::endl;
        pangolin::BindToContext(Viewer::kMapWindowName);
    }

    if (mbSaveMap) SaveMap(mStrMapfile);    

    std::cout << "System::Shutdown() - end" << std::endl;     
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFramePtr> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<PLVS::KeyFramePtr>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFramePtr pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFramePtr> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFramePtr> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<PLVS::KeyFramePtr>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        PLVS::KeyFramePtr pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPointPtr> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

std::vector<MapLinePtr> System::GetTrackedMapLines()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapLines;
}

std::vector<cv::line_descriptor_c::KeyLine> System::GetTrackedKeyLinesUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyLinesUn;
}    

void System::ResetPointCloudMapping()
{
    if(mpPointCloudMapping) mpPointCloudMapping->Reset();
}


void System::RebuildPointCloudMapping()
{
    if(mpPointCloudMapping) mpPointCloudMapping->RebuildMap();
}


void System::StartGlobalBundleAdjustment()
{
    mpLoopCloser->StartGlobalBundleAdjustment();
}


void System::EnableLoopClosing() 
{
    unique_lock< mutex > lock(mpMap->mMutexLoopClosing);
    mpMap->mnEnableLoopClosing = true;
}

void System::DisableLoopClosing() 
{
    unique_lock< mutex > lock(mpMap->mMutexLoopClosing);
    mpMap->mnEnableLoopClosing = false;
}

void System::SetPause(bool val)
{
    mbPaused = val;    
}

void System::TogglePause() 
{ 
    mbPaused = !mbPaused;
}

void System::SaveMap() 
{
    unique_lock< mutex > lock(mMutexReset);
    
    std::string tmpFilename( Utils::getFileNameWithouExtension(mStrMapfile) );
    if( tmpFilename.empty() ) tmpFilename = "sparse_map";
    
    std::stringstream map_name;
    map_name << tmpFilename << "_out_" << mnSaveMapCount++ << ".map";   
    
    SaveMap( map_name.str() );
}

void System::SaveMap(const string &filename) 
{
    if( filename.empty() ) 
    {
        std::cerr << "filename empty" << std::endl; 
        return; 
    }
    
    std::ofstream out(filename, std::ios_base::binary);
    if (!out) 
    {
        std::cerr << "Cannot Write to Mapfile: " << filename << std::endl;
        exit(-1);
    }
    std::cout << "Saving map file: " << filename << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    out.close();
    std::cout << " ...done" << std::endl << std::flush;    
    
    mpMap->printStatistics();
}

bool System::LoadMap(const string &filename) 
{
    std::cout << "Loading sparse map ..." << std::endl; 
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) 
    {
        std::cerr << "Cannot open sparse map file: " << filename << " , creating a new one..." << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << filename << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector< PLVS::KeyFramePtr> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it : vpKFS) 
    {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId) mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl << std::flush;
    in.close();
    
    mpMap->printStatistics();
        
    return true;
}

void System::PrintMapStatistics()
{
    mpMap->printStatistics();    
}

void System::SetCalibration(const float fx, const float fy, const float cx, const float cy, const cv::Mat& distCoef, const float bf)
{
    mpTracker->SetCalibration(fx, fy, cx, cy, distCoef, bf);   
    if(mpViewer) mpViewer->SetCameraCalibration(fx, fy, cx, cy);
    if(mpPointCloudMapping) mpPointCloudMapping->SetCameraCalibration(fx, fy, cx, cy);
}


} //namespace PLVS
