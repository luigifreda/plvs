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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "GeometricTools.h"

#include "MapLine.h"
#include "MapObject.h"
#include "LineMatcher.h"
#include "Utils.h"

#if RERUN_ENABLED
#include "RerunSingleton.h" 
#endif 

#include<mutex>
#include<chrono>

#define TRIANGULATE_NEW_LINES 1
#define ASSIGN_VIRTUAL_DISPARITY_WHEN_TRIANGULATING_LINES 1
#define VISUALIZE_LINE_MATCHES 0 && RERUN_ENABLED

namespace PLVS2
{

LocalMapping::LocalMapping(System* pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName):
    mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
    mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), 
    mIdxIteration(0), infoInertial(Eigen::MatrixXd::Zero(9,9))
{
    mnMatchesInliers = 0;

    mbBadImu = false;

    mTinit = 0.f;

    mNumLM = 0;
    mNumKFCulling=0;

#ifdef REGISTER_TIMES
    nLBA_exec = 0;
    nLBA_abort = 0;
#endif

}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{
    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mbBadImu)
        {
#ifdef REGISTER_TIMES
            double timeLBA_ms = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

            double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif

            // Check recent MapPoints
            MapPointCulling();

            // Check recent MapLines
            MapLineCulling();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

            double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif

            // Triangulate new MapPoints
            CreateNewMapFeatures();

            mbAbortBA = false;

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point and line duplications
                SearchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

            double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            bool b_doneLBA = false;
            int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_MLs_BA = 0;            
            int num_MOs_BA = 0;                     
            int num_edges_BA = 0;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                if(mpAtlas->KeyFramesInMap()>2)
                {

                    if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                    {
                        float dist = (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()).norm() +
                                (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter()).norm();

                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                        {
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                cout << "Not enough motion for initializing. Resetting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mpMapToReset = mpCurrentKeyFrame->GetMap();
                                mbBadImu = true;
                            }
                        }

                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
                        Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                        b_doneLBA = true;
                    }
                    else
                    {
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(), num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_MLs_BA, num_edges_BA);
                        b_doneLBA = true;
                    }

                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

                if(b_doneLBA)
                {
                    timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLBA - time_EndMPCreation).count();
                    vdLBA_ms.push_back(timeLBA_ms);

                    nLBA_exec += 1;
                    if(mbAbortBA)
                    {
                        nLBA_abort += 1;
                    }
                    vnLBA_edges.push_back(num_edges_BA);
                    vnLBA_KFopt.push_back(num_OptKF_BA);
                    vnLBA_KFfixed.push_back(num_FixedKF_BA);
                    vnLBA_MPs.push_back(num_MPs_BA);
                }

#endif

                // Initialize IMU here
                if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                {
                    if (mbMonocular)
                        InitializeIMU(1e2, 1e10, true);
                    else
                        InitializeIMU(1e2, 1e5, true);
                }


                // Check redundant local Keyframes
                KeyFrameCulling();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
                vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

                if ((mTinit<50.0f) && mbInertial)
                {
                    if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK) // Enter here everytime local-mapping is called
                    {
                        if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                cout << "start VIBA 1" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                if (mbMonocular)
                                    InitializeIMU(1.f, 1e5, true);
                                else
                                    InitializeIMU(1.f, 1e5, true);

                                cout << "end VIBA 1" << endl;
                            }
                        }
                        else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){
                                cout << "start VIBA 2" << endl;
                                mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                if (mbMonocular)
                                    InitializeIMU(0.f, 0.f, true);
                                else
                                    InitializeIMU(0.f, 0.f, true);

                                cout << "end VIBA 2" << endl;
                                
                                mSignalVIBAFinished.emit(); 
                            }
                        }

                        // scale refinement
                        if (((mpAtlas->KeyFramesInMap())<=200) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            cout << "start scale refinement" << endl;
                            if (mbMonocular)
                                ScaleRefinement();
                            cout << "end scale refinement" << endl;
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            vdLBASync_ms.push_back(timeKFCulling_ms);
            vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

            double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif
        }
        else if(Stop() && !mbBadImu)
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is free
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFramePtr pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

 void LocalMapping::AddNewKeyFramesToSet(std::set<KeyFramePtr>& set)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    set.insert(mlNewKeyFrames.begin(),mlNewKeyFrames.end());
    set.insert(mpCurrentKeyFrame);
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPointPtr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPointPtr pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    
    
    // Associate MapLines to the new keyframe and update normal and descriptor
    if(mpTracker->IsLineTracking())
    {
        // Associate MapLines to the new keyframe and update normal and descriptor
        const vector<MapLinePtr> vpMapLineMatches = mpCurrentKeyFrame->GetMapLineMatches();

        for(size_t i=0; i<vpMapLineMatches.size(); i++)
        {
            MapLinePtr pML = vpMapLineMatches[i];
            if(pML)
            {
                if(!pML->isBad())
                {
                    if(!pML->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        pML->AddObservation(mpCurrentKeyFrame, i);
                        pML->UpdateNormalAndDepth();
                        pML->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo lines inserted by the Tracking
                    {
                        mlpRecentAddedMapLines.push_back(pML);
                    }
                }
            }
        }         
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPointPtr>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPointPtr pMP = *lit;

        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3) // after three keyframes we do not consider the point a recent one
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
        }
    }
}

void LocalMapping::MapLineCulling()
{
    if(!mpTracker->IsLineTracking()) return; 
    
    // Check Recent Added MapLines
    list<MapLinePtr>::iterator lit = mlpRecentAddedMapLines.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
    {
        nThObs = 2;
    }
    else
    {
        nThObs = 3; // starting setting
    }
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapLines.end())
    { 
        MapLinePtr pML = *lit;
        if(pML->isBad())
        {
            lit = mlpRecentAddedMapLines.erase(lit);
        }
        else if(pML->GetFoundRatio()<0.25f )
        //else if(pML->GetFoundRatio()<0.1f ) 
        {
            //std::cout << "line eliminated since low pML->GetFoundRatio(): " << pML->GetFoundRatio() << std::endl; 
            pML->SetBadFlag();
            lit = mlpRecentAddedMapLines.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pML->mnFirstKFid)>=2 && pML->Observations()<=cnThObs)
        {
            //std::cout << "line eliminated since low pML->Observations(): " << pML->Observations() << std::endl; 
            pML->SetBadFlag();
            lit = mlpRecentAddedMapLines.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pML->mnFirstKFid)>=3) // after three keyframes we do not consider the line a recent one
        {
            lit = mlpRecentAddedMapLines.erase(lit);
        }
        else
        {
            lit++;
        }
    }
}

void LocalMapping::CreateNewMapFeatures()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    // For stereo inertial case
    if(mbMonocular)
        nn=30;
    vector<KeyFramePtr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    if (mbInertial)
    {
        KeyFramePtr pKF = mpCurrentKeyFrame;
        int count=0;
        while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
        {
            vector<KeyFramePtr>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if(it==vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th,false);
    std::unique_ptr<LineMatcher> pLineMatcher;
    if(mpTracker->IsLineTracking())
    {
        pLineMatcher.reset( new LineMatcher(0.6,false) );
    }

    Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
    Eigen::Matrix<float,3,4> eigTcw1 = sophTcw1.matrix3x4();
    Eigen::Matrix<float,3,3> Rcw1 = eigTcw1.block<3,3>(0,0);
    Eigen::Matrix<float,3,3> Rwc1 = Rcw1.transpose();
    Eigen::Vector3f tcw1 = sophTcw1.translation();
    Eigen::Vector3f twc1 = -Rwc1*tcw1;
    Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float fx1R = mpCurrentKeyFrame->mpCamera2 ? mpCurrentKeyFrame->mpCamera2->getLinearParameter(0) : 0.f;
    const float fy1R = mpCurrentKeyFrame->mpCamera2 ? mpCurrentKeyFrame->mpCamera2->getLinearParameter(1) : 0.f; 
    const float cx1R = mpCurrentKeyFrame->mpCamera2 ? mpCurrentKeyFrame->mpCamera2->getLinearParameter(2) : 0.f;
    const float cy1R = mpCurrentKeyFrame->mpCamera2 ? mpCurrentKeyFrame->mpCamera2->getLinearParameter(3) : 0.f; 
    const float invfx1R = mpCurrentKeyFrame->mpCamera2 ? 1.f/fx1R : 0.f;
    const float invfy1R = mpCurrentKeyFrame->mpCamera2 ? 1.f/fy1R : 0.f;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
    int nnewPoints=0;
    
#if TRIANGULATE_NEW_LINES       
    const Eigen::Matrix3f K1L = mpCurrentKeyFrame->mpCamera->toLinearK_();
    const Eigen::Matrix3f K1R = mpCurrentKeyFrame->mpCamera2 ? mpCurrentKeyFrame->mpCamera2->toLinearK_() : Eigen::Matrix3f();    
    const float linesRatioFactor = 1.5f*mpCurrentKeyFrame->mfLineScaleFactor;    
    int nnewLines=0; 
    size_t nlineTotMatches = 0;

    const Eigen::Matrix3f Rwc1L = Rwc1; 
    const Eigen::Vector3f twc1L = twc1; 
    const Sophus::SE3f Twc1R = mpCurrentKeyFrame->mpCamera2 ? sophTcw1.inverse() * mpCurrentKeyFrame->GetRelativePoseTlr() : Sophus::SE3f();
    const Eigen::Matrix3f Rwc1R = mpCurrentKeyFrame->mpCamera2 ? Twc1R.rotationMatrix() : Eigen::Matrix3f();
    const Eigen::Vector3f twc1R = mpCurrentKeyFrame->mpCamera2 ? Twc1R.translation() : Eigen::Vector3f();


#if VISUALIZE_LINE_MATCHES
    auto getUndistortedColorImages = [&](const KeyFramePtr& pKF, cv::Mat& imageLeftUnColor, cv::Mat& imageRightUnColor)
    {
        MSG_ASSERT(!pKF->imgLeft.empty(),"You probably need to set KEEP_IMGAGES to 1 in Frame.cc in order to store images into keyframes");
        cv::Mat imageLeftUn, imageRightUn; 
        if(pKF->mpCamera->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            const cv::Mat D = pKF->mpCamera->getDistortionParams();
            const cv::Mat K = pKF->mpCamera->toK();
            const cv::Mat Knew = pKF->mpCamera->toLinearK();
            cv::fisheye::undistortImage(pKF->imgLeft, imageLeftUn, K, D, Knew);

            const cv::Mat D2 = pKF->mpCamera2->getDistortionParams();
            const cv::Mat K2 = pKF->mpCamera2->toK();
            const cv::Mat K2new = pKF->mpCamera2->toLinearK();
            cv::fisheye::undistortImage(pKF->imgRight, imageRightUn, K2, D2, K2new);
            cv::cvtColor(imageLeftUn, imageLeftUnColor, cv::COLOR_GRAY2BGR);
            cv::cvtColor(imageRightUn, imageRightUnColor, cv::COLOR_GRAY2BGR);
        }
        else if(pKF->mpCamera->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            cv::undistort(pKF->imgLeft, imageLeftUn, pKF->mpCamera->toK(),pKF->mDistCoef, pKF->mpCamera->toK());
            cv::cvtColor(imageLeftUn, imageLeftUnColor, cv::COLOR_GRAY2BGR);
        }

    };

    auto& rec = RerunSingleton::instance();

    cv::Mat currImageLeftUnColor, currImageRightUnColor;
    getUndistortedColorImages(mpCurrentKeyFrame, currImageLeftUnColor, currImageRightUnColor);    
    //rec.log("debug/curr_KF_left", rerun::Image(tensor_shape(currImageLeftUnColor), rerun::TensorBuffer::u8(currImageLeftUnColor)));   
    //rec.log("debug/curr_KF_right", rerun::Image(tensor_shape(currImageRightUnColor), rerun::TensorBuffer::u8(currImageRightUnColor)));     
#endif

#endif 

    //int countStereo = 0;
    //int countStereoGoodProj = 0;
    //int countStereoAttempt = 0;
    //int totalStereoPts = 0;
    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFramePtr pKF2 = vpNeighKFs[i];

        GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3f vBaseline = Ow2-Ow1;
        const float baseline = vBaseline.norm();

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        bool bCoarse = mbInertial && mpTracker->mState==Tracking::RECENTLY_LOST && mpCurrentKeyFrame->GetMap()->GetIniertialBA2();

        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,vMatchedIndices,false,bCoarse);

        Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
        Eigen::Matrix<float,3,4> eigTcw2 = sophTcw2.matrix3x4();
        Eigen::Matrix<float,3,3> Rcw2 = eigTcw2.block<3,3>(0,0);
        Eigen::Matrix<float,3,3> Rwc2 = Rcw2.transpose();
        Eigen::Vector3f tcw2 = sophTcw2.translation();
        Eigen::Vector3f twc2 = -Rwc2*tcw2;

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

#if TRIANGULATE_NEW_LINES   
        const float fx2R = pKF2->mpCamera2 ? pKF2->mpCamera2->getLinearParameter(0) : 0.f;
        const float fy2R = pKF2->mpCamera2 ? pKF2->mpCamera2->getLinearParameter(1) : 0.f; 
        const float cx2R = pKF2->mpCamera2 ? pKF2->mpCamera2->getLinearParameter(2) : 0.f;
        const float cy2R = pKF2->mpCamera2 ? pKF2->mpCamera2->getLinearParameter(3) : 0.f; 
        const float invfx2R = pKF2->mpCamera2 ? 1.f/fx2R : 0.f;
        const float invfy2R = pKF2->mpCamera2 ? 1.f/fy2R : 0.f;

        const Eigen::Matrix3f Rwc2L = Rwc2; 
        const Eigen::Vector3f twc2L = twc2;
        const Sophus::SE3f Twc2R = pKF2->mpCamera2 ? sophTcw2.inverse() * pKF2->GetRelativePoseTlr() : Sophus::SE3f();        
        const Eigen::Matrix3f Rwc2R = pKF2->mpCamera2 ? Twc2R.rotationMatrix() : Eigen::Matrix3f();
        const Eigen::Vector3f twc2R = pKF2->mpCamera2 ? Twc2R.translation() : Eigen::Vector3f();         


#if VISUALIZE_LINE_MATCHES
    cv::Mat kf2ImageLeftUnColor, kf2ImageRightUnColor;
    getUndistortedColorImages(pKF2, kf2ImageLeftUnColor, kf2ImageRightUnColor);    
    //rec.log("debug/KF" + std::to_string(i) + "_left", rerun::Image(tensor_shape(kf2ImageLeftUnColor), rerun::TensorBuffer::u8(kf2ImageLeftUnColor)));   
    //rec.log("debug/KF" + std::to_string(i) + "_right", rerun::Image(tensor_shape(kf2ImageRightUnColor), rerun::TensorBuffer::u8(kf2ImageRightUnColor))); 
#endif           

#endif 

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                                                         : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]
                                                                                                               : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0);
            const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false
                                                                                                         : true;

            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){
                if(bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                    Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    sophTcw1 = mpCurrentKeyFrame->GetPose();
                    Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
                eigTcw1 = sophTcw1.matrix3x4();
                Rcw1 = eigTcw1.block<3,3>(0,0);
                Rwc1 = Rcw1.transpose();
                tcw1 = sophTcw1.translation();

                eigTcw2 = sophTcw2.matrix3x4();
                Rcw2 = eigTcw2.block<3,3>(0,0);
                Rwc2 = Rcw2.transpose();
                tcw2 = sophTcw2.translation();
            }

            // Check parallax between rays
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            //if (bStereo1 || bStereo2) totalStereoPts++;
            
            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj = false;
            bool bPointStereo = false;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
                                                                          (cosParallaxRays<0.9996 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
            {
                goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                if(!goodProj)
                    continue;
            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                //countStereoAttempt++;
                bPointStereo = true;
                goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                //countStereoAttempt++;
                bPointStereo = true;
                goodProj = pKF2->UnprojectStereo(idx2, x3D);
            }
            else
            {
                continue; //No stereo and very low parallax
            }

            // if(goodProj && bPointStereo)
            //     countStereoGoodProj++;

            if(!goodProj)
                continue;

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3D)+tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3D)+tcw1(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;

                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3D)+tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3D)+tcw2(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            Eigen::Vector3f normal1 = x3D - Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesful
            MapPointPtr pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
            //if (bPointStereo)
            //    countStereo++;
            
            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnewPoints++;
        }
        
#if TRIANGULATE_NEW_LINES         
        if(mpTracker->IsLineTracking())
        {
            //Eigen::Matrix3f F12;
            // Compute Fundamental Matrix
            //auto F12 = ComputeF12_(mpCurrentKeyFrame,pKF2);
            //F12 = ComputeF12(mpCurrentKeyFrame, pKF2); 
            //ComputeF12(mpCurrentKeyFrame, pKF2, F12, H12, e1);     

            Eigen::Matrix3f H21_LL, H21_LR, H21_RL, H21_RR; 
            Eigen::Vector3f e2_LL, e2_LR, e2_RL, e2_RR;

            ComputeH21(mpCurrentKeyFrame, pKF2, H21_LL, e2_LL, false/*bRight1*/, false/*bRight2*/); 
            if(pKF2->mpCamera2)
            {
                ComputeH21(mpCurrentKeyFrame, pKF2, H21_LR, e2_LR, false/*bRight1*/, true/*bRight2*/);
                ComputeH21(mpCurrentKeyFrame, pKF2, H21_RL, e2_RL, true/*bRight1*/, false/*bRight2*/);
                ComputeH21(mpCurrentKeyFrame, pKF2, H21_RR, e2_RR, true/*bRight1*/, true/*bRight2*/);
            }       
            
            const Eigen::Matrix3f K2L = pKF2->mpCamera->toLinearK_(); // pKF2->mK;
            const Eigen::Matrix3f K2R = pKF2->mpCamera2 ? pKF2->mpCamera2->toLinearK_() : Eigen::Matrix3f::Zero();
            const float minZ = pKF2->mb; // baseline in meters  
            const float maxZ = Tracking::skLineStereoMaxDist;   
                
            // Search line matches 
            std::vector<pair<size_t,size_t> > vMatchedLineIndices;
            pLineMatcher->SearchForTriangulation(mpCurrentKeyFrame,pKF2,vMatchedLineIndices,false);
            
#if VISUALIZE_LINE_MATCHES
            cv::Mat lineImageMatching;
            const auto& vKeyLinesLeftKF = mpCurrentKeyFrame->mvKeyLinesUn;
            const auto& vpKeyLinesLeftKF2 = pKF2->mvKeyLinesUn;            
            std::vector<cv::DMatch> vMatches;
            vMatches.reserve(vMatchedLineIndices.size());
            for( size_t ii = 0; ii < vMatchedLineIndices.size(); ii++ ) vMatches.push_back({vMatchedLineIndices[ii].first, vMatchedLineIndices[ii].second, 0.}); 
            std::vector<char> lineMatchingMask( vMatchedLineIndices.size(),1);            
            if(mpCurrentKeyFrame->mpCamera2 && !vMatchedLineIndices.empty())
            {
                const auto& vKeyLinesRightKF = mpCurrentKeyFrame->mvKeyLinesRightUn;
                const auto& vpKeyLinesRightKF2 = pKF2->mvKeyLinesRightUn;
                cv::line_descriptor_c::drawLineMatchesStereo(currImageLeftUnColor, currImageRightUnColor, vKeyLinesLeftKF, vKeyLinesRightKF, mpCurrentKeyFrame->NlinesLeft,
                                                             kf2ImageLeftUnColor, kf2ImageRightUnColor, vpKeyLinesLeftKF2, vpKeyLinesRightKF2, pKF2->NlinesLeft, 
                                                             vMatches, lineImageMatching, cv::Scalar::all( -1 ), cv::Scalar::all( -1 ), lineMatchingMask, cv::line_descriptor_c::DrawLinesMatchesFlags::DEFAULT);
            }
            else 
            {
                cv::line_descriptor_c::drawLineMatches(currImageLeftUnColor, vKeyLinesLeftKF, kf2ImageLeftUnColor, vpKeyLinesLeftKF2, vMatches, lineImageMatching, 
                                           cv::Scalar::all( -1 ), cv::Scalar::all( -1 ), lineMatchingMask, cv::line_descriptor_c::DrawLinesMatchesFlags::DEFAULT);                       
            }
            rec.log("debug/loc_mapping_image_line_stereo_matching" + std::to_string(i), rerun::Image(tensor_shape(lineImageMatching), rerun::TensorBuffer::u8(lineImageMatching)));     

#endif 

            // Triangulate each line match
            const size_t nlineMatches = vMatchedLineIndices.size();
            nlineTotMatches += nlineMatches;

            //std::cout << "lines triangulation, matched " << nlineMatches << " lines " << std::endl; 
            for(size_t ikl=0; ikl<nlineMatches; ikl++)
            {
                const int &idx1 = vMatchedLineIndices[ikl].first;
                const int &idx2 = vMatchedLineIndices[ikl].second;              
            
		        // get undistorted kl1 
                const auto& kl1 = (mpCurrentKeyFrame->NlinesLeft == -1) ? mpCurrentKeyFrame->mvKeyLinesUn[idx1] :
                                                                          (idx1 < mpCurrentKeyFrame->NlinesLeft) ? mpCurrentKeyFrame->mvKeyLinesUn[idx1] :
                                                                                                                   mpCurrentKeyFrame->mvKeyLinesRightUn[idx1 - mpCurrentKeyFrame->NlinesLeft];
                const float sigma1 = sqrt( mpCurrentKeyFrame->mvLineLevelSigma2[kl1.octave] );  

                // NOTE: With fisheye cameras, mvuRightLineStart and mvuRightLineEnd values cannot be directly used, however if >0 they signal the availability of the depths.  
                const bool bStereo1 = (!mpCurrentKeyFrame->mvuRightLineStart.empty()) && (mpCurrentKeyFrame->mvuRightLineStart[idx1]>=0) && (mpCurrentKeyFrame->mvuRightLineEnd[idx1]>=0);                

                const bool bRight1 = (mpCurrentKeyFrame->NlinesLeft == -1 || idx1 < mpCurrentKeyFrame->NlinesLeft) ? false : true;             
                const auto& K1 = bRight1 ? K1R : K1L; 
                
                // left image original index or left index corresponding to left-right stereo match if any (valid if >=0)
                const int idx1L = !bRight1 ? idx1 : mpCurrentKeyFrame->mvRightToLeftLinesMatch[idx1-mpCurrentKeyFrame->NlinesLeft]; // get the corresponding left line if any  

		        // get undistorted kl2 
                const auto& kl2 = (pKF2->NlinesLeft == -1) ? pKF2->mvKeyLinesUn[idx2] :
                                                             (idx2 < pKF2->NlinesLeft) ? pKF2->mvKeyLinesUn[idx2] :
                                                                                         pKF2->mvKeyLinesRightUn[idx2 - pKF2->NlinesLeft];
                const float sigma2 = sqrt( pKF2->mvLineLevelSigma2[kl2.octave] );       
                
                // NOTE: With fisheye cameras, mvuRightLineStart and mvuRightLineEnd values cannot be directly used, however if >0 they signal the availability of the depths.  
                const bool bStereo2 = (!pKF2->mvuRightLineStart.empty()) && (pKF2->mvuRightLineStart[idx2]>=0) && (pKF2->mvuRightLineEnd[idx2]>=0);  
                
                const bool bRight2 = (pKF2->NlinesLeft == -1 || idx2 < pKF2->NlinesLeft) ? false : true;
                const auto& K2 = bRight2 ? K2R : K2L;

                // left image original index or left index corresponding to left-right stereo match if any (valid if >=0)
                const int idx2L = !bRight2 ? idx2 : pKF2->mvRightToLeftLinesMatch[idx2-pKF2->NlinesLeft]; 

                const Eigen::Vector3f p1(kl1.startPointX , kl1.startPointY, 1.0);
                const Eigen::Vector3f q1(kl1.endPointX ,   kl1.endPointY, 1.0); 
                const Eigen::Vector3f m1 = 0.5*(p1+q1);
                Eigen::Vector3f l1 = p1.cross(q1);   
                const float l1Norm = sqrt( Utils::Pow2(l1[0]) + Utils::Pow2(l1[1]) );
                l1 = l1/l1Norm; // in this way we have l1 = (nx, ny, -d) with (nx^2 + ny^2) = 1
                                    
                const Eigen::Vector3f p2(kl2.startPointX, kl2.startPointY, 1.0);
                const Eigen::Vector3f q2(kl2.endPointX, kl2.endPointY, 1.0);
                const Eigen::Vector3f m2 = 0.5*(p2+q2);
                Eigen::Vector3f l2 = p2.cross(q2); 
                const float l2Norm = sqrt( Utils::Pow2(l2[0]) + Utils::Pow2(l2[1]) );
                l2 = l2/l2Norm; // in this way we have l2 = (nx, ny, -d) with (nx^2 + ny^2) = 1             
                
                // Check if we can triangulate, i.e. check if the normals of the two planes corresponding to lines are not parallel
                bool bCanTriangulateLines = true;
                Eigen::Vector3f n1 = K1.transpose()*l1; n1 /= n1.norm();
                Eigen::Vector3f n2 = K2.transpose()*l2; n2 /= n2.norm();

                const auto& Rwc1 = bRight1 ? Rwc1R : Rwc1L;
                const auto& twc1 = bRight1 ? twc1R : twc1L;
                const auto& Rwc2 = bRight2 ? Rwc2R : Rwc2L;
                const auto& twc2 = bRight2 ? twc2R : twc2L;

                Eigen::Vector3f n1w = Rwc1*n1;
                Eigen::Vector3f n2w = Rwc2*n2;
                const float normalsDotProduct= fabs( n1w.dot(n2w) );
                const float sigma = std::max( sigma1, sigma2 );
                const float dotProductThreshold = (mpCurrentKeyFrame->NlinesLeft == -1) ? 0.002 * sigma : 0.005 * sigma;// 0.002 // 0.005 //Frame::kLineNormalsDotProdThreshold * sigma; // this is a percentage over unitary modulus ( we modulate threshold by sigma)
                //const float dotProductThreshold = Frame::kLineNormalsDotProdThreshold * sigma; // this is a percentage over unitary modulus ( we modulate threshold by sigma)
                if( fabs( normalsDotProduct - 1.f ) < dotProductThreshold) 
                {
                    bCanTriangulateLines = false; // normals are almost parallel => cannot triangulate lines
                    //std::cout << "  -cannot triangulate n1: " << n1w.transpose() << ", n2: " << n2w.transpose() << std::endl;                     
                }
                
                // Check parallax between rays backprojecting the middle points 
                const Eigen::Vector3f xm1 = bRight1 ? Eigen::Vector3f((m1[0]-cx1R)*invfx1R, (m1[1]-cy1R)*invfy1R, 1.0) : 
                                                      Eigen::Vector3f((m1[0]-cx1)*invfx1,   (m1[1]-cy1)*invfy1,   1.0);
                const Eigen::Vector3f xm2 = bRight2 ? Eigen::Vector3f((m2[0]-cx2R)*invfx2R, (m2[1]-cy2R)*invfy2R, 1.0) : 
                                                      Eigen::Vector3f((m2[0]-cx2)*invfx2,   (m2[1]-cy2)*invfy2,   1.0);

                const Eigen::Vector3f ray1 = Rwc1*xm1;
                const Eigen::Vector3f ray2 = Rwc2*xm2;
                const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm()*ray2.norm());                

                float cosParallaxStereo = cosParallaxRays+1; // +1 since in case we do not have stereo we have cosParallaxRays<cosParallaxStereo at line 937
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;
                
                if(bStereo1 && idx1L>=0)
                {
                    const float depthM1 = 0.5* ( mpCurrentKeyFrame->mvDepthLineStart[idx1L] + mpCurrentKeyFrame->mvDepthLineEnd[idx1L] ); // depth middle point left
                    cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,depthM1));
                }
                else if(bStereo2 && idx2L>=0)
                {
                    const float depthM2 = 0.5* ( pKF2->mvDepthLineStart[idx2L] + pKF2->mvDepthLineEnd[idx2L] ); // depth middle point right                   
                    cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,depthM2));
                }

                cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

                Eigen::Vector3f x3DS, x3DE;
                bool bLineTriangulatedByIntersection = false;
                if( bCanTriangulateLines && (cosParallaxRays<cosParallaxStereo) && (cosParallaxRays>0) && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
                {
                    const auto& e2 = bRight1 ? (bRight2 ? e2_RR : e2_RL) : 
                                              (bRight2 ? e2_LR : e2_LL); 
                    const auto& H21 = bRight1 ? (bRight2 ? H21_RR : H21_RL) : 
                                               (bRight2 ? H21_LR : H21_LL);

                    // compute the intersections of rays backprojected from camera 1 with the 3D plane corresponding to l2  
                    // (check PLVS report)                                        
                    const float num = -l2.dot(e2);
                    const float den1 = (l2.dot(H21*p1)); // distance point-line in pixels 
                    const float den2 = (l2.dot(H21*q1)); // distance point-line in pixels 
                    
                    constexpr float denTh = 5;
                    if( ( fabs(den1) < denTh ) || (fabs(den2) < denTh) ) continue; 
                        
                    const float depthP1 = num/den1;
                    const float depthQ1 = num/den2;

                    if( (depthP1 >= minZ ) && (depthQ1 >= minZ) && (depthP1 <= maxZ) && (depthQ1 <= maxZ) )
                    {
                        const Eigen::Vector3f x3DSc = bRight1 ? Eigen::Vector3f( (p1[0]-cx1R)*invfx1R*depthP1, (p1[1]-cy1R)*invfy1R*depthP1, depthP1 ) : // depthP1 * K1R.inv()*p1;
                                                                Eigen::Vector3f( (p1[0]-cx1)*invfx1*depthP1,   (p1[1]-cy1)*invfy1*depthP1,   depthP1 );  // depthP1 * K1.inv()*p1;
                        x3DS = Rwc1*x3DSc + twc1;
                                                
                        const Eigen::Vector3f x3DEc = bRight1 ? Eigen::Vector3f( (q1[0]-cx1R)*invfx1R*depthQ1, (q1[1]-cy1R)*invfy1R*depthQ1, depthQ1 ) : // depthQ1 * K1R.inv()*q1;
                                                                Eigen::Vector3f( (q1[0]-cx1)*invfx1*depthQ1,   (q1[1]-cy1)*invfy1*depthQ1,   depthQ1 );  // depthQ1 * K1.inv()*q1;  
                        x3DE = Rwc1*x3DEc + twc1;

                        Eigen::Vector3f camRay = x3DSc.normalized();
                        Eigen::Vector3f lineES = x3DSc-x3DEc;  
                        const double lineLength = lineES.norm();
                        if(lineLength >= Frame::skMinLineLength3D)
                        {        
                            lineES /= lineLength;
                            const float cosViewAngle = fabs((float)camRay.dot(lineES));
                            if(cosViewAngle<=Frame::kCosViewZAngleMax)
                            {                    
            #if ASSIGN_VIRTUAL_DISPARITY_WHEN_TRIANGULATING_LINES
                                if(!bStereo1) 
                                {
                                    if(!mpCurrentKeyFrame->mpCamera2)
                                    {
                                        // assign depth and (virtual) disparity to left line end points 
                                        mpCurrentKeyFrame->mvDepthLineStart[idx1] = depthP1;
                                        const double disparity_p1 = mpCurrentKeyFrame->mbf/depthP1;                                
                                        mpCurrentKeyFrame->mvuRightLineStart[idx1] =  p1(0) - disparity_p1; 

                                        mpCurrentKeyFrame->mvDepthLineEnd[idx1] = depthQ1;
                                        const double disparity_q1 = mpCurrentKeyFrame->mbf/depthQ1;                         
                                        mpCurrentKeyFrame->mvuRightLineEnd[idx1] = q1(0) - disparity_q1; 
                                    }
                                    else
                                    {
                                        // assign depth and (virtual) disparity to left line end points      
                                        if(idx1L>=0)
                                        {
                                            mpCurrentKeyFrame->mvDepthLineStart[idx1L] = depthP1;                             
                                            mpCurrentKeyFrame->mvuRightLineStart[idx1L] =  1; // fake value to signal depth availability 
              
                                            mpCurrentKeyFrame->mvDepthLineEnd[idx1L] = depthQ1;                        
                                            mpCurrentKeyFrame->mvuRightLineEnd[idx1L] = 1; // // fake value to signal depth availability      
                                        }                                   
                                    }
                                }
            #endif                         
                                bLineTriangulatedByIntersection = true; 
                                //std::cout << "triangulated line : " << x3DS << " , " << x3DE << std::endl; 
                            }
                        }
                    }    
                }

#if 0
                if(if(mpCurrentKeyFrame->mpCamera2 && bLineTriangulatedByIntersection)
                {
                    // Just for debugging 
                    Sophus::SE3<float> Tcw1 = mpCurrentKeyFrame->GetPose();
                    const auto projectedX3DS = mpCurrentKeyFrame->mpCamera->project(Tcw1*x3DS);
                    const auto projectedX3DE = mpCurrentKeyFrame->mpCamera->project(Tcw1*x3DE);
                    const auto diffS = (projectedX3DS - p1.head<2>()).norm();
                    const auto diffE = (projectedX3DE - q1.head<2>()).norm();
                    if(diffS>1.0f || diffE>1.0f)
                    {
                        std::cout << "Warning: triangulated line is not close to line end points - diffS: " << diffS << " diffE: " << diffE << std::endl;
                    }
                }
#endif 

                if(!bLineTriangulatedByIntersection)
                {
                    if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
                    {
                        if(!mpCurrentKeyFrame->UnprojectStereoLine(idx1, x3DS, x3DE)) continue;   
                        //std::cout << "unprojected1 line : " << x3DS << " , " << x3DE << std::endl; 
                    }
                    else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
                    {
                        if(!pKF2->UnprojectStereoLine(idx2, x3DS, x3DE)) continue; 
                        //std::cout << "unprojected2 line : " << x3DS << " , " << x3DE << std::endl;                         
                    }
                    else
                    {
                        continue; //No stereo and very low parallax
                    }

                    // N.B.1: if we enter here we do not have fisheye cameras 
                    //MSG_ASSERT(!mpCurrentKeyFrame->mpCamera2, "Invalid camera type - It should be pinhole or fisheye!");

                    // N.B.2: if line is triangulated, the following block of checks are automatically satisfied by construction 
                    //(given the backprojection of the points p2 q2 from frame 2 on the plane corresponding to l1)

                    const Eigen::Vector3f x3DSt = x3DS.transpose();
                    const Eigen::Vector3f x3DEt = x3DE.transpose(); 

                    //Check triangulation in front of cameras
                    const float sz1 = Rcw1.row(2).dot(x3DSt)+tcw1[2];
                    if(sz1<=0)
                        continue;
                    const float ez1 = Rcw1.row(2).dot(x3DEt)+tcw1[2];
                    if(ez1<=0)
                        continue;                

                    const float sz2 = Rcw2.row(2).dot(x3DSt)+tcw2[2];
                    if(sz2<=0)
                        continue;
                    const float ez2 = Rcw2.row(2).dot(x3DEt)+tcw2[2];
                    if(ez2<=0)
                        continue;                

                    //Check reprojection error in first keyframe
                    const float &sigmaSquare1 = mpCurrentKeyFrame->mvLineLevelSigma2[kl1.octave];

                    const float sx1 = Rcw1.row(0).dot(x3DSt)+tcw1[0];
                    const float sy1 = Rcw1.row(1).dot(x3DSt)+tcw1[1];
                    const float sinvz1 = 1.0/sz1;

                    const float su1 = fx1*sx1*sinvz1+cx1;
                    const float sv1 = fy1*sy1*sinvz1+cy1;
                    const float dl1s = l1[0] * su1 + l1[1] * sv1 + l1[2]; // distance point-line
                    if((dl1s*dl1s)>3.84*sigmaSquare1)
                        continue;


                    const float ex1 = Rcw1.row(0).dot(x3DEt)+tcw1[0];
                    const float ey1 = Rcw1.row(1).dot(x3DEt)+tcw1[1];
                    const float einvz1 = 1.0/ez1;         

                    const float eu1 = fx1*ex1*einvz1+cx1;
                    const float ev1 = fy1*ey1*einvz1+cy1;
                    const float dl1e = l1[0] * eu1 + l1[1] * ev1 + l1[2]; // distance point-line
                    if((dl1e*dl1e)>3.84*sigmaSquare1)
                        continue;                

                    //Check reprojection error in second keyframe                                          
                    const float sigmaSquare2 = pKF2->mvLineLevelSigma2[kl2.octave];

                    const float sx2 = Rcw2.row(0).dot(x3DSt)+tcw2[0];
                    const float sy2 = Rcw2.row(1).dot(x3DSt)+tcw2[1];
                    const float sinvz2 = 1.0/sz2;         

                    const float su2 = fx2*sx2*sinvz2+cx2;
                    const float sv2 = fy2*sy2*sinvz2+cy2;
                    const float dl2s = l2[0] * su2 + l2[1] * sv2 + l2[2];
                    if((dl2s*dl2s)>3.84*sigmaSquare2)
                        continue;


                    const float ex2 = Rcw2.row(0).dot(x3DEt)+tcw2[0];
                    const float ey2 = Rcw2.row(1).dot(x3DEt)+tcw2[1];
                    const float einvz2 = 1.0/ez2;       

                    float eu2 = fx2*ex2*einvz2+cx2;
                    float ev2 = fy2*ey2*einvz2+cy2;
                    const float dl2e = l2[0] * eu2 + l2[1] * ev2 + l2[2];
                    if((dl2e*dl2e)>3.84*sigmaSquare2)
                        continue;                    
                }

                //Check scale consistency
                Eigen::Vector3f x3DM = 0.5*(x3DS+x3DE);
                
                Eigen::Vector3f normal1 = x3DM-Ow1;
                float dist1 = normal1.norm();

                Eigen::Vector3f normal2 = x3DM-Ow2;
                float dist2 = normal2.norm();

                if(dist1==0 || dist2==0)
                    continue;

                const float linesRatioDist = dist2/dist1;
                const float linesRatioOctave = mpCurrentKeyFrame->mvLineScaleFactors[kl1.octave]/pKF2->mvLineScaleFactors[kl2.octave];

                if(linesRatioDist*linesRatioFactor<linesRatioOctave || linesRatioDist>linesRatioOctave*linesRatioFactor)
                    continue;                
            
                // Triangulation is successful
                //MapLinePtr pML = new MapLine(x3DS,x3DE,mpCurrentKeyFrame,mpAtlas->GetCurrentMap());
                MapLinePtr pML = MapLineNewPtr(x3DS,x3DE,mpCurrentKeyFrame,mpAtlas->GetCurrentMap());
 
                pML->AddObservation(mpCurrentKeyFrame,idx1);
                mpCurrentKeyFrame->AddMapLine(pML,idx1);                    
                
                pML->AddObservation(pKF2,idx2);
                pKF2->AddMapLine(pML,idx2);

                pML->ComputeDistinctiveDescriptors();

                pML->UpdateNormalAndDepth();

                mpAtlas->AddMapLine(pML);
                mlpRecentAddedMapLines.push_back(pML);

                nnewLines++;                                
            }     
                        
        }
#endif        
        
    }
    
#if TRIANGULATE_NEW_LINES    
    if(nnewLines > 0)
    {
        //std::cout << "triangulated " << nnewPoints << " points "  <<" and " <<  nnewLines << " lines "  << std::endl;     
        std::cout << "triangulated " <<  nnewLines << " lines "  << " (tot matches: " << nlineTotMatches << ")" << std::endl; 
    }
#endif
    
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=30;
    const vector<KeyFramePtr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFramePtr> vpTargetKFs;
    for(vector<KeyFramePtr>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFramePtr pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for(int i=0, imax=vpTargetKFs.size(); i<imax; i++)
    {
        const vector<KeyFramePtr> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        for(vector<KeyFramePtr>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFramePtr pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA)
            break;
    }

    // Extend to temporal neighbors
    if(mbInertial)
    {
        KeyFramePtr pKFi = mpCurrentKeyFrame->mPrevKF;
        while(vpTargetKFs.size()<20 && pKFi)
        {
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId)
            {
                pKFi = pKFi->mPrevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
            pKFi = pKFi->mPrevKF;
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPointPtr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFramePtr>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFramePtr pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
        if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,3.0/*th*/,true);
    }


    if (mbAbortBA)
        return;

    // Search matches by projection from target KFs in current KF
    vector<MapPointPtr> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFramePtr>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFramePtr pKFi = *vitKF;

        vector<MapPointPtr> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPointPtr>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPointPtr pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);
    if(mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,3.0/*th*/,true);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPointPtr pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }
    
    
    
    // line tracking part 
    if(mpTracker->IsLineTracking()) 
    {
        // Search line matches by projection from current KF in target KFs
        LineMatcher lineMatcher;
        vector<MapLinePtr> vpMapLineMatches = mpCurrentKeyFrame->GetMapLineMatches();
        for(vector<KeyFramePtr>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
        {
            KeyFramePtr pKFi = *vit;

            lineMatcher.Fuse(pKFi,vpMapLineMatches);
            if(pKFi->NlinesLeft != -1) lineMatcher.Fuse(pKFi,vpMapLineMatches,3.0/*th*/,true);
        }
    
        if (mbAbortBA)
            return;

        // Search matches by projection from target KFs in current KF
        vector<MapLinePtr> vpFuseLineCandidates;
        vpFuseLineCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

        for(vector<KeyFramePtr>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
        {
            KeyFramePtr pKFi = *vitKF;

            vector<MapLinePtr> vpMapLinesKFi = pKFi->GetMapLineMatches();

            for(vector<MapLinePtr>::iterator vitML=vpMapLinesKFi.begin(), vendML=vpMapLinesKFi.end(); vitML!=vendML; vitML++)
            {
                MapLinePtr pML = *vitML;
                if(!pML)
                    continue;
                if(pML->isBad() || pML->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;
                pML->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseLineCandidates.push_back(pML);
            }
        }

        lineMatcher.Fuse(mpCurrentKeyFrame,vpFuseLineCandidates);
        if(mpCurrentKeyFrame->NlinesLeft != -1) lineMatcher.Fuse(mpCurrentKeyFrame,vpFuseLineCandidates,3.0/*th*/,true);

        // Update Lines
        vpMapLineMatches = mpCurrentKeyFrame->GetMapLineMatches();
        for(size_t i=0, iend=vpMapLineMatches.size(); i<iend; i++)
        {
            MapLinePtr pML=vpMapLineMatches[i];
            if(pML)
            {
                if(!pML->isBad())
                {
                    pML->ComputeDistinctiveDescriptors();
                    pML->UpdateNormalAndDepth();
                }
            }
        }
        
    } // line tracking part 
    

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}



void LocalMapping::ComputeH12(KeyFramePtr& pKF1, KeyFramePtr& pKF2, Eigen::Matrix3f& H12, Eigen::Vector3f& e1, const bool bRight1, const bool bRight2)      
{
    // const Eigen::Matrix3f R1w = pKF1->GetRotation();
    // const Eigen::Vector3f t1w = pKF1->GetTranslation();
    // const Eigen::Matrix3f R2w = pKF2->GetRotation();
    // const Eigen::Vector3f t2w = pKF2->GetTranslation();

    const auto T1w = bRight1 ? pKF1->GetRightPose() : pKF1->GetPose();
    const Eigen::Matrix3f R1w = T1w.rotationMatrix();
    const Eigen::Vector3f t1w = T1w.translation();

    const auto T2w = bRight2 ? pKF2->GetRightPose() : pKF2->GetPose();
    const Eigen::Matrix3f R2w = T2w.rotationMatrix();
    const Eigen::Vector3f t2w = T2w.translation();

    const Eigen::Matrix3f R12 =  R1w*R2w.transpose();
    const Eigen::Vector3f t12 = -R12*t2w+t1w;

    const Eigen::Matrix3f K1 = bRight1 ? pKF1->mpCamera2->toLinearK_() : pKF1->mpCamera->toLinearK_();
    const Eigen::Matrix3f K2 = bRight2 ? pKF2->mpCamera2->toLinearK_() : pKF2->mpCamera->toLinearK_();
    
    Eigen::Matrix3f K2inv;
    K2inv <<  pKF2->invfx,           0,  -pKF2->cx*pKF2->invfx, 
                        0, pKF2->invfy,  -pKF2->cy*pKF2->invfy, 
                        0,           0,                     1.;

    e1  = K1*t12; // epipole in image 1 
    //H12 = K1*R12*K2.inverse();
    H12 = K1*R12*K2inv;
    
}


void LocalMapping::ComputeH21(KeyFramePtr& pKF1, KeyFramePtr& pKF2, Eigen::Matrix3f& H21, Eigen::Vector3f& e2, const bool bRight1, const bool bRight2)      
{
    // const Eigen::Matrix3f R1w = pKF1->GetRotation();
    // const Eigen::Vector3f t1w = pKF1->GetTranslation();
    // const Eigen::Matrix3f R2w = pKF2->GetRotation();
    // const Eigen::Vector3f t2w = pKF2->GetTranslation();

    const auto T1w = bRight1 ? pKF1->GetRightPose() : pKF1->GetPose();
    const Eigen::Matrix3f R1w = T1w.rotationMatrix();
    const Eigen::Vector3f t1w = T1w.translation();

    const auto T2w = bRight2 ? pKF2->GetRightPose() : pKF2->GetPose();
    const Eigen::Matrix3f R2w = T2w.rotationMatrix();
    const Eigen::Vector3f t2w = T2w.translation();

    const Eigen::Matrix3f R21 =  R2w*R1w.transpose();
    const Eigen::Vector3f t21 = -R21*t1w+t2w;

    const Eigen::Matrix3f K1 = bRight1 ? pKF1->mpCamera2->toLinearK_() : pKF1->mpCamera->toLinearK_();
    const Eigen::Matrix3f K2 = bRight2 ? pKF2->mpCamera2->toLinearK_() : pKF2->mpCamera->toLinearK_();

    Eigen::Matrix3f K1inv;
    K1inv <<  pKF1->invfx,           0,  -pKF1->cx*pKF1->invfx, 
                        0, pKF1->invfy,  -pKF1->cy*pKF1->invfy, 
                        0,           0,                     1.;

    e2  = K2*t21; // epipole in image 2
    //H21 = K2*R21*K1.inverse();
    H21 = K2*R21*K1inv;    
}

#if 0

cv::Mat LocalMapping::ComputeF12(KeyFramePtr& pKF1, KeyFramePtr& pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mpCamera->toLinearK();
    const cv::Mat &K2 = pKF2->mpCamera->toLinearK();


    return K1.t().inv()*t12x*R12*K2.inv();
}

cv::Matx33f LocalMapping::ComputeF12_(KeyFramePtr& pKF1, KeyFramePtr& pKF2)
{
    auto R1w = pKF1->GetRotation_();
    auto t1w = pKF1->GetTranslation_();
    auto R2w = pKF2->GetRotation_();
    auto t2w = pKF2->GetTranslation_();

    auto R12 = R1w*R2w.t();
    auto t12 = -R1w*R2w.t()*t2w+t1w;

    auto t12x = SkewSymmetricMatrix_(t12);

    const auto &K1 = pKF1->mpCamera->toLinearK_();
    const auto &K2 = pKF2->mpCamera->toLinearK_();


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::ComputeF12(KeyFramePtr& pKF1, KeyFramePtr& pKF2, cv::Mat& F12, cv::Mat& H12, cv::Mat& e1)      
{
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    const cv::Mat R12 = R1w*R2w.t();
    const cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    const cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mpCamera->toLinearK();
    const cv::Mat &K2 = pKF2->mpCamera->toLinearK();
        
    /*const cv::Mat K1invT = (cv::Mat_<float>(3,3) <<           pKF1->invfx,                     0,   0., 
                                                                        0,           pKF1->invfy,   0., 
                                                    -pKF1->cx*pKF1->invfx, -pKF1->cy*pKF1->invfy,   1.);*/
    
    /*const cv::Mat K2inv = (cv::Mat_<float>(3,3) <<  pKF2->invfx,           0,  -pKF2->cx*pKF2->invfx, 
                                                              0, pKF2->invfy,  -pKF2->cy*pKF2->invfy, 
                                                              0,           0,                     1.);
    
    const cv::Mat R12xK2inv = R12*K2inv;*/
    const cv::Mat R12xK2inv = R12*K2.inv();

    e1  = K1*t12;
    H12 = K1*R12xK2inv;
    
    //return K1.t().inv()*t12x*R12*K2.inv();
    
    //F12 = K1invT*t12x*R12xK2inv;
    F12 = K1.t().inv()*t12x*R12xK2inv;
    
#if 0    
    const cv::Mat e1x = SkewSymmetricMatrix(e1);    
    cv::Mat F12_2 = e1x*H12;
    //F12 = F12/cv::norm(F12);
    
    std::cout << "dist: " << cv::norm(F12_2/cv::norm(F12_2) - F12/cv::norm(F12)) << std::endl;
#endif
    
#if 0    
    const cv::Mat &K2 = pKF2->mpCamera->toLinearK();    
    cv::Mat F12_2 = K1.t().inv()*t12x*R12*K2.inv();
    std::cout << "dist: " << cv::norm(F12_2 - F12) << std::endl;    
#endif
    
}

void LocalMapping::ComputeF12_(KeyFramePtr& pKF1, KeyFramePtr& pKF2, cv::Matx33f& F12, cv::Matx33f& H12, cv::Matx31f& e1)        
{
    const auto R1w = pKF1->GetRotation_();
    const auto t1w = pKF1->GetTranslation_();
    const auto R2w = pKF2->GetRotation_();
    const auto t2w = pKF2->GetTranslation_();

    const auto R12 = R1w*R2w.t();
    const auto t12 = -R1w*R2w.t()*t2w+t1w;

    const auto t12x = SkewSymmetricMatrix_(t12);

    const auto &K1 = pKF1->mpCamera->toLinearK_();
    const auto &K2 = pKF2->mpCamera->toLinearK_();
        
    /*const cv::Mat K1invT = (cv::Mat_<float>(3,3) <<           pKF1->invfx,                     0,   0., 
                                                                        0,           pKF1->invfy,   0., 
                                                    -pKF1->cx*pKF1->invfx, -pKF1->cy*pKF1->invfy,   1.);*/
    
    /*const cv::Mat K2inv = (cv::Mat_<float>(3,3) <<  pKF2->invfx,           0,  -pKF2->cx*pKF2->invfx, 
                                                              0, pKF2->invfy,  -pKF2->cy*pKF2->invfy, 
                                                              0,           0,                     1.);
    
    const cv::Mat R12xK2inv = R12*K2inv;*/
    const auto R12xK2inv = R12*K2.inv();

    e1  = K1*t12;
    H12 = K1*R12xK2inv;
    
    //return K1.t().inv()*t12x*R12*K2.inv();
    
    //F12 = K1invT*t12x*R12xK2inv;
    F12 = K1.t().inv()*t12x*R12xK2inv;
    
#if 0    
    const auto e1x = SkewSymmetricMatrix_(e1);    
    auto F12_2 = e1x*H12;
    //F12 = F12/cv::norm(F12);
    
    std::cout << "dist: " << cv::norm(F12_2/cv::norm(F12_2) - F12/cv::norm(F12)) << std::endl;
#endif
    
#if 0    
    const auto &K2 = pKF2->mpCamera->toLinearK_();    
    auto F12_2 = K1.t().inv()*t12x*R12*K2.inv();
    std::cout << "dist: " << cv::norm(F12_2 - F12) << std::endl;    
#endif
    
}

#endif 

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFramePtr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        //delete *lit;
        DeletePtr(*lit);
    }
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}


#define USE_LINES_FOR_KEYFRAMES_CULLING 1

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints and MapLines it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points and stereo lines 
    const int Nd = 21;
    mpCurrentKeyFrame->UpdateBestCovisibles();
    vector<KeyFramePtr> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    float redundant_th;
    if(!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count=0;

    // Compoute last KF from optimizable window:
    unsigned int last_ID;
    if (mbInertial)
    {
        int count = 0;
        KeyFramePtr aux_KF = mpCurrentKeyFrame;
        while(count<Nd && aux_KF->mPrevKF)
        {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }


    for(vector<KeyFramePtr>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;
        KeyFramePtr pKF = *vit;

        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
            continue;
                
        const vector<MapPointPtr> vpMapPoints = pKF->GetMapPointMatches();

        int nPointObs = 3;
        const int thPointObs=nPointObs;
        int nRedundantPointObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPointPtr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thPointObs)
                    {
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i-pKF->NLeft].octave; // Luigi fix
                        const map<KeyFramePtr, tuple<int,int>> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFramePtr, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFramePtr pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            tuple<int,int> indexes = mit->second;
                            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if(pKFi -> NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                  : scaleLeveli;
                                }
                            }

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>thPointObs) // ORBSLAM2 was if(nObs>=thPointObs)
                                    break;
                            }
                        }
                        if(nObs>thPointObs)  // ORBSLAM2 was if(nObs>=thPointObs)
                        {
                            nRedundantPointObservations++;
                        }
                    }
                }
            }
        }

        
#if USE_LINES_FOR_KEYFRAMES_CULLING        
                
        int nLineObs = 3; // was 4
        const int thLineObs=nLineObs;
        int nRedundantLineObservations=0;
        int nMLs=0;
        
        if(mpTracker->IsLineTracking())
        {        
            const vector<MapLinePtr> vpMapLines = pKF->GetMapLineMatches();
            for(size_t i=0, iend=vpMapLines.size(); i<iend; i++)
            {
                MapLinePtr pML = vpMapLines[i];
                if(pML)
                {
                    if(!pML->isBad())
                    {
                        if(!mbMonocular)
                        {
                            // if line is too far or is not stereo then continue
                            if( 
                                (pKF->mvDepthLineStart.empty()) ||
                                (pKF->mvDepthLineStart[i] > pKF->mThDepth || pKF->mvDepthLineStart[i] < 0) || 
                                (pKF->mvDepthLineEnd[i] > pKF->mThDepth || pKF->mvDepthLineEnd[i] < 0)
                              )
                                continue;
                        }

                        nMLs++;
                        if(pML->Observations()>thLineObs)
                        {                                
                            const int &scaleLevel = (pKF->NlinesLeft == -1) ? pKF->mvKeyLinesUn[i].octave
                                                                                : (i < pKF->NlinesLeft) ? pKF->mvKeyLinesUn[i].octave
                                                                                                        : pKF->mvKeyLinesRightUn[i-pKF->NlinesLeft].octave;
                            const map<KeyFramePtr, tuple<int,int>> observations = pML->GetObservations();
                            int nObs=0;
                            for(map<KeyFramePtr, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                            {
                                KeyFramePtr pKFi = mit->first;
                                if(pKFi==pKF)
                                    continue;
                                tuple<int,int> indexes = mit->second;
                                int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                                int scaleLeveli = -1;
                                if(pKFi -> NlinesLeft == -1)
                                    scaleLeveli = pKFi->mvKeyLinesUn[leftIndex].octave;
                                else {
                                    if (leftIndex != -1) {
                                        scaleLeveli = pKFi->mvKeyLinesUn[leftIndex].octave;
                                    }
                                    if (rightIndex != -1) {
                                        int rightLevel = pKFi->mvKeyLinesRightUn[rightIndex - pKFi->NlinesLeft].octave;
                                        scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                        : scaleLeveli;
                                    }
                                }             
                                
                            
                                if(scaleLeveli<=scaleLevel+1)
                                {
                                    nObs++;
                                    if(nObs>=thLineObs)
                                        break;
                                }
                            }
                            if(nObs>=thLineObs)
                            {
                                nRedundantLineObservations++;
                            }
                        }
                    }
                }
            }   
        } 
#else
        int nRedundantLineObservations=0;
        int nMLs=0;        
#endif

        if((nRedundantPointObservations + nRedundantLineObservations)>redundant_th*(nMPs + nMLs))
        {
            if (mbInertial)
            {
                if (mpAtlas->KeyFramesInMap()<=Nd)
                    continue;

                if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
                    continue;

                if(pKF->mPrevKF && pKF->mNextKF)
                {
                    const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                    else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && ((pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition()).norm()<0.02) && (t<3))
                    {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = NULL;
                        pKF->mPrevKF = NULL;
                        pKF->SetBadFlag();
                    }
                }
            }
            else
            {
                pKF->SetBadFlag();
            }
        }
        if((count > 20 && mbAbortBA) || count>100)
        {
            break;
        }
    }
}


cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

cv::Matx33f LocalMapping::SkewSymmetricMatrix_(const cv::Matx31f &v)
{
    cv::Matx33f skew{0.f, -v(2), v(1),
                     v(2), 0.f, -v(0),
                     -v(1), v(0), 0.f};

    return skew;
}

Eigen::Matrix3f LocalMapping::SkewSymmetricMatrix(const Eigen::Vector3f &v)
{
    Eigen::Matrix3f skew; 
    skew << 0.f, -v[2], v[1],
            v[2], 0.f, -v[0],
            -v[1], v[0], 0.f;
    return skew; 
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Map reset recieved" << endl;
        mbResetRequested = true;
    }
    cout << "LM: Map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Map reset, Done!!!" << endl;
}

void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequestedActiveMap)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            executed_reset = true;

            cout << "LM: Resetting Atlas in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mlpRecentAddedMapLines.clear();
            mbResetRequested = false;
            mbResetRequestedActiveMap = false;

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mIdxInit=0;

            cout << "LM: End resetting Local Mapping..." << endl;
        }

        if(mbResetRequestedActiveMap) {
            executed_reset = true;
            cout << "LM: Resetting current map in Local Mapping..." << endl;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mlpRecentAddedMapLines.clear();

            // Inertial parameters
            mTinit = 0.f;
            mbNotBA2 = true;
            mbNotBA1 = true;
            mbBadImu=false;

            mbResetRequested = false;
            mbResetRequestedActiveMap = false;
            cout << "LM: End resetting Local Mapping..." << endl;
        }
    }
    if(executed_reset)
        cout << "LM: Reset free the mutex" << endl;

}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
{
    if (mbResetRequested)
        return;

    float minTime;
    int nMinKF;
    if (mbMonocular)
    {
        minTime = 2.0;
        nMinKF = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF = 10;
    }


    if(mpAtlas->KeyFramesInMap()<nMinKF)
        return;

    // Retrieve all keyframe in temporal order
    list<KeyFramePtr> lpKF;
    KeyFramePtr pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFramePtr> vpKF(lpKF.begin(),lpKF.end());

    if(vpKF.size()<nMinKF)
        return;

    mFirstTs=vpKF.front()->mTimeStamp;
    if(mpCurrentKeyFrame->mTimeStamp-mFirstTs<minTime)
        return;

    bInitializing = true;

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0,0,0,0,0,0);

    // Compute and KF velocities mRwg estimation
    if (!mpCurrentKeyFrame->GetMap()->isImuInitialized())
    {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();
        for(vector<KeyFramePtr>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)
        {
            if (!(*itKF)->mpImuPreintegrated)
                continue;
            if (!(*itKF)->mPrevKF)
                continue;

            dirG -= (*itKF)->mPrevKF->GetImuRotation() * (*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            Eigen::Vector3f _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        dirG = dirG/dirG.norm();
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f v = gI.cross(dirG);
        const float nv = v.norm();
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);
        Eigen::Vector3f vzg = v*ang/nv;
        Rwg = Sophus::SO3f::exp(vzg).matrix();
        mRwg = Rwg.cast<double>();
        mTinit = mpCurrentKeyFrame->mTimeStamp-mFirstTs;
    }
    else
    {
        mRwg = Eigen::Matrix3d::Identity();
        mbg = mpCurrentKeyFrame->GetGyroBias().cast<double>();
        mba = mpCurrentKeyFrame->GetAccBias().cast<double>();
    }

    mScale=1.0;

    mInitTime = mpTracker->mLastFrame.mTimeStamp-vpKF.front()->mTimeStamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbMonocular, infoInertial, false, false, priorG, priorA);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1)
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }

    // Before this line we are not changing the map
    {
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        if ((fabs(mScale - 1.f) > 0.00001) || !mbMonocular) {
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
            mpTracker->UpdateFrameIMU(mScale, vpKF[0]->GetImuBias(), mpCurrentKeyFrame);
        }

        // Check if initialization OK
        if (!mpAtlas->isImuInitialized())
            for (int i = 0; i < N; i++) {
                KeyFrame *pKF2 = vpKF[i];
                pKF2->bImu = true;
            }
    }

    mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
    if (!mpAtlas->isImuInitialized())
    {
        mpAtlas->SetImuInitialized();
        mpTracker->t0IMU = mpTracker->mCurrentFrame.mTimeStamp;
        mpCurrentKeyFrame->bImu = true;
    }

    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    if (bFIBA)
    {
        if (priorA!=0.f)
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId, NULL, true, priorG, priorA);
        else
            Optimizer::FullInertialBA(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId, NULL, false);
    }

    std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();

    Verbose::PrintMess("Global Bundle Adjustment finished\nUpdating map ...", Verbose::VERBOSITY_NORMAL);

    // Get Map Mutex
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    unsigned long GBAid = mpCurrentKeyFrame->mnId;

    // Process keyframes in the queue
    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    // Correct keyframes starting at map first keyframe
    list<KeyFramePtr> lpKFtoCheck(mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.begin(),mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.end());

    while(!lpKFtoCheck.empty())
    {
        KeyFramePtr pKF = lpKFtoCheck.front();
        const set<KeyFramePtr> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        for(set<KeyFramePtr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
        {
            KeyFramePtr pChild = *sit;
            if(!pChild || pChild->isBad())
                continue;

            if(pChild->mnBAGlobalForKF!=GBAid)
            {
                Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

                Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                if(pChild->isVelocitySet()){
                    pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                }
                else {
                    Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);
                }

                pChild->mBiasGBA = pChild->GetImuBias();
                pChild->mnBAGlobalForKF = GBAid;

            }
            lpKFtoCheck.push_back(pChild);
        }

        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);

        if(pKF->bImu)
        {
            pKF->mVwbBefGBA = pKF->GetVelocity();
            pKF->SetVelocity(pKF->mVwbGBA);
            pKF->SetNewBias(pKF->mBiasGBA);
        } else {
            cout << "KF " << pKF->mnId << " not set to inertial!! \n";
        }

        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<MapPointPtr> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPointPtr pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        if(pMP->mnBAGlobalForKF==GBAid)
        {
            // If optimized by Global BA, just update
            pMP->SetWorldPos(pMP->mPosGBA);
        }
        else
        {
            // Update according to the correction of its reference keyframe
            KeyFramePtr pRefKF = pMP->GetReferenceKeyFrame();

            if(pRefKF->mnBAGlobalForKF!=GBAid)
                continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

            // Backproject using corrected camera
            pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
    }

    // Correct MapLines
    if(mpTracker->IsLineTracking())
    {
        const vector<MapLinePtr> vpMLs = mpAtlas->GetCurrentMap()->GetAllMapLines();

        for(size_t i=0; i<vpMLs.size(); i++)
        {
            MapLinePtr pML = vpMLs[i];

            if(pML->isBad())
                continue;

            if(pML->mnBAGlobalForKF==GBAid)
            {
                // If optimized by Global BA, just update
                pML->SetWorldEndPoints(pML->mPosStartGBA, pML->mPosEndGBA);
                pML->UpdateNormalAndDepth();
            }
            else
            {
                // Update according to the correction of its reference keyframe
                KeyFramePtr pRefKF = pML->GetReferenceKeyFrame();

                if(pRefKF->mnBAGlobalForKF!=GBAid)
                    continue;

                // Map to non-corrected camera
                Eigen::Vector3f XSw, XEw;
                pML->GetWorldEndPoints(XSw, XEw);  
                const Eigen::Vector3f XSc = pRefKF->mTcwBefGBA * XSw;
                const Eigen::Vector3f XEc = pRefKF->mTcwBefGBA * XEw;

                // Backproject using corrected camera
                const Sophus::SE3f Twc = pRefKF->GetPoseInverse();

                pML->SetWorldEndPoints(Twc*XSc, Twc*XEc);                        
                pML->UpdateNormalAndDepth();
            }
        }    
    }

    // Correct MapObjects
    if(mpTracker->IsObjectTracking())
    {
        const vector<MapObjectPtr> vpMObjs = mpAtlas->GetCurrentMap()->GetAllMapObjects();

        for(size_t i=0; i<vpMObjs.size(); i++)
        {
            MapObjectPtr pMObj = vpMObjs[i];

            if(pMObj->isBad())
                continue;

            if(pMObj->mnBAGlobalForKF==GBAid)
            {
                // If optimized by Global BA, just update
                pMObj->SetSim3Pose(pMObj->mSowGBA);
                //pMObj->UpdateNormalAndDepth();
            }
            else
            {
                // Update according to the correction of its reference keyframe
                KeyFramePtr pRefKF = pMObj->GetReferenceKeyFrame();

                if(pRefKF->mnBAGlobalForKF!=GBAid)
                    continue;

                // Map to non-corrected camera
                const Eigen::Matrix3f Rcw = pRefKF->mTcwBefGBA.rotationMatrix();
                const Eigen::Vector3f tcw = pRefKF->mTcwBefGBA.translation();
                
                Eigen::Matrix3f Rwo; 
                Eigen::Vector3f two; 
                const double scale = pMObj->GetScale();
                Rwo = pMObj->GetInverseRotation();
                two = pMObj->GetInverseTranslation();
                const Eigen::Matrix3f Rco = Rcw*Rwo;
                const Eigen::Vector3f tco = Rcw*two+tcw;  

                // Backproject using corrected camera
                const Sophus::SE3f Twc = pRefKF->GetPoseInverse();
                const Eigen::Matrix3f Rwc = Twc.rotationMatrix();
                const Eigen::Vector3f twc = Twc.translation();
                
                const Eigen::Matrix3f RwoNew = Rwc*Rco;
                const Eigen::Vector3f twoNew = Rwc*tco+twc;
                
                pMObj->SetSim3InversePose(RwoNew, twoNew, scale);
            }
        }    
    }   

    Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    mnKFs=vpKF.size();
    mIdxInit++;

    for(list<KeyFramePtr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    mpTracker->mState=Tracking::OK;
    bInitializing = false;

    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}

void LocalMapping::ScaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(mMutexImuInit);
    if (mbResetRequested)
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFramePtr> lpKF;
    KeyFramePtr pKF = mpCurrentKeyFrame;
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFramePtr> vpKF(lpKF.begin(),lpKF.end());

    while(CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
        vpKF.push_back(mpCurrentKeyFrame);
        lpKF.push_back(mpCurrentKeyFrame);
    }

    const int N = vpKF.size();

    mRwg = Eigen::Matrix3d::Identity();
    mScale=1.0;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    if (mScale<1e-1) // 1e-1
    {
        cout << "scale too small" << endl;
        bInitializing=false;
        return;
    }
    
    Sophus::SO3d so3wg(mRwg);
    // Before this line we are not changing the map
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    if ((fabs(mScale-1.f)>0.002)||!mbMonocular)
    {
        Sophus::SE3f Tgw(mRwg.cast<float>().transpose(),Eigen::Vector3f::Zero());
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw,mScale,true);
        mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame);
    }
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    for(list<KeyFramePtr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        (*lit)->SetBadFlag();
        delete *lit;
    }
    mlNewKeyFrames.clear();

    double t_inertial_only = std::chrono::duration_cast<std::chrono::duration<double> >(t1 - t0).count();

    // To perform pose-inertial opt w.r.t. last keyframe
    mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

    return;
}



bool LocalMapping::IsInitializing()
{
    return bInitializing;
}


double LocalMapping::GetCurrKFTime()
{

    if (mpCurrentKeyFrame)
    {
        return mpCurrentKeyFrame->mTimeStamp;
    }
    else
        return 0.0;
}

KeyFramePtr LocalMapping::GetCurrKF()
{
    return mpCurrentKeyFrame;
}

} // namespace PLVS2
