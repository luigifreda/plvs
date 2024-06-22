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


#include "LoopClosing.h"

#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"
#include "Atlas.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include "LineMatcher.h"
#include "MapObject.h"

#include<mutex>
#include<thread>

#define USE_LINES_FOR_VOTING_LOOP_CLOSURE 1
#define USE_OBJECTS_FOR_VOTING_LOOP_CLOSURE 1

#define ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE 1

#define VERBOSE 1

namespace PLVS2
{

LoopClosing::LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale, const bool bActiveLC):
    mbResetRequested(false), mbResetActiveMapRequested(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mnLoopNumCoincidences(0), mnMergeNumCoincidences(0),
    mbLoopDetected(false), mbMergeDetected(false), mnLoopNumNotFound(0), mnMergeNumNotFound(0), mbActiveLC(bActiveLC)
{
    mnCovisibilityConsistencyTh = 3;
    mpLastCurrentKF = static_cast<KeyFramePtr>(NULL);

#ifdef REGISTER_TIMES

    vdDataQuery_ms.clear();
    vdEstSim3_ms.clear();
    vdPRTotal_ms.clear();

    vdMergeMaps_ms.clear();
    vdWeldingBA_ms.clear();
    vdMergeOptEss_ms.clear();
    vdMergeTotal_ms.clear();
    vnMergeKFs.clear();
    vnMergeMPs.clear();
    nMerges = 0;

    vdLoopFusion_ms.clear();
    vdLoopOptEss_ms.clear();
    vdLoopTotal_ms.clear();
    vnLoopKFs.clear();
    nLoop = 0;

    vdGBA_ms.clear();
    vdUpdateMap_ms.clear();
    vdFGBATotal_ms.clear();
    vnGBAKFs.clear();
    vnGBAMPs.clear();
    nFGBA_exec = 0;
    nFGBA_abort = 0;

#endif

    mstrFolderSubTraj = "SubTrajectories/";
    mnNumCorrection = 0;
    mnCorrectionGBA = 0;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {

        //NEW LOOP AND MERGE DETECTION ALGORITHM
        //----------------------------


        if(CheckNewKeyFrames())
        {
            if(mpLastCurrentKF)
            {
                mpLastCurrentKF->mvpLoopCandKFs.clear();
                mpLastCurrentKF->mvpMergeCandKFs.clear();
            }
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartPR = std::chrono::steady_clock::now();
#endif

            bool bFindedRegion = NewDetectCommonRegions();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndPR = std::chrono::steady_clock::now();

            double timePRTotal = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPR - time_StartPR).count();
            vdPRTotal_ms.push_back(timePRTotal);
#endif
            if(bFindedRegion)
            {
                if(mbMergeDetected)
                {
                    if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                        (!mpCurrentKF->GetMap()->isImuInitialized()))
                    {
                        cout << "IMU is not initilized, merge is aborted" << endl;
                    }
                    else
                    {
                        Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
                        g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
                        Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
                        g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
                        g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                        g2o::Sim3 gSw1m = mg2oMergeSlw;

                        mSold_new = (gSw2c * gScw1);


                        if(mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                        {
                            cout << "Merge check transformation with IMU" << endl;
                            if(mSold_new.scale()<0.90||mSold_new.scale()>1.1){
                                mpMergeLastCurrentKF->SetErase();
                                mpMergeMatchedKF->SetErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mvpMergeMatchedMLs.clear();
                                mvpMergeMLs.clear();
                                mvpMergeMatchedMOs.clear();
                                mvpMergeMOs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected = false;
                                Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                continue;
                            }
                            // If inertial, force only yaw
                            if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                                   mpCurrentKF->GetMap()->GetIniertialBA1())
                            {
                                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                phi(0)=0;
                                phi(1)=0;
                                mSold_new = g2o::Sim3(ExpSO3(phi),mSold_new.translation(),1.0);
                            }
                        }
                        
                        cout << "**************" << endl;
                        cout << "Merge detected!" << endl;
                        cout << "**************" << endl;                        

                        mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

                        mg2oMergeScw = mg2oMergeSlw;

                        //mpTracker->SetStepByStep(true);

                        Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();

                        nMerges += 1;
#endif
                        // TODO UNCOMMENT
                        if (mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
                            MergeLocal2();
                        else
                            MergeLocal();

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndMerge = std::chrono::steady_clock::now();

                        double timeMergeTotal = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMerge - time_StartMerge).count();
                        vdMergeTotal_ms.push_back(timeMergeTotal);
#endif

                        Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                    }

                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(1);

                    // Reset all variables
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mvpMergeMatchedMLs.clear();
                    mvpMergeMLs.clear();
                    mvpMergeMatchedMOs.clear();
                    mvpMergeMOs.clear();
                    mnMergeNumNotFound = 0;
                    mbMergeDetected = false;

                    if(mbLoopDetected)
                    {
                        // Reset Loop variables
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMatchedMLs.clear();
     		            mvpMergeMatchedMOs.clear();
                        mvpLoopMPs.clear();
                        mvpLoopMLs.clear();
                        mvpLoopMOs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }

                }

                /// < Loop detected 
                if(mbLoopDetected)
                {
                    bool bGoodLoop = true;
                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(0);

                    Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);

                    mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];
                    if(mpCurrentKF->GetMap()->IsInertial())
                    {
                        Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
                        g2o::Sim3 g2oTwc(Twc.unit_quaternion(),Twc.translation(),1.0);
                        g2o::Sim3 g2oSww_new = g2oTwc*mg2oLoopScw;

                        Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
                        cout << "phi = " << phi.transpose() << endl; 
                        if (fabs(phi(0))<0.008f && fabs(phi(1))<0.008f && fabs(phi(2))<0.349f)
                        {
                            if(mpCurrentKF->GetMap()->IsInertial())
                            {
                                // If inertial, force only yaw
                                if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                                        mpCurrentKF->GetMap()->GetIniertialBA2())
                                {
                                    phi(0)=0;
                                    phi(1)=0;
                                    g2oSww_new = g2o::Sim3(ExpSO3(phi),g2oSww_new.translation(),1.0);
                                    mg2oLoopScw = g2oTwc.inverse()*g2oSww_new;
                                }
                            }

                        }
                        else
                        {
                            cout << "BAD LOOP!!!" << endl;
                            bGoodLoop = false;
                        }

                    }

                    if (bGoodLoop) {

                        mvpLoopMapPoints = mvpLoopMPs;
                        if(mpTracker->IsLineTracking()) mvpLoopMapLines = mvpLoopMLs;
                        if(mpTracker->IsObjectTracking()) mvpLoopMapObjects = mvpLoopMOs;
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();

                        nLoop += 1;

#endif
                        CorrectLoop();
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();

                        double timeLoopTotal = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLoop - time_StartLoop).count();
                        vdLoopTotal_ms.push_back(timeLoopTotal);
#endif

                        mnNumCorrection += 1;
                    }

                    // Reset all variables
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMatchedMLs.clear();
                    mvpLoopMatchedMOs.clear();
                    mvpLoopMPs.clear();
                    mvpLoopMLs.clear();
                    mvpLoopMOs.clear();
                    mnLoopNumNotFound = 0;
                    mbLoopDetected = false;
                }

            }
            mpLastCurrentKF = mpCurrentKF;
        }

        ResetIfRequested();

        if(CheckFinish()){
            break;
        }

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::NewDetectCommonRegions()
{
    // To deactivate placerecognition. No loopclosing nor merging will be performed
    if(!mbActiveLC)
        return false;

    bool bLoopClosingEnabled = true;
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
        mpCurrentKF->mbCurrentPlaceRecognition = true;

        mpLastMap = mpCurrentKF->GetMap();
    }

    {
        unique_lock< mutex > lock(mpAtlas->mMutexLoopClosing);
        bLoopClosingEnabled = mpAtlas->mnEnableLoopClosing;
    }

    if(mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if(mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) //12
    {
        // cout << "LoopClousure: Stereo KF inserted without check: " << mpCurrentKF->mnId << endl;
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if(mpLastMap->GetAllKeyFrames().size() < 12  || !bLoopClosingEnabled)
    {
        // cout << "LoopClousure: Stereo KF inserted without check, map is small: " << mpCurrentKF->mnId << endl;
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    //cout << "LoopClousure: Checking KF: " << mpCurrentKF->mnId << endl;

    //Check the last candidates with geometric validation
    // Loop candidates
    bool bLoopDetectedInKF = false;
    bool bCheckSpatial = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_1 = std::chrono::steady_clock::now();
#endif
    if(mnLoopNumCoincidences > 0)
    {
        bCheckSpatial = true;
        // Find from the last KF candidates
        Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(),mTcl.translation(),1.0);
        g2o::Sim3 gScw = gScl * mg2oLoopSlw;
        int numProjMatches = 0;
        vector<MapPointPtr> vpMatchedMPs;
        vector<MapLinePtr> vpMatchedMLs;
        vector<MapObjectPtr> vpMatchedMOs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs, 
                                                                                                                mvpLoopMLs, vpMatchedMLs,
                                                                                                                mvpLoopMOs, vpMatchedMOs);
        if(bCommonRegion)
        {

            bLoopDetectedInKF = true;

            mnLoopNumCoincidences++;
            mpLoopLastCurrentKF->SetErase();
            mpLoopLastCurrentKF = mpCurrentKF;
            mg2oLoopSlw = gScw;
            mvpLoopMatchedMPs = vpMatchedMPs;


            mbLoopDetected = mnLoopNumCoincidences >= 3;
            mnLoopNumNotFound = 0;

            if(!mbLoopDetected)
            {
                cout << "PR: Loop detected with Reffine Sim3" << endl;
            }
        }
        else
        {
            bLoopDetectedInKF = false;

            mnLoopNumNotFound++;
            if(mnLoopNumNotFound >= 2)
            {
                mpLoopLastCurrentKF->SetErase();
                mpLoopMatchedKF->SetErase();
                mnLoopNumCoincidences = 0;
                mvpLoopMatchedMPs.clear();
                mvpLoopMatchedMLs.clear();
                mvpLoopMatchedMOs.clear();
                mvpLoopMPs.clear();
                mvpLoopMLs.clear();
                mvpLoopMOs.clear();
                mnLoopNumNotFound = 0;
            }

        }
    }

    //Merge candidates
    bool bMergeDetectedInKF = false;
    if(mnMergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();

        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw = gScl * mg2oMergeSlw;
        int numProjMatches = 0;
        vector<MapPointPtr> vpMatchedMPs;
        vector<MapLinePtr> vpMatchedMLs;
        vector<MapObjectPtr> vpMatchedMOs;        
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs, 
                                                                                                                 mvpMergeMLs, vpMatchedMLs,
                                                                                                                 mvpMergeMOs, vpMatchedMOs);
        if(bCommonRegion)
        {
            bMergeDetectedInKF = true;

            mnMergeNumCoincidences++;
            mpMergeLastCurrentKF->SetErase();
            mpMergeLastCurrentKF = mpCurrentKF;
            mg2oMergeSlw = gScw;
            mvpMergeMatchedMPs = vpMatchedMPs;
            if(mpTracker->IsLineTracking())
            {
                mvpMergeMatchedMLs = vpMatchedMLs;
            }
            if(mpTracker->IsObjectTracking())
            {
                mvpMergeMatchedMOs = vpMatchedMOs;
            }            
            mbMergeDetected = mnMergeNumCoincidences >= 3;
        }
        else
        {
            mbMergeDetected = false;
            bMergeDetectedInKF = false;

            mnMergeNumNotFound++;
            if(mnMergeNumNotFound >= 2)
            {
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mvpMergeMatchedMLs.clear();
                mvpMergeMLs.clear();
                mvpMergeMatchedMOs.clear();
                mvpMergeMOs.clear();                
                mnMergeNumNotFound = 0;
            }


        }
    }  
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_1 = std::chrono::steady_clock::now();

        double timeEstSim3 = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndEstSim3_1 - time_StartEstSim3_1).count();
#endif

    if(mbMergeDetected || mbLoopDetected)
    {
#ifdef REGISTER_TIMES
        vdEstSim3_ms.push_back(timeEstSim3);
#endif
        mpKeyFrameDB->add(mpCurrentKF);
        return true;
    }

    //TODO: This is only necessary if we use a minimun score for pick the best candidates
    const vector<KeyFramePtr> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

    // Extract candidates from the bag of words
    vector<KeyFramePtr> vpMergeBowCand, vpLoopBowCand;
    if(!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
        // Search in BoW
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartQuery = std::chrono::steady_clock::now();
#endif
        mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand,3);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndQuery = std::chrono::steady_clock::now();

        double timeDataQuery = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndQuery - time_StartQuery).count();
        vdDataQuery_ms.push_back(timeDataQuery);
#endif
    }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartEstSim3_2 = std::chrono::steady_clock::now();
#endif
    // Check the BoW candidates if the geometric candidate list is empty
    //Loop candidates
    if(!bLoopDetectedInKF && !vpLoopBowCand.empty())
    {
        mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs,
                                                                                                                                             mvpLoopMLs, mvpLoopMatchedMLs,
                                                                                                                                             mvpLoopMOs, mvpLoopMatchedMOs);
    }
    // Merge candidates
    if(!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs,
                                                                                                                                                   mvpMergeMLs, mvpMergeMatchedMLs,
                                                                                                                                                   mvpMergeMOs, mvpMergeMatchedMOs);
    }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_2 = std::chrono::steady_clock::now();

        timeEstSim3 += std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndEstSim3_2 - time_StartEstSim3_2).count();
        vdEstSim3_ms.push_back(timeEstSim3);
#endif

    mpKeyFrameDB->add(mpCurrentKF);

    if(mbMergeDetected || mbLoopDetected)
    {
        return true;
    }

    mpCurrentKF->SetErase();
    mpCurrentKF->mbCurrentPlaceRecognition = false;

    return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                 std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs,
                                                 std::vector<MapLinePtr> &vpMLs, std::vector<MapLinePtr> &vpMatchedMLs,
                                                 std::vector<MapObjectPtr> &vpMOs, std::vector<MapObjectPtr> &vpMatchedMOs)
{
    set<MapPointPtr> spAlreadyMatchedMPs;
    set<MapLinePtr> spAlreadyMatchedMLs;  
    set<MapObjectPtr> spAlreadyMatchedMOs;
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs, 
                                                                            spAlreadyMatchedMLs, vpMLs, vpMatchedMLs,
                                                                            spAlreadyMatchedMOs, vpMOs, vpMatchedMOs);


    int nProjMatches = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    if(nNumProjMatches >= nProjMatches)
    {
        //Verbose::PrintMess("Sim3 reffine: There are " + to_string(nNumProjMatches) + " initial matches ", Verbose::VERBOSITY_DEBUG);
        Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
        g2o::Sim3 gSwm(mTwm.unit_quaternion(),mTwm.translation(),1.0);
        g2o::Sim3 gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale = mbFixScale;       // TODO CHECK; Solo para el monocular inertial
        if(mpTracker->mSensor==System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale=false;
        int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);

        //Verbose::PrintMess("Sim3 reffine: There are " + to_string(numOptMatches) + " matches after of the optimization ", Verbose::VERBOSITY_DEBUG);

        if(numOptMatches > nProjOptMatches)
        {
            g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(),1.0);

            vector<MapPointPtr> vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPointPtr>(NULL));

            nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs,
                                                                                               spAlreadyMatchedMLs, vpMLs, vpMatchedMLs,
                                                                                               spAlreadyMatchedMOs, vpMOs, vpMatchedMOs);
            if(nNumProjMatches >= nProjMatchesRep)
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<KeyFramePtr> &vpBowCand, KeyFramePtr &pMatchedKF2, KeyFramePtr &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                             int &nNumCoincidences, std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs,
                                             std::vector<MapLinePtr> &vpMLs, std::vector<MapLinePtr> &vpMatchedMLs,
                                             std::vector<MapObjectPtr> &vpMOs, std::vector<MapObjectPtr> &vpMatchedMOs)
{
    /// < TODO: add line matching here and use it in the Sim3Solver  ?  
    
    int nBoWMatches = 20;
    int nBoWInliers = 15;
    int nSim3Inliers = 20;
    int nProjMatches = 50;
    int nProjOptMatches = 80;

    set<KeyFramePtr> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

    int nNumCovisibles = 10;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);

    std::shared_ptr<LineMatcher> pLineMatcher;    
    if(mpTracker->IsLineTracking())
    {    
        pLineMatcher.reset( new LineMatcher(0.75) );
    }

    // Varibles to select the best numbe
    KeyFramePtr pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    
    std::vector<MapPointPtr> vpBestMapPoints;
    std::vector<MapPointPtr> vpBestMatchedMapPoints;
    
    std::vector<MapLinePtr> vpBestMapLines;    
    std::vector<MapLinePtr> vpBestMatchedMapLines;    
    
    std::vector<MapObjectPtr> vpBestMapObjects;    
    std::vector<MapObjectPtr> vpBestMatchedMapObjects;        

    int numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    int index = 0;
    //Verbose::PrintMess("BoW candidates: There are " + to_string(vpBowCand.size()) + " possible candidates ", Verbose::VERBOSITY_DEBUG);
    for(KeyFramePtr pKFi : vpBowCand)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        // std::cout << "KF candidate: " << pKFi->mnId << std::endl;
        // Current KF against KF with covisibles version
        std::vector<KeyFramePtr> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
        if(vpCovKFi.empty())
        {
            std::cout << "Covisible list empty" << std::endl;
            vpCovKFi.push_back(pKFi);
        }
        else
        {
            vpCovKFi.push_back(vpCovKFi[0]);
            vpCovKFi[0] = pKFi;
        }


        bool bAbortByNearKF = false;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }
        }
        if(bAbortByNearKF)
        {
            //std::cout << "Check BoW aborted because is close to the matched one " << std::endl;
            continue;
        }
        //std::cout << "Check BoW continue because is far to the matched one " << std::endl;


        std::vector<std::vector<MapPointPtr> > vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPointPtr> spMatchedMPi;
        int numBoWMatches = 0;

        KeyFramePtr pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;

        std::vector<MapPointPtr> vpMatchedPoints = std::vector<MapPointPtr>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPointPtr>(NULL));
        std::vector<KeyFramePtr> vpKeyFrameMatchedMP = std::vector<KeyFramePtr>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFramePtr>(NULL));

        int nIndexMostBoWMatchesKF=0;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;

            int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches = num;
                nIndexMostBoWMatchesKF = j;
            }
        }

        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            for(int k=0; k < vvpMatchedMPs[j].size(); ++k)
            {
                MapPointPtr pMPi_j = vvpMatchedMPs[j][k];
                if(!pMPi_j || pMPi_j->isBad())
                    continue;

                if(spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k]= pMPi_j;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }

        //pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

        if(numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            // Geometric validation
            bool bFixedScale = mbFixScale;
            if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale=false;

            Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            Eigen::Matrix4f mTcm;
            while(!bConverge && !bNoMore)
            {
                mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);
                //Verbose::PrintMess("BoW guess: Solver achieve " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
            }

            if(bConverge)
            {
                //std::cout << "Check BoW: SolverSim3 converged" << std::endl;

                //Verbose::PrintMess("BoW guess: Convergende with " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
                // Match by reprojection
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                vpCovKFi.push_back(pMostBoWMatchesKF);
                set<KeyFramePtr> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                //std::cout << "There are " << vpCovKFi.size() <<" near KFs" << std::endl;

                set<MapPointPtr> spMapPoints;
                vector<MapPointPtr> vpMapPoints;
                
                set<MapLinePtr> spMapLines;
                vector<MapLinePtr> vpMapLines;

                set<MapObjectPtr> spMapObjects;
                vector<MapObjectPtr> vpMapObjects;                
                
                vector<KeyFramePtr> vpKeyFrames;
                unordered_set<KeyFramePtr> spKeyFrames;  // Luigi's change: added set in order to avoid pushing many instances of the same KF when considering points, lines, etc
                for(KeyFramePtr pCovKFi : vpCovKFi)
                {
                    for(MapPointPtr pCovMPij : pCovKFi->GetMapPointMatches())
                    {
                        if(!pCovMPij || pCovMPij->isBad())
                            continue;

                        if(spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            //vpKeyFrames.push_back(pCovKFi); // Luigi's change 
                            spKeyFrames.insert(pCovKFi);
                        }
                    }

                    if(mpTracker->IsLineTracking())
                    {
                        for(MapLinePtr pCovMLij : pCovKFi->GetMapLineMatches())
                        {
                            if(!pCovMLij || pCovMLij->isBad())
                                continue;

			                if(spMapLines.find(pCovMLij) == spMapLines.end())
                            {
                                spMapLines.insert(pCovMLij);
                                vpMapLines.push_back(pCovMLij);
                                //vpKeyFrames.push_back(pCovKFi); // Luigi's change 
                                spKeyFrames.insert(pCovKFi);
                            }
                        }                        
                    }
                    
                    if(mpTracker->IsObjectTracking())
                    {
                        for(MapObjectPtr pCovMOij : pCovKFi->GetMapObjectMatches())
                        {
                            if(!pCovMOij || pCovMOij->isBad())
                                continue;

			                if(spMapObjects.find(pCovMOij) == spMapObjects.end())
                            {
                                spMapObjects.insert(pCovMOij);
                                vpMapObjects.push_back(pCovMOij);
                                //vpKeyFrames.push_back(pCovKFi); // Luigi's change 
                                spKeyFrames.insert(pCovKFi);
                            }
                        }                        
                    }                    
     
                }
                vpKeyFrames = vector<KeyFramePtr>(spKeyFrames.begin(),spKeyFrames.end()); // Luigi's change 

                //std::cout << "There are " << vpKeyFrames.size() <<" KFs which view all the mappoints" << std::endl;

                g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),solver.GetEstimatedTranslation().cast<double>(), (double) solver.GetEstimatedScale());
                g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
                g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                Sophus::Sim3f mScw = Converter::toSophus(gScw);

                vector<MapPointPtr> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPointPtr>(NULL));
                vector<KeyFramePtr> vpMatchedKF;
                vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFramePtr>(NULL));
                int numProjPointMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);
                //cout <<"BoW: " << numProjMatches << " matches between " << vpMapPoints.size() << " points with coarse Sim3" << endl;

                int numProjLineMatches = 0; 
                vector<MapLinePtr> vpMatchedML;    
                vector<KeyFramePtr> vpMatchedLinesKF;
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE                
                if(mpTracker->IsLineTracking())
                {
                    vpMatchedML.resize(mpCurrentKF->GetMapLineMatches().size(), static_cast<MapLinePtr>(NULL));                                        
                    vpMatchedLinesKF.resize(mpCurrentKF->GetMapLineMatches().size(), static_cast<KeyFramePtr>(NULL));
                    numProjLineMatches = pLineMatcher->SearchByProjection(mpCurrentKF, mScw, vpMapLines, vpKeyFrames, vpMatchedML, vpMatchedLinesKF, 8, 1.5);
                }
#endif    
                int numProjObjectMatches = 0; 
                vector<MapObjectPtr> vpMatchedMO;    
                vector<KeyFramePtr> vpMatchedObjectsKF;
#if USE_OBJECTS_FOR_VOTING_LOOP_CLOSURE                
                if(mpTracker->IsObjectTracking())
                {
                    /// < TODO: Luigi add objects matching 
                }
#endif                   
                int numProjFeatureMatches = (numProjPointMatches + Tracking::sknLineTrackWeigth*numProjLineMatches);
                        
                if(numProjFeatureMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = mbFixScale;
                    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                        bFixedScale=false;

                    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

                    if(numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
                        g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                        Sophus::Sim3f mScw = Converter::toSophus(gScw);

                        vector<MapPointPtr> vpMatchedMP;
                        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPointPtr>(NULL));
                        int numProjOptPointMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                        vector<MapLinePtr> vpMatchedML;
                        int numProjOptLineMatches = 0; 
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE                              
                        if(mpTracker->IsLineTracking())
			{
                            vpMatchedML.resize(mpCurrentKF->GetMapLineMatches().size(), static_cast<MapLinePtr>(NULL));
                            LineMatcher lineMatcher(0.9);
                            numProjOptLineMatches = lineMatcher.SearchByProjection(mpCurrentKF, mScw, vpMapLines, vpMatchedML, 5, 1.0);
                        }
#endif                      
#if USE_OBJECTS_FOR_VOTING_LOOP_CLOSURE                
                        if(mpTracker->IsObjectTracking())
                        {
                            /// < TODO: Luigi add objects matching 
                        }
#endif                            
                        int numProjOptFeatureMatches = (numProjOptPointMatches + Tracking::sknLineTrackWeigth*numProjOptLineMatches);
                        
                        if( numProjOptFeatureMatches >= nProjOptMatches)
                        {
#if 0
			    // TODO: Luigi: check this, it seems useless
                            int max_x = -1, min_x = 1000000;
                            int max_y = -1, min_y = 1000000;
                            for(MapPointPtr pMPi : vpMatchedMP)
                            {
                                if(!pMPi || pMPi->isBad())
                                {
                                    continue;
                                }

                                tuple<size_t,size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                                int index = get<0>(indexes);
                                if(index >= 0)
                                {
                                    int coord_x = pKFi->mvKeysUn[index].pt.x;
                                    if(coord_x < min_x)
                                    {
                                        min_x = coord_x;
                                    }
                                    if(coord_x > max_x)
                                    {
                                        max_x = coord_x;
                                    }
                                    int coord_y = pKFi->mvKeysUn[index].pt.y;
                                    if(coord_y < min_y)
                                    {
                                        min_y = coord_y;
                                    }
                                    if(coord_y > max_y)
                                    {
                                        max_y = coord_y;
                                    }
                                }
                            }
#endif 
                            int nNumKFs = 0;
                            //vpMatchedMPs = vpMatchedMP;
                            //vpMPs = vpMapPoints;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            vector<KeyFramePtr> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

                            int j = 0;
                            while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                            {
                                KeyFramePtr pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                                g2o::Sim3 gSjc(mTjc.unit_quaternion(),mTjc.translation(),1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPointPtr> vpMatchedMPs_j;
                                vector<MapLinePtr> vpMatchedMLs_j;
                                vector<MapObjectPtr> vpMatchedMOs_j;
                                bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j,
                                                                                                                           vpMapLines, vpMatchedMLs_j,
                                                                                                                           vpMapObjects, vpMatchedMOs_j);

                                if(bValid)
                                {
                                    Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                                    Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                                    Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                    Eigen::Vector3f vector_dist = Tc_cj.translation();
                                    nNumKFs++;
                                }
                                j++;
                            }

                            if(nNumKFs < 3)
                            {
                                vnStage[index] = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            if(nBestMatchesReproj < numProjOptFeatureMatches)
                            {
                                nBestMatchesReproj = numProjOptFeatureMatches;
                                nBestNumCoindicendes = nNumKFs;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                                
                                vpBestMapLines = vpMapLines;
                                vpBestMatchedMapLines = vpMatchedML;      
                                
                                vpBestMapObjects = vpMapObjects;
                                vpBestMatchedMapObjects = vpMatchedMO;                                  
                            }
                        }
                    }
                }
            }
            /*else
            {
                Verbose::PrintMess("BoW candidate: it don't match with the current one", Verbose::VERBOSITY_DEBUG);
            }*/
        }
        index++;
    }

    if(nBestMatchesReproj > 0)
    {
        pLastCurrentKF = mpCurrentKF;
        nNumCoincidences = nBestNumCoindicendes;
        pMatchedKF2 = pBestMatchedKF;
        pMatchedKF2->SetNotErase();
        g2oScw = g2oBestScw;
        
        vpMPs = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;
        
        vpMLs = vpBestMapLines;
        vpMatchedMLs = vpBestMatchedMapLines;    
        
        vpMOs = vpBestMapObjects;
        vpMatchedMOs = vpBestMatchedMapObjects;            

        return nNumCoincidences >= 3;
    }
    else
    {
        int maxStage = -1;
        int maxMatched;
        for(int i=0; i<vnStage.size(); ++i)
        {
            if(vnStage[i] > maxStage)
            {
                maxStage = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }
    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs,
                                                std::vector<MapLinePtr> &vpMLs, std::vector<MapLinePtr> &vpMatchedMLs,
                                                std::vector<MapObjectPtr> &vpMOs, std::vector<MapObjectPtr> &vpMatchedMOs)
{
    set<MapPointPtr> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
    set<MapLinePtr> spAlreadyMatchedMLs(vpMatchedMLs.begin(), vpMatchedMLs.end());
    set<MapObjectPtr> spAlreadyMatchedMOs(vpMatchedMOs.begin(), vpMatchedMOs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs,
                                                                            spAlreadyMatchedMLs, vpMLs, vpMatchedMLs,
                                                                            spAlreadyMatchedMOs, vpMOs, vpMatchedMOs);

    int nProjMatches = 30;
    if(nNumProjMatches >= nProjMatches)
    {
        return true;
    }

    return false;
}

int LoopClosing::FindMatchesByProjection(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKFw, g2o::Sim3 &g2oScw,
                                std::set<MapPointPtr> &spMatchedMPinOrigin, std::vector<MapPointPtr> &vpMapPoints, std::vector<MapPointPtr> &vpMatchedMapPoints,
                                std::set<MapLinePtr> &spMatchedMLinOrigin, std::vector<MapLinePtr> &vpMapLines, std::vector<MapLinePtr> &vpMatchedMapLines,
                                std::set<MapObjectPtr> &spMatchedMOinOrigin, std::vector<MapObjectPtr> &vpMapObjects, std::vector<MapObjectPtr> &vpMatchedMapObjects)
{
    int nNumCovisibles = 10;
    vector<KeyFramePtr> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFramePtr> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFramePtr> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    if(nInitialCov < nNumCovisibles)
    {
        for(int i=0; i<nInitialCov; ++i)
        {
            vector<KeyFramePtr> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
            int nInserted = 0;
            int j = 0;
            while(j < vpKFs.size() && nInserted < nNumCovisibles)
            {
                if(spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
                {
                    spCheckKFs.insert(vpKFs[j]);
                    ++nInserted;
                }
                ++j;
            }
            vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
        }
    }
    set<MapPointPtr> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE
    set<MapLinePtr> spMapLines;      
    if(mpTracker->IsLineTracking())
    {
        vpMapLines.clear();
        vpMatchedMapLines.clear();
    }   
#endif
#if USE_OBJECTS_FOR_VOTING_LOOP_CLOSURE
    set<MapObjectPtr> spMapObjects;      
    if(mpTracker->IsObjectTracking())
    {
        vpMapObjects.clear();
        vpMatchedMapObjects.clear();
    }   
#endif
    for(KeyFramePtr pKFi : vpCovKFm)
    {
        for(MapPointPtr pMPij : pKFi->GetMapPointMatches())
        {
            if(!pMPij || pMPij->isBad())
                continue;

            if(spMapPoints.find(pMPij) == spMapPoints.end())
            {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE
        if(mpTracker->IsLineTracking())
        {
            for(MapLinePtr pMLij : pKFi->GetMapLineMatches())
            {
                if(!pMLij || pMLij->isBad())
                    continue;

                if(spMapLines.find(pMLij) == spMapLines.end())
                {
                    spMapLines.insert(pMLij);
                    vpMapLines.push_back(pMLij);
                }
            }        
        }
#endif      
        
#if USE_OBJECTS_FOR_VOTING_LOOP_CLOSURE
        if(mpTracker->IsObjectTracking())
        {
            for(MapObjectPtr pMOij : pKFi->GetMapObjectMatches())
            {
                if(!pMOij || pMOij->isBad())
                    continue;

                if(spMapObjects.find(pMOij) == spMapObjects.end())
                {
                    spMapObjects.insert(pMOij);
                    vpMapObjects.push_back(pMOij);
                }
            }        
        }
#endif          
    }

    Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
    ORBmatcher matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPointPtr>(NULL));
    int numPointMatches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

    int numLineMatches = 0;
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE    
    if(mpTracker->IsLineTracking())
    {
        vpMatchedMapLines.resize(pCurrentKF->GetMapLineMatches().size(), static_cast<MapLinePtr>(NULL));
        LineMatcher lineMatcher(0.9);
        numLineMatches = lineMatcher.SearchByProjection(pCurrentKF, mScw, vpMapLines, vpMatchedMapLines, 3, 1.5);
    }
#endif
    
#if USE_OBJECTS_FOR_VOTING_LOOP_CLOSURE
    if(mpTracker->IsObjectTracking())
    {
        /// < TODO: Luigi add objects matching 
        // indeed, we need just to check if a given object has been found in the current KF 
    }    
#endif     
    
    return numPointMatches + Tracking::sknLineTrackWeigth*numLineMatches;
}

void LoopClosing::CorrectLoop()
{
    cout << "**************" << endl;
    cout << "Loop detected!" << endl;
    cout << "**************" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();
    mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
	cout << "Request GBA abort" << endl;
        cout << "Stoping Global Bundle Adjustment...";
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            cout << "GBA running... Abort!" << endl;
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        cout << "  Done!!" << endl;
    }

#if VERBOSE
    cout << "LoopClosing::CorrectLoop() - stopping local mapper " << endl; 
#endif
    
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    
#if VERBOSE    
    cout << "LoopClosing::CorrectLoop() - local mapper stopped" << endl; 
#endif

    // Ensure current keyframe is updated
    //cout << "Start updating connections" << endl;
    //assert(mpCurrentKF->GetMap()->CheckEssentialGraph());
    mpCurrentKF->UpdateConnections();
    //assert(mpCurrentKF->GetMap()->CheckEssentialGraph());

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - updated connections" << endl; 
#endif

    // Retrieve keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    //std::cout << "Loop: number of connected KFs -> " + to_string(mvpCurrentConnectedKFs.size()) << std::endl;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oLoopScw;
    Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
    Sophus::SE3f Tcw = mpCurrentKF->GetPose();
    g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>(),1.0);
    NonCorrectedSim3[mpCurrentKF]=g2oScw;

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
    Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),mg2oLoopScw.translation() / mg2oLoopScw.scale());
    mpCurrentKF->SetPose(correctedTcw.cast<float>());

    Map* pLoopMap = mpCurrentKF->GetMap();

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - updating map" << endl; 
#endif    

#ifdef REGISTER_TIMES
    /*KeyFramePtr pKF = mpCurrentKF;
    int numKFinLoop = 0;
    while(pKF && pKF->mnId > mpLoopMatchedKF->mnId)
    {
        pKF = pKF->GetParent();
        numKFinLoop += 1;
    }
    vnLoopKFs.push_back(numKFinLoop);*/

    std::chrono::steady_clock::time_point time_StartFusion = std::chrono::steady_clock::now();
#endif

    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized();

        for(vector<KeyFramePtr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFramePtr pKFi = *vit;

            if(pKFi!=mpCurrentKF)
            {
                Sophus::SE3f Tiw = pKFi->GetPose();
                Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                g2o::Sim3 g2oSic(Tic.unit_quaternion(),Tic.translation(),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oLoopScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                pKFi->SetPose(correctedTiw.cast<float>());

                //Pose without correction
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(),Tiw.translation().cast<double>(),1.0);
                NonCorrectedSim3[pKFi]=g2oSiw;
            }  
        }

        // Correct all MapPoints, MapLines and MapObjects observed by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];
            
            // Transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3f eigR = g2oCorrectedSiw.rotation().toRotationMatrix().cast<float>();
            Eigen::Vector3f eigt = g2oCorrectedSiw.translation().cast<float>();
            const double s = g2oCorrectedSiw.scale();
            eigt *=(1./s); //[R t/s;0 1]
            //cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);
            
            Eigen::Matrix3f correctedRwi, Riw; 
            Eigen::Vector3f correctedtwi, tiw; 
            if( mpTracker->IsObjectTracking() )
            {
                Riw = g2oSiw.rotation().toRotationMatrix().cast<float>(); // not yet corrected
                tiw = g2oSiw.translation().cast<float>() / g2oSiw.scale();                
            
                correctedRwi = eigR.transpose(); 
                correctedtwi = -correctedRwi * eigt; 
            }           

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            /*Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
            pKFi->SetPose(correctedTiw.cast<float>());*/

            // correct MapPoints 
            vector<MapPointPtr> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPointPtr pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }
            
            // correct MapLines 
            if(mpTracker->IsLineTracking())
            {
                vector<MapLinePtr> vpMLsi = pKFi->GetMapLineMatches();
                for(size_t iML=0, endMLi = vpMLsi.size(); iML<endMLi; iML++)
                {
                    MapLinePtr pMLi = vpMLsi[iML];
                    if(!pMLi)
                        continue;
                    if(pMLi->isBad())
                        continue;
                    if(pMLi->mnCorrectedByKF==mpCurrentKF->mnId)
                        continue;

                    // Project with non-corrected pose and project back with corrected pose
                    Eigen::Vector3f P3DSwf, P3DEwf;
                    pMLi->GetWorldEndPoints(P3DSwf, P3DEwf);   
                    const Eigen::Vector3d P3DSw = P3DSwf.cast<double>();
                    const Eigen::Vector3d P3DEw = P3DEwf.cast<double>();                                       
                    
                    const Eigen::Vector3d eigCorrectedP3DSw = g2oCorrectedSwi.map(g2oSiw.map(P3DSw));
                    const Eigen::Vector3d eigCorrectedP3DEw = g2oCorrectedSwi.map(g2oSiw.map(P3DEw));

                    pMLi->SetWorldEndPoints(eigCorrectedP3DSw.cast<float>(), eigCorrectedP3DEw.cast<float>());                    
                    pMLi->mnCorrectedByKF = mpCurrentKF->mnId;
                    pMLi->mnCorrectedReference = pKFi->mnId;
                    pMLi->UpdateNormalAndDepth();
                }                
            }
            
            // Correct MapObjects
            if(mpTracker->IsObjectTracking())
            {
                vector<MapObjectPtr> vpMOsi = pKFi->GetMapObjectMatches();

                for(size_t iMO=0, endMOi = vpMOsi.size(); iMO<endMOi; iMO++)
                {
                    MapObjectPtr pMOi = vpMOsi[iMO];
                    if(!pMOi)
                        continue;
                    if(pMOi->isBad())
                        continue;
                    if(pMOi->mnCorrectedByKF==mpCurrentKF->mnId)
                        continue;
                    
                    // "Map" to non-corrected camera                            
                    Eigen::Matrix3f Rwo;
                    Eigen::Vector3f two; 
                    double scale = pMOi->GetScale();
                    Rwo = pMOi->GetInverseRotation();
                    two = pMOi->GetInverseTranslation();
                    const Eigen::Matrix3f Rio = Riw*Rwo;
                    const Eigen::Vector3f tio = Riw*two+tiw;  

                    // "Backproject" using corrected camera                         
                    const Eigen::Matrix3f RwoNew = correctedRwi*Rio;
                    const Eigen::Vector3f twoNew = correctedRwi*tio+correctedtwi;  

                    pMOi->SetSim3InversePose(RwoNew, twoNew, scale);  // NOTE: s has been already integrated in the translation           
                    
                    pMOi->mnCorrectedByKF = mpCurrentKF->mnId;
                    pMOi->mnCorrectedReference = pKFi->mnId;                    
                    
                }
            }

            // Correct velocity according to orientation correction
            if(bImuInit)
            {
                Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse()*g2oSiw.rotation()).cast<float>();
                pKFi->SetVelocity(Rcor*pKFi->GetVelocity());
            }

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }
        // TODO Check this index increasement
        mpAtlas->GetCurrentMap()->IncreaseChangeIndex();


        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpLoopMatchedMPs.size(); i++)
        {
            if(mvpLoopMatchedMPs[i])
            {
                MapPointPtr pLoopMP = mvpLoopMatchedMPs[i];
                MapPointPtr pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

        if(mpTracker->IsLineTracking())
        {
            // Update matched map lines and replace if duplicated
            for(size_t i=0; i<mvpLoopMatchedMLs.size(); i++)
            {
                if(mvpLoopMatchedMLs[i])
                {
                    MapLinePtr pLoopML = mvpLoopMatchedMLs[i];
                    MapLinePtr pCurML = mpCurrentKF->GetMapLine(i);
                    if(pCurML)
                        pCurML->Replace(pLoopML);
                    else
                    {
                        mpCurrentKF->AddMapLine(pLoopML,i);   
                        pLoopML->AddObservation(mpCurrentKF,i);
                        pLoopML->ComputeDistinctiveDescriptors();
                    }
                }                
            }
        }
        
        /*
        if(mpTracker->IsObjectTracking())
        {
            // Update matched map objects and replace if duplicated
            for(size_t i=0; i<mvpLoopMatchedMOs.size(); i++)
            {
                if(mvpLoopMatchedMOs[i])
                {
                    MapObjectPtr pLoopMO = mvpLoopMatchedMOs[i];
                    MapObjectPtr pCurMO = mpCurrentKF->GetMapObject(i);
                    if(pCurMO)
                        pCurMO->Replace(pLoopMO);
                    else
                    {
                        mpCurrentKF->AddMapObject(pLoopMO,i);   
                        pLoopMO->AddObservation(mpCurrentKF,i);
                        //pLoopMO->ComputeDistinctiveDescriptors();
                    }
                }                
            }
        }*/

        //cout << "LC: end replacing duplicated" << endl;
    }

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - searching and fusing features " << endl; 
#endif      
    
    // Project MapPoints and MapLines observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3, mvpLoopMapPoints, mvpLoopMapLines, mvpLoopMapObjects);

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - updating covisibility graph" << endl; 
#endif  

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFramePtr, set<KeyFramePtr> > LoopConnections;

    for(vector<KeyFramePtr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFramePtr pKFi = *vit;
        vector<KeyFramePtr> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFramePtr>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFramePtr>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - optimizing pose graph " << endl; 
#endif     

    // Optimize graph
    bool bFixedScale = mbFixScale;
    // TODO CHECK; Solo para el monocular inertial
    if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
        bFixedScale=false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndFusion = std::chrono::steady_clock::now();

        double timeFusion = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndFusion - time_StartFusion).count();
        vdLoopFusion_ms.push_back(timeFusion);
#endif
    //cout << "Optimize essential graph" << endl;
    if(pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
    {
        cout << "With 4DoF" << endl;
        Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
    }
    else
    {
        cout << "With 7DoF" << endl;
        //cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
        Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndOpt = std::chrono::steady_clock::now();

    double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOpt - time_EndFusion).count();
    vdLoopOptEss_ms.push_back(timeOptEss);
#endif

    mpAtlas->InformNewBigChange();

    // Add loop edge
    mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
    if(!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1))
    {
#if VERBOSE     
        cout << "LoopClosing::CorrectLoop() - running bundle adjustment " << endl; 
#endif   
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mnCorrectionGBA = mnNumCorrection;

        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
#if VERBOSE     
        cout << "LoopClosing::CorrectLoop() - bundle adjustment done - releasing local mapper" << endl; 
#endif       
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm
}


void LoopClosing::StartGlobalBundleAdjustment()
{
    cout << "****************" << endl;
    cout << "Start Global BA!" << endl;
    cout << "****************" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

#if VERBOSE
    cout << "LoopClosing::StartGlobalBundleAdjustment() - stopping local mapper " << endl; 
#endif
    
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    
#if VERBOSE    
    cout << "LoopClosing::StartGlobalBundleAdjustment() - local mapper stopped" << endl; 
#endif

    
    //mpMap->InformNewBigChange();


#if VERBOSE     
    cout << "LoopClosing::StartGlobalBundleAdjustment() - running bundle adjustment " << endl; 
#endif     
    
    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    // < TODO: ?Luigi add the possibility to BA all the maps?
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, mpAtlas->GetCurrentMap(), mpCurrentKF->mnId);

#if VERBOSE     
    cout << "LoopClosing::StartGlobalBundleAdjustment() - bundle adjustment done - releasing local mapper" << endl; 
#endif       
    
    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm    
}

void LoopClosing::MergeLocal()
{
    Verbose::PrintMess("MERGE: Merge Visual detected!!!!", Verbose::VERBOSITY_NORMAL);
    //mpTracker->SetStepByStep(true);

    int numTemporalKFs = 25; //Temporal KFs in the local window if the map is inertial.

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    KeyFramePtr pNewChild;
    KeyFramePtr pNewParent;

    vector<KeyFramePtr> vpLocalCurrentWindowKFs;
    vector<KeyFramePtr> vpMergeConnectedKFs;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }

    Verbose::PrintMess("MERGE-VISUAL: Request Stop Local Mapping", Verbose::VERBOSITY_DEBUG);
    //cout << "Request Stop Local Mapping" << endl;
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    Verbose::PrintMess("MERGE: Local Map stopped", Verbose::VERBOSITY_DEBUG);

    mpLocalMapper->EmptyQueue();

    // Merge map will become the new active map with the local window of KFs and MPs from the current map.
    // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    //std::cout << "Merge local, Active map: " << pCurrentMap->GetId() << std::endl;
    //std::cout << "Merge local, Non-Active map: " << pMergeMap->GetId() << std::endl;

    std::string messageLinesActiveMap; 
    std::string messageLinesMatchedMap; 
    if(mpTracker->IsLineTracking())
    {
        messageLinesActiveMap = "and " + to_string(pCurrentMap->MapLinesInMap()) + " MLs in the active map";
        messageLinesMatchedMap = "and " + to_string(pMergeMap->MapLinesInMap()) + " MLs in the matched map";        
    }
    std::string messageObjectsActiveMap; 
    std::string messageObjectssMatchedMap; 
    if(mpTracker->IsObjectTracking())
    {
        messageObjectsActiveMap = "and " + to_string(pCurrentMap->MapObjectsInMap()) + " MOs in the active map";
        messageObjectssMatchedMap = "and " + to_string(pMergeMap->MapObjectsInMap()) + " MOs in the matched map";        
    }    
    
    Verbose::PrintMess("MERGE: Initially there are " + to_string(pCurrentMap->KeyFramesInMap()) + " KFs and " + to_string(pCurrentMap->MapPointsInMap()) + " MPs in the active map " + messageLinesActiveMap + " " + messageObjectsActiveMap, Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("MERGE: Initially there are " + to_string(pMergeMap->KeyFramesInMap()) + " KFs and " + to_string(pMergeMap->MapPointsInMap()) + " MPs in the matched map " + messageLinesMatchedMap + " " + messageObjectssMatchedMap, Verbose::VERBOSITY_DEBUG);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();
#endif

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    //Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
    set<KeyFramePtr> spLocalWindowKFs;
    //Get MPs in the welding area from the current map
    set<MapPointPtr> spLocalWindowMPs;
    //Get MLs in the welding area from the current map
    set<MapLinePtr> spLocalWindowMLs;    
    //Get MOs in the welding area from the current map
    set<MapObjectPtr> spLocalWindowMOs;   
    
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        KeyFramePtr pKFi = mpCurrentKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;

            set<MapPointPtr> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
            if(mpTracker->IsLineTracking())
            {
                set<MapLinePtr> spMLi = pKFi->GetMapLines();
                spLocalWindowMLs.insert(spMLi.begin(), spMLi.end());                
            }
            if(mpTracker->IsObjectTracking())
            {
                set<MapObjectPtr> spMOi = pKFi->GetMapObjects();
                spLocalWindowMOs.insert(spMOi.begin(), spMOi.end());                
            }            
        }

        pKFi = mpCurrentKF->mNextKF;
        while(pKFi)
        {
            spLocalWindowKFs.insert(pKFi);

            set<MapPointPtr> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

            if(mpTracker->IsLineTracking())
            {
                set<MapLinePtr> spMLi = pKFi->GetMapLines();
                spLocalWindowMLs.insert(spMLi.begin(), spMLi.end());                
            }        
            if(mpTracker->IsObjectTracking())
            {
                set<MapObjectPtr> spMOi = pKFi->GetMapObjects();
                spLocalWindowMOs.insert(spMOi.begin(), spMOi.end());                
            }       

            pKFi = mpCurrentKF->mNextKF;
        }
    }
    else
    {
        spLocalWindowKFs.insert(mpCurrentKF);
    }

    vector<KeyFramePtr> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    spLocalWindowKFs.insert(mpCurrentKF);

    Verbose::PrintMess("MERGE: Initial number of KFs in local window from active map: " + to_string(spLocalWindowKFs.size()), Verbose::VERBOSITY_DEBUG);
    const int nMaxTries = 5;
    int nNumTries = 0;
    while(spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFramePtr> vpNewCovKFs;
        vpNewCovKFs.empty();
        for(KeyFramePtr pKFi : spLocalWindowKFs)
        {
            vector<KeyFramePtr> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(KeyFramePtr pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }

        spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }
    Verbose::PrintMess("MERGE: Last number of KFs in local window from the active map: " + to_string(spLocalWindowKFs.size()), Verbose::VERBOSITY_DEBUG);

    for(KeyFramePtr pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        set<MapPointPtr> spMPs = pKFi->GetMapPoints();
        spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
        
        if(mpTracker->IsLineTracking())
        {
            set<MapLinePtr> spMLi = pKFi->GetMapLines();
            spLocalWindowMLs.insert(spMLi.begin(), spMLi.end());                
        }     
        if(mpTracker->IsObjectTracking())
        {
            set<MapObjectPtr> spMOi = pKFi->GetMapObjects();
            spLocalWindowMOs.insert(spMOi.begin(), spMOi.end());                
        }            
    }

    //std::cout << "[Merge]: Ma = " << to_string(pCurrentMap->GetId()) << "; #KFs = " << to_string(spLocalWindowKFs.size()) << "; #MPs = " << to_string(spLocalWindowMPs.size()) << std::endl;

    std::string messageLocalWindowLinesActiveMap; 
    std::string messageLocalWindowObjectsActiveMap;     
    if(mpTracker->IsLineTracking())
    {
        messageLocalWindowLinesActiveMap = "; Number of MLs in local window from active map: " + to_string(spLocalWindowMLs.size());
        messageLinesActiveMap            = "; Number of MLs in the active map: " + to_string(pCurrentMap->GetAllMapLines().size());       
    }    
    if(mpTracker->IsObjectTracking())
    {
        messageLocalWindowObjectsActiveMap = "; Number of MOs in local window from active map: " + to_string(spLocalWindowMOs.size());
        messageObjectsActiveMap            = "; Number of MOs in the active map: " + to_string(pCurrentMap->GetAllMapObjects().size());       
    }      
    Verbose::PrintMess("MERGE: Number of MPs in local window from active map: " + to_string(spLocalWindowMPs.size()) + messageLocalWindowLinesActiveMap + " " + messageLocalWindowObjectsActiveMap , Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("MERGE: Number of MPs in the active map: " + to_string(pCurrentMap->GetAllMapPoints().size()) + messageLinesActiveMap + " " + messageObjectsActiveMap, Verbose::VERBOSITY_DEBUG);

    set<KeyFramePtr> spMergeConnectedKFs;
    if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
    {
        KeyFramePtr pKFi = mpMergeMatchedKF;
        int nInserted = 0;
        while(pKFi && nInserted < numTemporalKFs/2)
        {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;
        }

        pKFi = mpMergeMatchedKF->mNextKF;
        while(pKFi && nInserted < numTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mNextKF;
        }
    }
    else
    {
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
    }
    vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    spMergeConnectedKFs.insert(mpMergeMatchedKF);

    Verbose::PrintMess("MERGE: Initial number of KFs in the local window from matched map: " + to_string(spMergeConnectedKFs.size()), Verbose::VERBOSITY_DEBUG);
    
    nNumTries = 0;
    while(spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFramePtr> vpNewCovKFs;
        for(KeyFramePtr pKFi : spMergeConnectedKFs)
        {
            vector<KeyFramePtr> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            for(KeyFramePtr pKFcov : vpKFiCov)
            {
                if(pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }

            }
        }

        spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }
    Verbose::PrintMess("MERGE: Last number of KFs in the localwindow from matched map: " + to_string(spMergeConnectedKFs.size()), Verbose::VERBOSITY_DEBUG);

    set<MapPointPtr> spMapPointMerge;
    set<MapLinePtr> spMapLineMerge;     
    set<MapObjectPtr> spMapObjectMerge;       
    for(KeyFramePtr pKFi : spMergeConnectedKFs)
    {
        set<MapPointPtr> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
        
        if(mpTracker->IsLineTracking())
        {
            set<MapLinePtr> vpMLs = pKFi->GetMapLines();
            spMapLineMerge.insert(vpMLs.begin(),vpMLs.end());
        }
        
        if(mpTracker->IsObjectTracking())
        {
            set<MapObjectPtr> vpMOs = pKFi->GetMapObjects();
            spMapObjectMerge.insert(vpMOs.begin(),vpMOs.end());
        }        
    }

    vector<MapPointPtr> vpCheckFuseMapPoint;
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    vector<MapLinePtr> vpCheckFuseMapLine;
    if(mpTracker->IsLineTracking())
    {
        vpCheckFuseMapLine.reserve(spMapLineMerge.size());
        std::copy(spMapLineMerge.begin(), spMapLineMerge.end(), std::back_inserter(vpCheckFuseMapLine));
    }   
    
    vector<MapObjectPtr> vpCheckFuseMapObject;
    if(mpTracker->IsObjectTracking())
    {
        vpCheckFuseMapObject.reserve(spMapObjectMerge.size());
        std::copy(spMapObjectMerge.begin(), spMapObjectMerge.end(), std::back_inserter(vpCheckFuseMapObject));
    }    

    //std::cout << "[Merge]: Mm = " << to_string(pMergeMap->GetId()) << "; #KFs = " << to_string(spMergeConnectedKFs.size()) << "; #MPs = " << to_string(spMapPointMerge.size()) << std::endl;


    //
    Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
    g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(),Twc.translation(),1.0);
    g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
    g2o::Sim3 g2oCorrectedScw = mg2oMergeScw; //TODO Check the transformation

    KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
    vCorrectedSim3[mpCurrentKF]=g2oCorrectedScw;
    vNonCorrectedSim3[mpCurrentKF]=g2oNonCorrectedScw;


#ifdef REGISTER_TIMES
    vnMergeKFs.push_back(spLocalWindowKFs.size() + spMergeConnectedKFs.size());
    vnMergeMPs.push_back(spLocalWindowMPs.size() + spMapPointMerge.size());
#endif
    for(KeyFramePtr pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
        {
            Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
            continue;
        }

        if(pKFi->GetMap() != pCurrentMap)
            Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

        g2o::Sim3 g2oCorrectedSiw;

        if(pKFi!=mpCurrentKF)
        {
            Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
            //Pose without correction
            vNonCorrectedSim3[pKFi]=g2oSiw;

            Sophus::SE3d Tic = Tiw*Twc;
            g2o::Sim3 g2oSic(Tic.unit_quaternion(),Tic.translation(),1.0);
            g2oCorrectedSiw = g2oSic*mg2oMergeScw;
            vCorrectedSim3[pKFi]=g2oCorrectedSiw;
        }
        else
        {
            g2oCorrectedSiw = g2oCorrectedScw;
        }
        pKFi->mTcwMerge  = pKFi->GetPose();

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        double s = g2oCorrectedSiw.scale();
        pKFi->mfScale = s;
        Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

        pKFi->mTcwMerge = correctedTiw.cast<float>();

        if(pCurrentMap->isImuInitialized())
        {
            Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
            pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
        }

        //TODO DEBUG to know which are the KFs that had been moved to the other map
    }

    int numPointsWithCorrection = 0;

    //for(MapPointPtr pMPi : spLocalWindowMPs)
    set<MapPointPtr>::iterator itMP = spLocalWindowMPs.begin();
    while(itMP != spLocalWindowMPs.end())
    {
        MapPointPtr pMPi = *itMP;
        if(!pMPi || pMPi->isBad())
        {
            itMP = spLocalWindowMPs.erase(itMP);
            continue;
        }

        KeyFramePtr pKFref = pMPi->GetReferenceKeyFrame();
        if(vCorrectedSim3.find(pKFref) == vCorrectedSim3.end())
        {
            itMP = spLocalWindowMPs.erase(itMP);
            numPointsWithCorrection++;
            continue;
        }
        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

        // Project with non-corrected pose and project back with corrected pose
        Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
        Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

        pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
        pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

        itMP++;
    }
    /*if(numPointsWithCorrection>0)
    {
        std::cout << "[Merge]: " << std::to_string(numPointsWithCorrection) << " points removed from Ma due to its reference KF is not in welding area" << std::endl;
        std::cout << "[Merge]: Ma has " << std::to_string(spLocalWindowMPs.size()) << " points" << std::endl;
    }*/

    int numLinesWithCorrection = 0;

    if(mpTracker->IsLineTracking())
    {
        set<MapLinePtr>::iterator itML = spLocalWindowMLs.begin();
        while(itML != spLocalWindowMLs.end())
        {
            MapLinePtr pMLi = *itML;
            if(!pMLi || pMLi->isBad())
            {
                itML = spLocalWindowMLs.erase(itML);
                continue;
            }

            KeyFramePtr pKFref = pMLi->GetReferenceKeyFrame();
            if(vCorrectedSim3.find(pKFref) == vCorrectedSim3.end())
            {
                itML = spLocalWindowMLs.erase(itML);
                numLinesWithCorrection++;
                continue;
            }
            g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
            g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

            // Project with non-corrected pose and project back with corrected pose
            Eigen::Vector3f P3DStartwf, P3DEndwf;
            pMLi->GetWorldEndPoints(P3DStartwf, P3DEndwf);
            Eigen::Vector3d P3DStartw = P3DStartwf.cast<double>();
            Eigen::Vector3d P3DEndw = P3DEndwf.cast<double>();
            Eigen::Vector3d eigCorrectedP3DStartw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3DStartw));
            Eigen::Vector3d eigCorrectedP3DEndw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3DEndw));            
            Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

            pMLi->mPosStartMerge = eigCorrectedP3DStartw.cast<float>();
            pMLi->mPosEndMerge = eigCorrectedP3DEndw.cast<float>();
            pMLi->mNormalVectorMerge = Rcor.cast<float>() * pMLi->GetNormal();

            itML++;
        }        
    }

    int numObjectsWithCorrection = 0;

    if(mpTracker->IsObjectTracking())
    {
        //for(MapPointPtr pMOi : spLocalWindowMOs)
        set<MapObjectPtr>::iterator itMO = spLocalWindowMOs.begin();
        while(itMO != spLocalWindowMOs.end())
        {
            MapObjectPtr pMOi = *itMO;
            if(!pMOi || pMOi->isBad())
            {
                itMO = spLocalWindowMOs.erase(itMO);
                continue;
            }

            KeyFramePtr pKFref = pMOi->GetReferenceKeyFrame();
            if(vCorrectedSim3.find(pKFref) == vCorrectedSim3.end())
            {
                itMO = spLocalWindowMOs.erase(itMO);
                numObjectsWithCorrection++;
                continue;
            }
            //g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
            g2o::Sim3 g2oCorrectedSiw = vCorrectedSim3[pKFref];
            //g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

            // // Project with non-corrected pose and project back with corrected pose
            // Eigen::Vector3d P3Dw = pMOi->GetWorldPos().cast<double>();
            // Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
            // Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

            // pMOi->mPosMerge = eigCorrectedP3Dw.cast<float>();
            // pMOi->mNormalVectorMerge = Rcor.cast<float>() * pMOi->GetNormal();

            Eigen::Matrix3f correctedRwi, Riw;
            Eigen::Vector3f correctedtwi, tiw; 
                            
            // Transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3f eigRiw = g2oCorrectedSiw.rotation().toRotationMatrix().cast<float>();
            Eigen::Vector3f eigtiw = g2oCorrectedSiw.translation().cast<float>();
            double s = g2oCorrectedSiw.scale();
            eigtiw *=(1./s); //[R t/s;0 1]
            
            correctedRwi = eigRiw.transpose();    
            correctedtwi = -correctedRwi * eigtiw;             

            Sophus::SE3f Tiw = pKFref->GetPose(); // not yet corrected            
            Riw = Tiw.rotationMatrix();
            tiw = Tiw.translation();
            
            // "Map" to non-corrected camera                            
            Eigen::Matrix3f Rwo;
            Eigen::Vector3f two; 
            double scale = pMOi->GetScale();
            Rwo = pMOi->GetInverseRotation();
            two = pMOi->GetInverseTranslation();
            
            const Eigen::Matrix3f Rio = Riw*Rwo;
            const Eigen::Vector3f tio = Riw*two+tiw;  

            // "Backproject" using corrected camera                         
            const Eigen::Matrix3f RwoNew = correctedRwi*Rio;
            const Eigen::Vector3f twoNew = correctedRwi*tio+correctedtwi;  

            pMOi->mRwoMerge = RwoNew; 
            pMOi->mtwoMerge = twoNew; 
            pMOi->mScaleMerge = scale;   // s has been already integrated in the translation above!      

            itMO++;
        }   
    }
    
    
    {
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

        //std::cout << "Merge local window: " << spLocalWindowKFs.size() << std::endl;
        //std::cout << "[Merge]: init merging maps " << std::endl;
        for(KeyFramePtr pKFi : spLocalWindowKFs)
        {
            if(!pKFi || pKFi->isBad())
            {
                //std::cout << "Bad KF in correction" << std::endl;
                continue;
            }

            //std::cout << "KF id: " << pKFi->mnId << std::endl;

            pKFi->mTcwBefMerge = pKFi->GetPose();
            pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
            pKFi->SetPose(pKFi->mTcwMerge);

            // Make sure connections are updated
            pKFi->UpdateMap(pMergeMap);
            pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
            pMergeMap->AddKeyFrame(pKFi);
            pCurrentMap->EraseKeyFrame(pKFi);

            if(pCurrentMap->isImuInitialized())
            {
                pKFi->SetVelocity(pKFi->mVwbMerge);
            }
        }

        for(MapPointPtr pMPi : spLocalWindowMPs)
        {
            if(!pMPi || pMPi->isBad())
                continue;

            pMPi->SetWorldPos(pMPi->mPosMerge);
            pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
            pMPi->UpdateMap(pMergeMap);
            pMergeMap->AddMapPoint(pMPi);
            pCurrentMap->EraseMapPoint(pMPi);
        }
        
        if(mpTracker->IsLineTracking())
        {        
            for(MapLinePtr pMLi : spLocalWindowMLs)
            {
                if(!pMLi || pMLi->isBad())
                    continue;

                pMLi->SetWorldEndPoints(pMLi->mPosStartMerge, pMLi->mPosEndMerge);             
                pMLi->SetNormalVector(pMLi->mNormalVectorMerge);
                pMLi->UpdateMap(pMergeMap);
                pMergeMap->AddMapLine(pMLi);
                pCurrentMap->EraseMapLine(pMLi);
            }
        }
        
        if(mpTracker->IsObjectTracking())
        {        
            for(MapObjectPtr pMOi : spLocalWindowMOs)
            {
                if(!pMOi || pMOi->isBad())
                    continue;

                pMOi->SetSim3InversePose(pMOi->mRwoMerge, pMOi->mtwoMerge, pMOi->mScaleMerge);            
                
                pMOi->UpdateMap(pMergeMap);
                pMergeMap->AddMapObject(pMOi);
                pCurrentMap->EraseMapObject(pMOi);
            }
        }

        mpAtlas->ChangeMap(pMergeMap);
        mpAtlas->SetMapBad(pCurrentMap);
        pMergeMap->IncreaseChangeIndex();
        //TODO for debug
        pMergeMap->ChangeId(pCurrentMap->GetId());

        //std::cout << "[Merge]: merging maps finished" << std::endl;
    }

    //Rebuild the essential graph in the local window
    pCurrentMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpCurrentKF->ChangeParent(mpMergeMatchedKF);
    while(pNewChild)
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        KeyFramePtr pOldParent = pNewChild->GetParent();

        pNewChild->ChangeParent(pNewParent);

        pNewParent = pNewChild;
        pNewChild = pOldParent;

    }

    //Update the connections between the local window
    mpMergeMatchedKF->UpdateConnections();

    vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    //vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    //std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    /*
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));
    if(mpTracker->IsLineTracking())
    {
        vpCheckFuseMapLine.reserve(spMapLineMerge.size());
        std::copy(spMapLineMerge.begin(), spMapLineMerge.end(), std::back_inserter(vpCheckFuseMapLine));
    }
    if(mpTracker->IsObjectTracking())
    {
        vpCheckFuseMapObject.reserve(spMapObjectMerge.size());
        std::copy(spMapObjectMerge.begin(), spMapObjectMerge.end(), std::back_inserter(vpCheckFuseMapObject));
    }
    */    

    // Project MapPoints observed in the neighborhood of the merge keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    //std::cout << "[Merge]: start fuse points" << std::endl;
    SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint, vpCheckFuseMapLine, vpCheckFuseMapObject);
    //std::cout << "[Merge]: fuse points finished" << std::endl;

    // Update connectivity
    for(KeyFramePtr pKFi : spLocalWindowKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(KeyFramePtr pKFi : spMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    //std::cout << "[Merge]: Start welding bundle adjustment" << std::endl;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartWeldingBA = std::chrono::steady_clock::now();

    double timeMergeMaps = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_StartWeldingBA - time_StartMerge).count();
    vdMergeMaps_ms.push_back(timeMergeMaps);
#endif

    bool bStop = false;
    Verbose::PrintMess("MERGE: Start local BA ", Verbose::VERBOSITY_DEBUG);
    vpLocalCurrentWindowKFs.clear();
    vpMergeConnectedKFs.clear();
    std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
    std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
    if (mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
    {
        Optimizer::MergeInertialBA(mpCurrentKF,mpMergeMatchedKF,&bStop, pCurrentMap,vCorrectedSim3);
    }
    else
    {
        Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs,&bStop,( mpTracker->IsLineTracking() || mpTracker->IsObjectTracking() ));
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndWeldingBA = std::chrono::steady_clock::now();

    double timeWeldingBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndWeldingBA - time_StartWeldingBA).count();
    vdWeldingBA_ms.push_back(timeWeldingBA);
#endif
    //std::cout << "[Merge]: Welding bundle adjustment finished" << std::endl;

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    Verbose::PrintMess("MERGE: Finish the LBA", Verbose::VERBOSITY_DEBUG);


    ////
    //Update the non critical area from the current map to the merged map
    vector<KeyFramePtr> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
    vector<MapPointPtr> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();
    vector<MapLinePtr> vpCurrentMapMLs; 
    vector<MapObjectPtr> vpCurrentMapMOs;     
    if(mpTracker->IsLineTracking())
    {
        vpCurrentMapMLs = pCurrentMap->GetAllMapLines();
    }
    if(mpTracker->IsObjectTracking())
    {
        vpCurrentMapMOs = pCurrentMap->GetAllMapObjects();
    }    
    
    if(vpCurrentMapKFs.size() == 0)
    {
        Verbose::PrintMess("MERGE: There are not KFs outside of the welding area", Verbose::VERBOSITY_DEBUG);
    }
    else
    {
        Verbose::PrintMess("MERGE: Calculate the new position of the elements outside of the window", Verbose::VERBOSITY_DEBUG);
        //Apply the transformation
        if(mpTracker->mSensor == System::MONOCULAR)
        {
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information

            for(KeyFramePtr pKFi : vpCurrentMapKFs)
            {
                if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                {
                    continue;
                }

                g2o::Sim3 g2oCorrectedSiw;

                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
                //Pose without correction
                vNonCorrectedSim3[pKFi]=g2oSiw;

                Sophus::SE3d Tic = Tiw*Twc;
                g2o::Sim3 g2oSim(Tic.unit_quaternion(),Tic.translation(),1.0);
                g2oCorrectedSiw = g2oSim*mg2oMergeScw;
                vCorrectedSim3[pKFi]=g2oCorrectedSiw;

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                double s = g2oCorrectedSiw.scale();

                pKFi->mfScale = s;

                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / s);

                pKFi->mTcwBefMerge = pKFi->GetPose();
                pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                pKFi->SetPose(correctedTiw.cast<float>());

                if(pCurrentMap->isImuInitialized())
                {
                    Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
                    pKFi->SetVelocity(Rcor * pKFi->GetVelocity()); // TODO: should add here scale s
                }

            }
            for(MapPointPtr pMPi : vpCurrentMapMPs)
            {
                if(!pMPi || pMPi->isBad()|| pMPi->GetMap() != pCurrentMap)
                    continue;

                KeyFramePtr pKFref = pMPi->GetReferenceKeyFrame();
                g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                // Project with non-corrected pose and project back with corrected pose
                Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
                pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());

                pMPi->UpdateNormalAndDepth();
            }

                if(mpTracker->IsLineTracking())
                {
                    for(MapLinePtr pMLi : vpCurrentMapMLs)
                    {
                        if(!pMLi || pMLi->isBad()|| pMLi->GetMap() != pCurrentMap)
                            continue;

                        KeyFramePtr pKFref = pMLi->GetReferenceKeyFrame();
                        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                        // Project with non-corrected pose and project back with corrected pose
                        Eigen::Vector3f P3DSwf, P3DEwf;
                        pMLi->GetWorldEndPoints(P3DSwf, P3DEwf);   
                        Eigen::Vector3d P3DSw = P3DSwf.cast<double>();
                        Eigen::Vector3d P3DEw = P3DEwf.cast<double>();

                        Eigen::Matrix<double,3,1> eigCorrectedP3DSw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3DSw));
                        Eigen::Matrix<double,3,1> eigCorrectedP3DEw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3DEw));
                        pMLi->SetWorldEndPoints(eigCorrectedP3DSw.cast<float>(), eigCorrectedP3DEw.cast<float>());
                        
                        pMLi->UpdateNormalAndDepth();                        
                    }
                }
                
                if(mpTracker->IsObjectTracking())
                {
                    for(MapObjectPtr pMOi : vpCurrentMapMOs)
                    {
                        if(!pMOi || pMOi->isBad()|| pMOi->GetMap() != pCurrentMap)
                            continue;

                        KeyFramePtr pKFref = pMOi->GetReferenceKeyFrame();
                        g2o::Sim3 g2oCorrectedSiw = vCorrectedSim3[pKFref];
                        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                        Eigen::Matrix3f correctedRwi, Riw; 
                        Eigen::Vector3f correctedtwi, tiw; 

                        // Transform Sim3 to SE3 (scale translation)
                        Eigen::Matrix3f eigRiw = g2oCorrectedSiw.rotation().toRotationMatrix().cast<float>();
                        Eigen::Vector3f eigtiw = g2oCorrectedSiw.translation().cast<float>();
                        double s = g2oCorrectedSiw.scale();
                        eigtiw *=(1./s); //[R t/s;0 1]
                                    
                        correctedRwi = eigRiw.transpose();    
                        correctedtwi = -correctedRwi*eigtiw;             

                        Sophus::SE3f Tiw = pKFref->GetPose(); // not yet corrected            
                        // Riw = Tiw.rowRange(0,3).colRange(0,3);
                        // tiw = Tiw.rowRange(0,3).col(3);
                        Riw = Tiw.rotationMatrix(); 
                        tiw = Tiw.translation(); 

                        // "Map" to non-corrected camera                            
                        Eigen::Matrix3f Rwo; 
                        Eigen::Vector3f two; 
                        double scale = pMOi->GetScale();
                        Rwo = pMOi->GetInverseRotation();
                        two = pMOi->GetInverseTranslation();

                        const Eigen::Matrix3f Rio = Riw*Rwo;
                        const Eigen::Vector3f tio = Riw*two+tiw;  

                        // "Backproject" using corrected camera                         
                        const Eigen::Matrix3f RwoNew = correctedRwi*Rio;
                        const Eigen::Vector3f twoNew = correctedRwi*tio+correctedtwi;  
                        
                        pMOi->SetSim3InversePose(RwoNew, twoNew, scale);  // s has been already integrated in the translation above!
                    }
                }
                
        }

        mpLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        // Optimize graph (and update the loop position for each element form the begining to the end)
        if(mpTracker->mSensor != System::MONOCULAR)
        {
            // TODO: Luigi add map objects management
            Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs, vpCurrentMapMLs, vpCurrentMapMOs);
        }


        {
            // Get Merge Map Mutex
            unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            //std::cout << "Merge outside KFs: " << vpCurrentMapKFs.size() << std::endl;
            for(KeyFramePtr pKFi : vpCurrentMapKFs)
            {
                if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                {
                    continue;
                }
                //std::cout << "KF id: " << pKFi->mnId << std::endl;

                // Make sure connections are updated
                pKFi->UpdateMap(pMergeMap);
                pMergeMap->AddKeyFrame(pKFi);
                pCurrentMap->EraseKeyFrame(pKFi);
            }
            Verbose::PrintMess("MERGE: There are " + to_string(pMergeMap->MapPointsInMap()) + " MPs in the map", Verbose::VERBOSITY_DEBUG);
            Verbose::PrintMess("MERGE: It will be inserted " + to_string(vpCurrentMapMPs.size()) + " MPs in the map", Verbose::VERBOSITY_DEBUG);
            if(mpTracker->IsLineTracking())
                Verbose::PrintMess("MERGE: It will be inserted " + to_string(vpCurrentMapMLs.size()) + " MLs in the map", Verbose::VERBOSITY_DEBUG);
            if(mpTracker->IsObjectTracking())
                Verbose::PrintMess("MERGE: It will be inserted " + to_string(vpCurrentMapMOs.size()) + " MOs in the map", Verbose::VERBOSITY_DEBUG);            
            
            for(MapPointPtr pMPi : vpCurrentMapMPs)
            {
                if(!pMPi || pMPi->isBad())
                    continue;

                pMPi->UpdateMap(pMergeMap);
                pMergeMap->AddMapPoint(pMPi);
                pCurrentMap->EraseMapPoint(pMPi);
            }
            Verbose::PrintMess("MERGE: There are " + to_string(pMergeMap->MapPointsInMap()) + " MPs in the map", Verbose::VERBOSITY_DEBUG);
            
            if(mpTracker->IsLineTracking())
            {
                for(MapLinePtr pMLi : vpCurrentMapMLs)
                {
                    if(!pMLi || pMLi->isBad())
                        continue;

                    pMLi->UpdateMap(pMergeMap);
                    pMergeMap->AddMapLine(pMLi);
                    pCurrentMap->EraseMapLine(pMLi);
                }
                Verbose::PrintMess("MERGE: There are " + to_string(pMergeMap->MapLinesInMap()) + " MLs in the map", Verbose::VERBOSITY_DEBUG);                
            }
            
            if(mpTracker->IsObjectTracking())
            {
                for(MapObjectPtr pMOi : vpCurrentMapMOs)
                {
                    if(!pMOi || pMOi->isBad())
                        continue;

                    pMOi->UpdateMap(pMergeMap);
                    pMergeMap->AddMapObject(pMOi);
                    pCurrentMap->EraseMapObject(pMOi);
                }
                Verbose::PrintMess("MERGE: There are " + to_string(pMergeMap->MapObjectsInMap()) + " MOs in the map", Verbose::VERBOSITY_DEBUG);                
            }
        }
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndOptEss = std::chrono::steady_clock::now();

    double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndOptEss - time_EndWeldingBA).count();
    vdMergeOptEss_ms.push_back(timeOptEss);
#endif


    mpLocalMapper->Release();

    Verbose::PrintMess("MERGE:Completed!!!!!", Verbose::VERBOSITY_DEBUG);

    if(bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1)))
    {
        // Launch a new thread to perform Global Bundle Adjustment
        Verbose::PrintMess("Relaunch Global BA", Verbose::VERBOSITY_DEBUG);
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this, pMergeMap, mpCurrentKF->mnId);
    }

    mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
    mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

    pCurrentMap->IncreaseChangeIndex();
    pMergeMap->IncreaseChangeIndex();

    mpAtlas->RemoveBadMaps();

}

void LoopClosing::printReprojectionError(set<KeyFramePtr> &spLocalWindowKFs, KeyFramePtr mpCurrentKF, string &name)
{
    string path_imgs = "./test_Reproj/";
    for(KeyFramePtr pKFi : spLocalWindowKFs)
    {
        //cout << "KF " << pKFi->mnId << endl;
        cv::Mat img_i = cv::imread(pKFi->mNameFile, cv::IMREAD_UNCHANGED);
        //cout << "Image -> " << img_i.cols << ", " << img_i.rows << endl;
        cv::cvtColor(img_i, img_i, cv::COLOR_GRAY2BGR);
        //cout << "Change of color in the image " << endl;

        vector<MapPointPtr> vpMPs = pKFi->GetMapPointMatches();
        int num_points = 0;
        for(int j=0; j<vpMPs.size(); ++j)
        {
            MapPointPtr pMPij = vpMPs[j];
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            cv::KeyPoint point_img = pKFi->mvKeysUn[j];
            cv::Point2f reproj_p;
            float u, v;
            bool bIsInImage = pKFi->ProjectPointUnDistort(pMPij, reproj_p, u, v);
            if(bIsInImage){
                //cout << "Reproj in the image" << endl;
                cv::circle(img_i, point_img.pt, 1/*point_img.octave*/, cv::Scalar(0, 255, 0));
                cv::line(img_i, point_img.pt, reproj_p, cv::Scalar(0, 0, 255));
                num_points++;
            }
            else
            {
                //cout << "Reproj out of the image" << endl;
                cv::circle(img_i, point_img.pt, point_img.octave, cv::Scalar(0, 0, 255));
            }

        }
        //cout << "Image painted" << endl;
        string filename_img = path_imgs +  "KF" + to_string(mpCurrentKF->mnId) + "_" + to_string(pKFi->mnId) +  name + "points" + to_string(num_points) + ".png";
        cv::imwrite(filename_img, img_i);
    }

}


void LoopClosing::MergeLocal2()
{
    cout << "Merge detected!!!!" << endl;

    int numTemporalKFs = 11; //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    KeyFramePtr pNewChild;
    KeyFramePtr pNewParent;

    vector<KeyFramePtr> vpLocalCurrentWindowKFs;
    vector<KeyFramePtr> vpMergeConnectedKFs;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    // NonCorrectedSim3[mpCurrentKF]=mg2oLoopScw;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    cout << "Check Full Bundle Adjustment" << endl;
    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }


    cout << "Request Stop Local Mapping" << endl;
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }
    cout << "Local Map stopped" << endl;

    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    {
        float s_on = mSold_new.scale();
        Sophus::SE3f T_on(mSold_new.rotation().cast<float>(), mSold_new.translation().cast<float>());

        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

        //cout << "KFs before empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;
        mpLocalMapper->EmptyQueue();
        //cout << "KFs after empty: " << mpAtlas->GetCurrentMap()->KeyFramesInMap() << endl;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        //cout << "updating active map to merge reference" << endl;
        //cout << "curr merge KF id: " << mpCurrentKF->mnId << endl;
        //cout << "curr tracking KF id: " << mpTracker->GetLastKeyFrame()->mnId << endl;
        bool bScaleVel=false;
        if(s_on!=1)
            bScaleVel=true;
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on,s_on,bScaleVel);
        mpTracker->UpdateFrameIMU(s_on,mpCurrentKF->GetImuBias(),mpTracker->GetLastKeyFrame());

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    }

    const int numKFnew=pCurrentMap->KeyFramesInMap();

    if((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
       && !pCurrentMap->GetIniertialBA2()){
        // Map is not completly initialized
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::InertialOptimization(pCurrentMap,bg,ba);
        IMU::Bias b (ba[0],ba[1],ba[2],bg[0],bg[1],bg[2]);
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        mpTracker->UpdateFrameIMU(1.0f,b,mpTracker->GetLastKeyFrame());

        // Set map initialized
        pCurrentMap->SetIniertialBA2();
        pCurrentMap->SetIniertialBA1();
        pCurrentMap->SetImuInitialized();

    }


    cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    // Load KFs and MPs from merge map
    cout << "updating current map" << endl;
    {
        // Get Merge Map Mutex (This section stops tracking!!)
        unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map


        vector<KeyFramePtr> vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
        vector<MapPointPtr> vpMergeMapMPs = pMergeMap->GetAllMapPoints();
        vector<MapLinePtr> vpMergeMapMLs;
        if(mpTracker->IsLineTracking())
            vpMergeMapMLs = pMergeMap->GetAllMapLines();
        vector<MapObjectPtr> vpMergeMapMOs;
        if(mpTracker->IsObjectTracking())
            vpMergeMapMOs = pMergeMap->GetAllMapObjects(); 

        for(KeyFramePtr pKFi : vpMergeMapKFs)
        {
            if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            pKFi->UpdateMap(pCurrentMap);
            pCurrentMap->AddKeyFrame(pKFi);
            pMergeMap->EraseKeyFrame(pKFi);
        }

        for(MapPointPtr pMPi : vpMergeMapMPs)
        {
            if(!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                continue;

            pMPi->UpdateMap(pCurrentMap);
            pCurrentMap->AddMapPoint(pMPi);
            pMergeMap->EraseMapPoint(pMPi);
        }

        if(mpTracker->IsLineTracking())
        {
            for(MapLinePtr pMLi : vpMergeMapMLs)
            {
                if(!pMLi || pMLi->isBad() || pMLi->GetMap() != pMergeMap)
                    continue;

                pMLi->UpdateMap(pCurrentMap);
                pCurrentMap->AddMapLine(pMLi);
                pMergeMap->EraseMapLine(pMLi);
            }            
        }

        if(mpTracker->IsObjectTracking())
        {
            for(MapObjectPtr pMOi : vpMergeMapMOs)
            {
                if(!pMOi || pMOi->isBad() || pMOi->GetMap() != pMergeMap)
                    continue;

                pMOi->UpdateMap(pCurrentMap);
                pCurrentMap->AddMapObject(pMOi);
                pMergeMap->EraseMapObject(pMOi);
            }              
        }
        
        // Save non corrected poses (already merged maps)
        vector<KeyFramePtr> vpKFs = pCurrentMap->GetAllKeyFrames();
        for(KeyFramePtr pKFi : vpKFs)
        {
            Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
            NonCorrectedSim3[pKFi]=g2oSiw;
        }
    }

    //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    //cout << "end updating current map" << endl;

    // Critical zone
    //bool good = pCurrentMap->CheckEssentialGraph();
    /*if(!good)
        cout << "BAD ESSENTIAL GRAPH!!" << endl;*/

    //cout << "Update essential graph" << endl;
    // mpCurrentKF->UpdateConnections(); // to put at false mbFirstConnection
    pMergeMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF
    pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpMergeMatchedKF->ChangeParent(mpCurrentKF);
    while(pNewChild)
    {
        pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
        KeyFramePtr  pOldParent = pNewChild->GetParent();
        pNewChild->ChangeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild = pOldParent;

    }


    //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    //cout << "end update essential graph" << endl;

    /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 1!!" << endl;*/

    //cout << "Update relationship between KFs" << endl;

    vector<MapPointPtr> vpCheckFuseMapPoint;   // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
    vector<MapLinePtr> vpCheckFuseMapLine;     // MapLine vector from current map to allow to fuse duplicated lines with the old map (merge) 
    vector<MapObjectPtr> vpCheckFuseMapObject; // MapObject vector from current map to allow to fuse duplicated objects with the old map (merge)     
    vector<KeyFramePtr> vpCurrentConnectedKFs;

    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vector<KeyFramePtr> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mvpMergeConnectedKFs.size()>6)
        mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin()+6,mvpMergeConnectedKFs.end());
    /*mvpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);*/

    mpCurrentKF->UpdateConnections();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);
    /*vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);*/
    aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
    if (vpCurrentConnectedKFs.size()>6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin()+6,vpCurrentConnectedKFs.end());

    set<MapPointPtr> spMapPointMerge;
    set<MapLinePtr> spMapLineMerge;
    set<MapObjectPtr> spMapObjectMerge;    
    for(KeyFramePtr pKFi : mvpMergeConnectedKFs)
    {
        set<MapPointPtr> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
        if(mpTracker->IsLineTracking())
        {
            set<MapLinePtr> vpMLs = pKFi->GetMapLines();
            spMapLineMerge.insert(vpMLs.begin(),vpMLs.end());        
        }
        if(mpTracker->IsObjectTracking())
        {
            set<MapObjectPtr> vpMOs = pKFi->GetMapObjects();
            spMapObjectMerge.insert(vpMOs.begin(),vpMOs.end());        
        }        

        if(mpTracker->IsLineTracking())
        {
            // < TODO: Luigi improve this condition ?
            if(spMapPointMerge.size() + Tracking::sknLineTrackWeigth*spMapLineMerge.size()>1000)
                break;        
        }
        else
        {
            if(spMapPointMerge.size()>1000)
                break;
        }
    }

    /*cout << "vpCurrentConnectedKFs.size() " << vpCurrentConnectedKFs.size() << endl;
    cout << "mvpMergeConnectedKFs.size() " << mvpMergeConnectedKFs.size() << endl;
    cout << "spMapPointMerge.size() " << spMapPointMerge.size() << endl;*/


    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));
    if(mpTracker->IsLineTracking())
    {
        vpCheckFuseMapLine.reserve(spMapLineMerge.size());
        std::copy(spMapLineMerge.begin(), spMapLineMerge.end(), std::back_inserter(vpCheckFuseMapLine));
    }    
    if(mpTracker->IsObjectTracking())
    {
        vpCheckFuseMapObject.reserve(spMapObjectMerge.size());
        std::copy(spMapObjectMerge.begin(), spMapObjectMerge.end(), std::back_inserter(vpCheckFuseMapObject));
    }

    //cout << "Finished to update relationship between KFs" << endl;

    //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 2!!" << endl;*/

    //cout << "start SearchAndFuse" << endl;

    // TODO: Luigi add map objects management
    SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint, vpCheckFuseMapLine, vpCheckFuseMapObject);

    //cout << "end SearchAndFuse" << endl;

    //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 3!!" << endl;

    cout << "Init to update connections" << endl;*/


    for(KeyFramePtr pKFi : vpCurrentConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for(KeyFramePtr pKFi : mvpMergeConnectedKFs)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    //cout << "end update connections" << endl;

    //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

    /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 4!!" << endl;*/

    // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new to old map
    if (numKFnew<10){
        mpLocalMapper->Release();
        return;
    }

    /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 5!!" << endl;*/

    // Perform BA
    bool bStopFlag=false;
    KeyFramePtr pCurrKF = mpTracker->GetLastKeyFrame();
    //cout << "start MergeInertialBA" << endl;
    Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap,CorrectedSim3);
    //cout << "end MergeInertialBA" << endl;

    /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 6!!" << endl;*/

    // Release Local Mapping.
    mpLocalMapper->Release();


    return;
}

void LoopClosing::CheckObservations(set<KeyFramePtr> &spKFsMap1, set<KeyFramePtr> &spKFsMap2)
{
    cout << "----------------------" << endl;
    for(KeyFramePtr pKFi1 : spKFsMap1)
    {
        map<KeyFramePtr, int> mMatchedMP;
        set<MapPointPtr> spMPs = pKFi1->GetMapPoints();

        for(MapPointPtr pMPij : spMPs)
        {
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            map<KeyFramePtr, tuple<int,int>> mMPijObs = pMPij->GetObservations();
            for(KeyFramePtr pKFi2 : spKFsMap2)
            {
                if(mMPijObs.find(pKFi2) != mMPijObs.end())
                {
                    if(mMatchedMP.find(pKFi2) != mMatchedMP.end())
                    {
                        mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                    }
                    else
                    {
                        mMatchedMP[pKFi2] = 1;
                    }
                }
            }

        }

        if(mMatchedMP.size() == 0)
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId << " has not any matched MP with the other map" << endl;
        }
        else
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with " << mMatchedMP.size() << " KF from the other map" << endl;
            for(pair<KeyFramePtr, int> matchedKF : mMatchedMP)
            {
                cout << "   -KF: " << matchedKF.first->mnId << ", Number of matches: " << matchedKF.second << endl;
            }
        }
    }
    cout << "----------------------" << endl;
}

// TODO: Luigi add map objects management
void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPointPtr> &vpMapPoints, vector<MapLinePtr> &vpMapLines, vector<MapObjectPtr> &vpMapObjects)
{
    ORBmatcher matcher(0.8);
    std::shared_ptr<LineMatcher> pLineMatcher;    
    if(mpTracker->IsLineTracking())
    {    
        pLineMatcher.reset( new LineMatcher(0.8) );
    }

    int totalNumReplacedPoints = 0;
    int totalNumReplacedLines = 0;

    cout << "FUSE: Initially there are " << vpMapPoints.size() << " MPs" << endl;
    if(mpTracker->IsLineTracking())
    {        
        cout << "FUSE: Initially there are " << vpMapLines.size() << " MLs" << endl;
    }
    cout << "FUSE: Intially there are " << CorrectedPosesMap.size() << " KFs" << endl;
    
    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        int numReplacedPoints = 0;
        int numReplacedLines = 0;

        KeyFramePtr pKFi = mit->first;
        Map* pMap = pKFi->GetMap();

        g2o::Sim3 g2oScw = mit->second;
        Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

        vector<MapPointPtr> vpReplacePoints(vpMapPoints.size(),static_cast<MapPointPtr>(NULL));
        int numFused = matcher.Fuse(pKFi,Scw,vpMapPoints,4,vpReplacePoints);

        vector<MapLinePtr> vpReplaceLines(vpMapLines.size(),static_cast<MapLinePtr>(NULL));
        if(mpTracker->IsLineTracking())
        {
            pLineMatcher->Fuse(pKFi,Scw,vpMapLines,4,vpReplaceLines);
        }
        
        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPointPtr pRep = vpReplacePoints[i];
            if(pRep)
            {
                numReplacedPoints += 1;
                pRep->Replace(vpMapPoints[i]);

            }
        }

        if(mpTracker->IsLineTracking())
        {        
            const int nLL = vpMapLines.size();
            for(int i=0; i<nLL;i++)
            {
                MapLinePtr pRep = vpReplaceLines[i];
                if(pRep)
                {
                    numReplacedLines += 1;
                    pRep->Replace(vpMapLines[i]);
                }
            }
        }

        totalNumReplacedPoints += numReplacedPoints;
        
        totalNumReplacedLines += numReplacedLines;
    }
    
    std::string messageLineFused;
    if(mpTracker->IsLineTracking())
    {        
        messageLineFused = "; " + std::to_string(totalNumReplacedLines) + " MLs had been fused";
    }   
    cout << "FUSE: " << totalNumReplacedPoints << " MPs had been fused" << messageLineFused << endl; 
}

// TODO: Luigi add map objects management
void LoopClosing::SearchAndFuse(const vector<KeyFramePtr> &vConectedKFs, vector<MapPointPtr> &vpMapPoints, vector<MapLinePtr> &vpMapLines, vector<MapObjectPtr> &vpMapObjects)
{
    ORBmatcher matcher(0.8);
    std::shared_ptr<LineMatcher> pLineMatcher;    
    if(mpTracker->IsLineTracking())
    {    
        pLineMatcher.reset( new LineMatcher(0.8) );
    }    

    int total_replaced_points = 0;
    int total_replaced_lines = 0;    

    cout << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs" << endl;
    cout << "FUSE-POSE: Initially there are " << vpMapLines.size() << " MLs" << endl;    
    cout << "FUSE-POSE: Initially there are " << vConectedKFs.size() << " KFs" << endl;
    for(auto mit=vConectedKFs.begin(), mend=vConectedKFs.end(); mit!=mend;mit++)
    {
        int num_replaced_points = 0;
        int num_replaced_lines = 0;        
        
        KeyFramePtr pKF = (*mit);
        Map* pMap = pKF->GetMap();
        Sophus::SE3f Tcw = pKF->GetPose();
        Sophus::Sim3f Scw(Tcw.unit_quaternion(),Tcw.translation());
        Scw.setScale(1.f);
        /*std::cout << "These should be zeros: " <<
            Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
            Scw.translation() - Tcw.translation() << std::endl <<
            Scw.scale() - 1.f << std::endl;*/

        vector<MapPointPtr> vpReplacePoints(vpMapPoints.size(),static_cast<MapPointPtr>(NULL));
        matcher.Fuse(pKF,Scw,vpMapPoints,4,vpReplacePoints);
        
        vector<MapLinePtr> vpReplaceLines(vpMapLines.size(),static_cast<MapLinePtr>(NULL));
        if(mpTracker->IsLineTracking())
        {
            pLineMatcher->Fuse(pKF,Scw,vpMapLines,4,vpReplaceLines);
        }

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPointPtr pRep = vpReplacePoints[i];
            if(pRep)
            {
                num_replaced_points += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }
        if(mpTracker->IsLineTracking())
        {        
            const int nLL = vpMapLines.size();
            for(int i=0; i<nLL;i++)
            {
                MapLinePtr pRep = vpReplaceLines[i];
                if(pRep)
                {
                    num_replaced_lines += 1;
                    pRep->Replace(vpMapLines[i]);
                }
            }
        }        
        cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaced_points << " MPs fused" << endl;
        total_replaced_points += num_replaced_points;
        total_replaced_lines += num_replaced_lines;
    }
    
    cout << "FUSE-POSE: " << total_replaced_points << " MPs had been fused" << endl;
    if(mpTracker->IsLineTracking())    
        cout << "FUSE-POSE: " << total_replaced_lines << " MLs had been fused" << endl;    
}



void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::RequestResetActiveMap(Map *pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMapRequested = true;
        mpMapToReset = pMap;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;  //TODO old variable, it is not use in the new algorithm
        mbResetRequested=false;
        mbResetActiveMapRequested = false;
    }
    else if(mbResetActiveMapRequested)
    {

        for (list<KeyFramePtr>::const_iterator it=mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();)
        {
            KeyFramePtr pKFi = *it;
            if(pKFi->GetMap() == mpMapToReset)
            {
                it = mlpLoopKeyFrameQueue.erase(it);
            }
            else
                ++it;
        }

        mLastLoopKFid=mpAtlas->GetLastInitKFid(); //TODO old variable, it is not use in the new algorithm
        mbResetActiveMapRequested=false;

    }
}

void LoopClosing::RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF)
{
    Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFGBA = std::chrono::steady_clock::now();

    nFGBA_exec += 1;

    vnGBAKFs.push_back(pActiveMap->GetAllKeyFrames().size());
    vnGBAMPs.push_back(pActiveMap->GetAllMapPoints().size());
#endif

    const bool bImuInit = pActiveMap->isImuInitialized();

    if(!bImuInit)
        Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,&mbStopGBA,nLoopKF,false);
    else
        Optimizer::FullInertialBA(pActiveMap,7,false,nLoopKF,&mbStopGBA);

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndGBA = std::chrono::steady_clock::now();

    double timeGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndGBA - time_StartFGBA).count();
    vdGBA_ms.push_back(timeGBA);

    if(mbStopGBA)
    {
        nFGBA_abort += 1;
    }
#endif

    int idx =  mnFullBAIdx;
    // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!bImuInit && pActiveMap->isImuInitialized())
            return;

        if(!mbStopGBA)
        {
            Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
            Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
            // cout << "LC: Update Map Mutex adquired" << endl;

#if ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE              
            // Reset the visited flag of all the keyframes          
            vector< KeyFramePtr > vpKeyFrames = pActiveMap->GetAllKeyFrames();
            for (int i = 0; i < (int)vpKeyFrames.size(); i++)
                vpKeyFrames[i]->mbVisited = false;
#endif

            //pActiveMap->PrintEssentialGraph();

            // Correct keyframes starting at map first keyframe
            list<KeyFramePtr> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(),pActiveMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFramePtr pKF = lpKFtoCheck.front();
#if ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE                     
                pKF->mbVisited = true;                
#endif         
                const set<KeyFramePtr> sChilds = pKF->GetChilds();
                //cout << "---Updating KF " << pKF->mnId << " with " << sChilds.size() << " childs" << endl;
                //cout << " KF mnBAGlobalForKF: " << pKF->mnBAGlobalForKF << endl;
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                //cout << "Twc: " << Twc << endl;
                //cout << "GBA: Correct KeyFrames" << endl;
                for(set<KeyFramePtr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFramePtr pChild = *sit;
#if ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE                        
                    if (pChild->mbVisited) continue;                    
#endif
                    if(!pChild || pChild->isBad())
                        continue;

                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        //cout << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                        //cout << " child id: " << pChild->mnId << endl;
                        Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                        //cout << "Child pose: " << Tchildc << endl;
                        //cout << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                        pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;

                        Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                        if(pChild->isVelocitySet()){
                            pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                        }
                        else
                            Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);


                        //cout << "Child bias: " << pChild->GetImuBias() << endl;
                        pChild->mBiasGBA = pChild->GetImuBias();


                        pChild->mnBAGlobalForKF = nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                //cout << "-------Update pose" << endl;
                pKF->mTcwBefGBA = pKF->GetPose();
                //cout << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                pKF->SetPose(pKF->mTcwGBA);
                /*cv::Mat Tco_cn = pKF->mTcwBefGBA * pKF->mTcwGBA.inv();
                cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
                double dist = cv::norm(trasl);
                cout << "GBA: KF " << pKF->mnId << " had been moved " << dist << " meters" << endl;
                double desvX = 0;
                double desvY = 0;
                double desvZ = 0;
                if(pKF->mbHasHessian)
                {
                    cv::Mat hessianInv = pKF->mHessianPose.inv();

                    double covX = hessianInv.at<double>(3,3);
                    desvX = std::sqrt(covX);
                    double covY = hessianInv.at<double>(4,4);
                    desvY = std::sqrt(covY);
                    double covZ = hessianInv.at<double>(5,5);
                    desvZ = std::sqrt(covZ);
                    pKF->mbHasHessian = false;
                }
                if(dist > 1)
                {
                    cout << "--To much distance correction: It has " << pKF->GetConnectedKeyFrames().size() << " connected KFs" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(80).size() << " connected KF with 80 common matches or more" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(50).size() << " connected KF with 50 common matches or more" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(20).size() << " connected KF with 20 common matches or more" << endl;

                    cout << "--STD in meters(x, y, z): " << desvX << ", " << desvY << ", " << desvZ << endl;


                    string strNameFile = pKF->mNameFile;
                    cv::Mat imLeft = cv::imread(strNameFile, cv::IMREAD_UNCHANGED);

                    cv::cvtColor(imLeft, imLeft, cv::COLOR_GRAY2BGR);

                    vector<MapPointPtr> vpMapPointsKF = pKF->GetMapPointMatches();
                    int num_MPs = 0;
                    for(int i=0; i<vpMapPointsKF.size(); ++i)
                    {
                        if(!vpMapPointsKF[i] || vpMapPointsKF[i]->isBad())
                        {
                            continue;
                        }
                        num_MPs += 1;
                        string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                        cv::circle(imLeft, pKF->mvKeys[i].pt, 2, cv::Scalar(0, 255, 0));
                        cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                    }
                    cout << "--It has " << num_MPs << " MPs matched in the map" << endl;

                    string namefile = "./test_GBA/GBA_" + to_string(nLoopKF) + "_KF" + to_string(pKF->mnId) +"_D" + to_string(dist) +".png";
                    cv::imwrite(namefile, imLeft);
                }*/


                if(pKF->bImu)
                {
                    //cout << "-------Update inertial values" << endl;
                    pKF->mVwbBefGBA = pKF->GetVelocity();
                    //if (pKF->mVwbGBA.empty())
                    //    Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                    //assert(!pKF->mVwbGBA.empty());
                    pKF->SetVelocity(pKF->mVwbGBA);
                    pKF->SetNewBias(pKF->mBiasGBA);                    
                }

                lpKFtoCheck.pop_front();
            }

            //cout << "GBA: Correct MapPoints" << endl;
            // Correct MapPoints
            const vector<MapPointPtr> vpMPs = pActiveMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPointPtr pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFramePtr pRefKF = pMP->GetReferenceKeyFrame();
                    if(!pRefKF) continue;
                    
                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    /*if(pRefKF->mTcwBefGBA.empty())
                        continue;*/

                    // Map to non-corrected camera
                    // cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    // cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                    // Backproject using corrected camera
                    pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                }
            }

	    // Correct MapLines
            if(mpTracker->IsLineTracking())
            {
                const vector<MapLinePtr> vpMLs = pActiveMap->GetAllMapLines();

                for(size_t i=0; i<vpMLs.size(); i++)
                {
                    MapLinePtr pML = vpMLs[i];

                    if(pML->isBad())
                        continue;

                    if(pML->mnBAGlobalForKF==nLoopKF)
                    {
                        // If optimized by Global BA, just update
                        pML->SetWorldEndPoints(pML->mPosStartGBA, pML->mPosEndGBA);
                        pML->UpdateNormalAndDepth();
                    }
                    else
                    {
                        // Update according to the correction of its reference keyframe
                        KeyFramePtr pRefKF = pML->GetReferenceKeyFrame();
                        if(!pRefKF) continue; 

                        if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                            continue;

                        // Map to non-corrected camera
                        //cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                        //cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                        
                        Eigen::Vector3f XSw, XEw;
                        pML->GetWorldEndPoints(XSw, XEw);  
                        Eigen::Vector3f XSc = pRefKF->mTcwBefGBA * XSw;
                        Eigen::Vector3f XEc = pRefKF->mTcwBefGBA * XEw;

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
                const vector<MapObjectPtr > vpMObjs = pActiveMap->GetAllMapObjects();

                for(size_t i=0; i<vpMObjs.size(); i++)
                {
                    MapObjectPtr pMObj = vpMObjs[i];

                    if(pMObj->isBad())
                        continue;

                    if(pMObj->mnBAGlobalForKF==nLoopKF)
                    {
                        // If optimized by Global BA, just update
                        pMObj->SetSim3Pose(pMObj->mSowGBA);
                        //pMObj->UpdateNormalAndDepth();
                    }
                    else
                    {
                        // Update according to the correction of its reference keyframe
                        KeyFramePtr pRefKF = pMObj->GetReferenceKeyFrame();

                        if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                            continue;

                        // Map to non-corrected camera
                        Eigen::Matrix3f Rcw = pRefKF->mTcwBefGBA.rotationMatrix();
                        Eigen::Vector3f tcw = pRefKF->mTcwBefGBA.translation();
                        
                        Eigen::Matrix3f Rwo; 
                        Eigen::Vector3f two; 
                        double scale = pMObj->GetScale();
                        Rwo = pMObj->GetInverseRotation();
                        two = pMObj->GetInverseTranslation();
                        const Eigen::Matrix3f Rco = Rcw*Rwo;
                        const Eigen::Vector3f tco = Rcw*two+tcw;  

                        // Backproject using corrected camera
                        const Sophus::SE3f Twc = pRefKF->GetPoseInverse();
                        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
                        Eigen::Vector3f twc = Twc.translation();
                     
                        const Eigen::Matrix3f RwoNew = Rwc*Rco;
                        const Eigen::Vector3f twoNew = Rwc*tco+twc;
                        
                        pMObj->SetSim3InversePose(RwoNew, twoNew, scale);
                    }
                }    
            }

            pActiveMap->InformNewBigChange();
            pActiveMap->IncreaseChangeIndex();

            // TODO Check this update
            // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());

            mpLocalMapper->Release();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndUpdateMap = std::chrono::steady_clock::now();

            double timeUpdateMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateMap - time_EndGBA).count();
            vdUpdateMap_ms.push_back(timeUpdateMap);

            double timeFGBA = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndUpdateMap - time_StartFGBA).count();
            vdFGBATotal_ms.push_back(timeFGBA);
#endif
            Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
    
    mSignalGlobalBundleAdjustmentFinished.emit();
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    // cout << "LC: Finish requested" << endl;
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} // namespace PLVS2
