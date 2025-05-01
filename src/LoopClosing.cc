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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"
#include "LineMatcher.h"
#include "MapObject.h"

#include<mutex>
#include<thread>

#define USE_LINES_FOR_VOTING_LOOP_CLOSURE 1

#define ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE 1

#define VERBOSE 1

namespace PLVS
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;
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
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

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

bool LoopClosing::DetectLoop()
{
    bool bLoopClosingEnabled = true;
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    {
        unique_lock< mutex > lock(mpMap->mMutexLoopClosing);
        bLoopClosingEnabled = mpMap->mnEnableLoopClosing;
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFramePtr> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFramePtr pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFramePtr> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty() || !bLoopClosingEnabled)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFramePtr pCandidateKF = vpCandidateKFs[i];

        set<KeyFramePtr> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFramePtr> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFramePtr>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPointPtr> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);
    

    /// < TODO: add line matching here and use it in the Sim3Solver  ?   
    mvpCurrentMatchedLines = vector<MapLinePtr>(mpCurrentKF->Nlines,static_cast<MapLinePtr>(NULL));
    

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches
    
    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFramePtr pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is successful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFramePtr pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPointPtr> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPointPtr>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                int num_new_matches = matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);
                std::cout << "[LoopClosing SearchBySim3] Number of new matches: " << num_new_matches << std::endl;

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);
                std::cout << "[LoopClosing OptimizeSim3] Number optimized inliers: " << nInliers << std::endl;

                // If optimization is successful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    
    // Retrieve MapPoints and MapLines seen in Loop Keyframe and neighbors
    vector<KeyFramePtr> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    
    mvpLoopMapPoints.clear();
    mvpLoopMapLines.clear();
    
    for(vector<KeyFramePtr>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFramePtr pKF = *vit;
        vector<MapPointPtr> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPointPtr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
        
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE
        if(mpTracker->IsLineTracking())
        {
            vector<MapLinePtr> vpMapLines = pKF->GetMapLineMatches();
            for(size_t i=0, iend=vpMapLines.size(); i<iend; i++)
            {
                MapLinePtr pML = vpMapLines[i];
                if(pML)
                {
                    if(!pML->isBad() && pML->mnLoopPointForKF!=mpCurrentKF->mnId)
                    {
                        mvpLoopMapLines.push_back(pML);
                        pML->mnLoopPointForKF=mpCurrentKF->mnId;
                    }
                }
            }            
        }
#endif 
        
// TODO: add objects         
        
    }

    // Find more matches projecting with the computed Sim3
    int nmatches = matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);
    std::cout << "[LoopClosing SearchByProjection] Number of new matches: " << nmatches << std::endl;
    
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE    
    if(mpTracker->IsLineTracking())
    {
        LineMatcher lineMatcher(0.75);
        lineMatcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapLines, mvpCurrentMatchedLines, 3.0, 1.5); // 1.0, 1.0 
    }
#endif
    
    // If enough matches accept Loop
    int nTotalPointMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalPointMatches++;
    }
    
    int nTotalLineMatches = 0; 
#if USE_LINES_FOR_VOTING_LOOP_CLOSURE      
    if(mpTracker->IsLineTracking())
    {
        for(size_t i=0; i<mvpCurrentMatchedLines.size(); i++)
        {
            if(mvpCurrentMatchedLines[i])
                nTotalLineMatches++;
        }        
    }
#endif
        

    if( (nTotalPointMatches + Tracking::sknLineTrackWeigth*nTotalLineMatches ) >=40 )
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

void LoopClosing::CorrectLoop()
{
    cout << "**************" << endl;
    cout << "Loop detected!" << endl;
    cout << "**************" << endl;

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
    mpCurrentKF->UpdateConnections();
    
#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - updated connections" << endl; 
#endif

    // Retrieve keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - updating map" << endl; 
#endif    
    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFramePtr>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFramePtr pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints and MapLines observed by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

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
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
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
                    cv::Mat P3DSw, P3DEw;
                    pMLi->GetWorldEndPoints(P3DSw, P3DEw);                      
                    
                    Eigen::Matrix<double,3,1> eigP3DSw = Converter::toVector3d(P3DSw);
                    Eigen::Matrix<double,3,1> eigP3DEw = Converter::toVector3d(P3DEw);
                    
                    Eigen::Matrix<double,3,1> eigCorrectedP3DSw = g2oCorrectedSwi.map(g2oSiw.map(eigP3DSw));
                    Eigen::Matrix<double,3,1> eigCorrectedP3DEw = g2oCorrectedSwi.map(g2oSiw.map(eigP3DEw));

                    cv::Mat cvCorrectedP3DSw = Converter::toCvMat(eigCorrectedP3DSw);
                    cv::Mat cvCorrectedP3DEw = Converter::toCvMat(eigCorrectedP3DEw);
                    
                    pMLi->SetWorldEndPoints(cvCorrectedP3DSw, cvCorrectedP3DEw);
                    pMLi->UpdateNormalAndDepth();
                    
                    pMLi->mnCorrectedByKF = mpCurrentKF->mnId;
                    pMLi->mnCorrectedReference = pKFi->mnId;
                }                
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPointPtr pLoopMP = mvpCurrentMatchedPoints[i];
                MapPointPtr pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    if( pLoopMP->AddObservation(mpCurrentKF,i) )
                        mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }
            
        
        if(mpTracker->IsLineTracking())
        {
            // Update matched map lines and replace if duplicated
            for(size_t i=0; i<mvpCurrentMatchedLines.size(); i++)
            {
                if(mvpCurrentMatchedLines[i])
                {
                    MapLinePtr pLoopML = mvpCurrentMatchedLines[i];
                    MapLinePtr pCurML = mpCurrentKF->GetMapLine(i);
                    if(pCurML)
                        pCurML->Replace(pLoopML);
                    else
                    {
                        if( pLoopML->AddObservation(mpCurrentKF,i) )
                            mpCurrentKF->AddMapLine(pLoopML,i);                        
                        pLoopML->ComputeDistinctiveDescriptors();
                    }
                }                
            }
        }
            
    }

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - searching and fusing features " << endl; 
#endif      
    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - updating covisibility graph" << endl; 
#endif  
    
    // After the MapPoint and MapLine fusion, new links in the covisibility graph will appear attaching both sides of the loop
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
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    // ?Should we move this after full global BA?
    mpMap->InformNewBigChange();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - running bundle adjustment " << endl; 
#endif     
    
    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

#if VERBOSE     
    cout << "LoopClosing::CorrectLoop() - bundle adjustment done - releasing local mapper" << endl; 
#endif       
    
    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
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
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

#if VERBOSE     
    cout << "LoopClosing::StartGlobalBundleAdjustment() - bundle adjustment done - releasing local mapper" << endl; 
#endif       
    
    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    //mLastLoopKFid = mpCurrentKF->mnId;   
    
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);
    std::shared_ptr<LineMatcher> pLineMatcher;    
    if(mpTracker->IsLineTracking())
    {    
        pLineMatcher.reset( new LineMatcher(0.8) );
    }

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFramePtr pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPointPtr> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPointPtr>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        vector<MapLinePtr> vpReplaceLines(mvpLoopMapLines.size(),static_cast<MapLinePtr>(NULL));
        if(mpTracker->IsLineTracking())
        {
            pLineMatcher->Fuse(pKF,cvScw,mvpLoopMapLines,4,vpReplaceLines);
        }
        
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPointPtr pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
        
        if(mpTracker->IsLineTracking())
        {        
            const int nLL = mvpLoopMapLines.size();
            for(int i=0; i<nLL;i++)
            {
                MapLinePtr pRep = vpReplaceLines[i];
                if(pRep)
                {
                    pRep->Replace(mvpLoopMapLines[i]);
                }
            }
        }
        
    }
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

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints, MapLines and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

#if ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE              
            // Reset the visited flag of all the keyframes          
            vector< KeyFramePtr > vpKeyFrames = mpMap->GetAllKeyFrames();
            for (int i = 0; i < (int)vpKeyFrames.size(); i++)
                vpKeyFrames[i]->mbVisited = false;
#endif            

            // Correct keyframes starting at map first keyframe
            list<KeyFramePtr> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFramePtr pKF = lpKFtoCheck.front();
#if ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE                     
                pKF->mbVisited = true;                
#endif                
                const set<KeyFramePtr> sChilds = pKF->GetChilds();
                const cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFramePtr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFramePtr pChild = *sit;
#if ENABLE_CHANGES_FOR_INFINITE_LOOP_ISSUE                        
                    if (pChild->mbVisited) continue;                    
#endif
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPointPtr> vpMPs = mpMap->GetAllMapPoints();

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

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }       
            
            // Correct MapLines
            if(mpTracker->IsLineTracking())
            {
                const vector<MapLinePtr> vpMLs = mpMap->GetAllMapLines();

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

                        if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                            continue;

                        // Map to non-corrected camera
                        cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                        cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                        
                        cv::Mat XSw, XEw;
                        pML->GetWorldEndPoints(XSw, XEw);  
                        cv::Mat XSc = Rcw*XSw+tcw;
                        cv::Mat XEc = Rcw*XEw+tcw;

                        // Backproject using corrected camera
                        cv::Mat Twc = pRefKF->GetPoseInverse();
                        cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                        cv::Mat twc = Twc.rowRange(0,3).col(3);

                        pML->SetWorldEndPoints(Rwc*XSc+twc, Rwc*XEc+twc);                        
                        pML->UpdateNormalAndDepth();
                    }
                }    
            }
            
            // Correct MapObjects
            if(mpTracker->IsObjectTracking())
            {
                const vector<MapObjectPtr > vpMObjs = mpMap->GetAllMapObjects();

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
                        cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                        cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                        
                        //cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;
                        
                        cv::Mat Rwo, two; 
                        double scale = pMObj->GetScale();
                        Rwo = pMObj->GetInverseRotation();
                        two = pMObj->GetInverseTranslation();
                        const cv::Mat Rco = Rcw*Rwo;
                        const cv::Mat tco = Rcw*two+tcw;  

                        // Backproject using corrected camera
                        cv::Mat Twc = pRefKF->GetPoseInverse();
                        cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                        cv::Mat twc = Twc.rowRange(0,3).col(3);

                        //pMP->SetWorldPos(Rwc*Xc+twc);                          
                        const cv::Mat RwoNew = Rwc*Rco;
                        const cv::Mat twoNew = Rwc*tco+twc;
                        
                        pMObj->SetSim3InversePose(RwoNew, twoNew, scale);
                        //pMObj->UpdateNormalAndDepth();
                    }
                }    
            }            
            

            mpMap->InformNewBigChange();
            
            //mSignalGlobalBundleAdjustmentFinished.emit();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
    
    mSignalGlobalBundleAdjustmentFinished.emit();
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
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


} //namespace PLVS
