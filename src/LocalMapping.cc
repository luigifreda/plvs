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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "MapLine.h"
#include "LineMatcher.h"

#include<mutex>

#define TRIANGULATE_NEW_LINES 1
#define ASSIGN_VIRTUAL_DISPARITY_WHEN_TRIANGULATING_LINES 1

namespace PLVS
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
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
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();
            
            // Check recent MapLines
            MapLineCulling();

            // Triangulate new MapPoints
            CreateNewMapFeatures();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point and line duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // Check redundant local Keyframes
                KeyFrameCulling();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
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
//                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
//                {
//                    pMP->AddObservation(mpCurrentKeyFrame, i);
                if(pMP->AddObservation(mpCurrentKeyFrame, i))
                {
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
//                    if(!pML->IsInKeyFrame(mpCurrentKeyFrame))
//                    {
//                        pML->AddObservation(mpCurrentKeyFrame, i);
                    if(pML->AddObservation(mpCurrentKeyFrame, i))
                    {
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
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
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
            lit++;
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
    if(mbMonocular)
        nn=20;
    const vector<KeyFramePtr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);
    std::shared_ptr<LineMatcher> pLineMatcher;
    if(mpTracker->IsLineTracking())
    {
        pLineMatcher.reset( new LineMatcher(0.6,false) );
    }

    const cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    const cv::Mat Rwc1 = Rcw1.t();
    const cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    const cv::Mat twc1 = -Rwc1*tcw1;
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    const cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;
    int nnewPoints=0;
    
#if TRIANGULATE_NEW_LINES       
    const cv::Mat &K1 = mpCurrentKeyFrame->mK;
    const float linesRatioFactor = 1.5f*mpCurrentKeyFrame->mfLineScaleFactor;    
    int nnewLines=0;    
    size_t nlineTotMatches = 0;
#endif 

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFramePtr pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

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

        // Compute Fundamental Matrix
        //cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        cv::Mat F12;
        cv::Mat H21, e2;

        if(!mpTracker->IsLineTracking())        
        {
            F12 = ComputeF12(mpCurrentKeyFrame,pKF2); 
        }
        else 
        {
            ComputeF12_H21(mpCurrentKeyFrame, pKF2, F12, H21, e2);   
        }  

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        const cv::Mat Rcw2 = pKF2->GetRotation();
        const cv::Mat Rwc2 = Rcw2.t();
        const cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each point match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1; // +1 since in case we do not have stereo we have cosParallaxRays < cosParallaxStereo at line 431
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
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
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
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
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is successful
            //MapPointPtr pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);
            MapPointPtr pMP = MapPointNewPtr(x3D,mpCurrentKeyFrame,mpMap);

            if (pMP->AddObservation(mpCurrentKeyFrame,idx1))            
                mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            
            if( pMP->AddObservation(pKF2,idx2))
                pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnewPoints++;
        }
        
#if TRIANGULATE_NEW_LINES         
        if(mpTracker->IsLineTracking())
        {
            const cv::Mat &K2 = pKF2->mK;
            const float minZ = pKF2->mb; // baseline in meters 
            const float maxZ = Tracking::skLineStereoMaxDist;   
            //const float maxZ = std::min(pKF2->mbf, Tracking::skLineStereoMaxDist);
                
            // Search line matches 
            std::vector<pair<size_t,size_t> > vMatchedLineIndices;
            pLineMatcher->SearchForTriangulation(mpCurrentKeyFrame,pKF2,vMatchedLineIndices,false);
            
            // Triangulate each line match
            const size_t nlineMatches = vMatchedLineIndices.size();
            nlineTotMatches += nlineMatches;
            //std::cout << "lines triangulation, matched " << nlineMatches << " lines " << std::endl; 
            for(size_t ikl=0; ikl<nlineMatches; ikl++)
            {
                const int &idx1 = vMatchedLineIndices[ikl].first;
                const int &idx2 = vMatchedLineIndices[ikl].second;              
            
                const cv::line_descriptor_c::KeyLine& kl1 = mpCurrentKeyFrame->mvKeyLinesUn[idx1]; 
                const float sigma1 = sqrt( mpCurrentKeyFrame->mvLineLevelSigma2[kl1.octave] );  
                const bool bStereo1 = (mpCurrentKeyFrame->mvuRightLineStart[idx1]>=0) && (mpCurrentKeyFrame->mvuRightLineEnd[idx1]>=0);                
                
                const cv::line_descriptor_c::KeyLine& kl2 = pKF2->mvKeyLinesUn[idx2]; 
                const float sigma2 = sqrt( pKF2->mvLineLevelSigma2[kl2.octave] );       
                const bool bStereo2 = (pKF2->mvuRightLineStart[idx2]>=0) && (pKF2->mvuRightLineEnd[idx2]>=0);  

                const cv::Mat p1 = (cv::Mat_<float>(3,1) << kl1.startPointX , kl1.startPointY, 1.0);
                const cv::Mat q1 = (cv::Mat_<float>(3,1) << kl1.endPointX ,   kl1.endPointY, 1.0); 
                const cv::Mat m1 = 0.5*(p1+q1);
                cv::Mat l1 = p1.cross(q1);   
                const float l1Norm = sqrt( Utils::Pow2(l1.at<float>(0)) + Utils::Pow2(l1.at<float>(1)) );
                l1 = l1/l1Norm; // in this way we have l1 = (nx, ny, -d) with (nx^2 + ny^2) = 1
                               
                
                const cv::Mat p2 = (cv::Mat_<float>(3,1) << kl2.startPointX , kl2.startPointY, 1.0);
                const cv::Mat q2 = (cv::Mat_<float>(3,1) << kl2.endPointX ,   kl2.endPointY, 1.0);           
                const cv::Mat m2 = 0.5*(p2+q2);
                cv::Mat l2 = p2.cross(q2); 
                const float l2Norm = sqrt( Utils::Pow2(l2.at<float>(0)) + Utils::Pow2(l2.at<float>(1)) );
                l2 = l2/l2Norm; // in this way we have l2 = (nx, ny, -d) with (nx^2 + ny^2) = 1             
                
                // Check if we can triangulate, i.e. check if the normals of the two planes corresponding to lines are not parallel
                bool bCanTriangulateLines = true;
                cv::Mat n1 = K1.t()*l1; n1 /= cv::norm(n1);
                cv::Mat n2 = K2.t()*l2; n2 /= cv::norm(n2);

                cv::Mat n1w = Rwc1*n1;
                cv::Mat n2w = Rwc2*n2;
                const float normalsDotProduct= fabs( n1w.dot(n2w) );
                const float sigma = std::max( sigma1, sigma2 );
                const float dotProductThreshold = 0.005 * sigma; //Frame::kLineNormalsDotProdThreshold * sigma; // this is a percentage over unitary modulus ( we modulate threshold by sigma)
                //const float dotProductThreshold = Frame::kLineNormalsDotProdThreshold * sigma; // this is a percentage over unitary modulus ( we modulate threshold by sigma)
                if( fabs( normalsDotProduct - 1.f ) < dotProductThreshold) 
                {
                    bCanTriangulateLines = false; // normals are almost parallel => cannot triangulate lines
                    //std::cout << "  -cannot triangulate n1: " << n1w << ", n2: " << n2w << std::endl;                     
                }
                
                // Check parallax between rays backprojecting the middle points 
                cv::Mat xm1 = (cv::Mat_<float>(3,1) << (m1.at<float>(0)-cx1)*invfx1, (m1.at<float>(1)-cy1)*invfy1, 1.0);
                cv::Mat xm2 = (cv::Mat_<float>(3,1) << (m2.at<float>(0)-cx2)*invfx2, (m2.at<float>(1)-cy2)*invfy2, 1.0);

                cv::Mat ray1 = Rwc1*xm1;
                cv::Mat ray2 = Rwc2*xm2;
                const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));                

                float cosParallaxStereo = cosParallaxRays+1; // +1 since in case we do not have stereo we have cosParallaxRays<cosParallaxStereo at line 413
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;
                
                if(bStereo1)
                {
                    const float depthM1 = 0.5* ( mpCurrentKeyFrame->mvDepthLineStart[idx1] + mpCurrentKeyFrame->mvDepthLineEnd[idx1] ); // depth middle point left
                    cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,depthM1));
                }
                else if(bStereo2)
                {
                    const float depthM2 = 0.5* ( pKF2->mvDepthLineStart[idx2] + pKF2->mvDepthLineEnd[idx2] ); // depth middle point right                   
                    cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,depthM2));
                }

                cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

                cv::Mat x3DS, x3DE;
                bool bLineTriangulatedByIntersection = false;
                if( bCanTriangulateLines && (cosParallaxRays<cosParallaxStereo) && (cosParallaxRays>0) && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
                {
                    // compute the intersections of rays backprojected from camera 1 with the 3D plane corresponding to l2  
                    // (check PLVS report)                                        
                    const float num = -l2.dot(e2);
                    const float den1 = (l2.dot(H21*p1));
                    const float den2 = (l2.dot(H21*q1));
                    
                    if( ( fabs(den1) < 0.001 ) || (fabs(den2) < 0.001) ) continue; 
                        
                    const float depthP1 = num/den1;
                    const float depthQ1 = num/den2;

                    if( (depthP1 >= minZ ) && (depthQ1 >= minZ) && (depthP1 <= maxZ) && (depthQ1 <= maxZ) )
                    {
                        cv::Mat x3DSc = (cv::Mat_<float>(3,1) << (p1.at<float>(0)-cx1)*invfx1*depthP1, (p1.at<float>(1)-cy1)*invfy1*depthP1, depthP1 ); // depthP1 * K1.inv()*p1;
                        x3DS = Rwc1*x3DSc + twc1;
                                                
                        cv::Mat x3DEc = (cv::Mat_<float>(3,1) << (q1.at<float>(0)-cx1)*invfx1*depthQ1, (q1.at<float>(1)-cy1)*invfy1*depthQ1, depthQ1 ); // depthQ1 * K1.inv()*q1;  
                        x3DE = Rwc1*x3DEc + twc1;

                        cv::Mat camRay = x3DSc/cv::norm(x3DSc);
                        cv::Mat lineES = x3DSc-x3DEc;  
                        const double lineLength = cv::norm(lineES);
                        if(lineLength >= Frame::skMinLineLength3D)
                        {        
                            lineES /= lineLength;
                            const float cosViewAngle = fabs((float)camRay.dot(lineES));
                            if(cosViewAngle<=Frame::kCosViewZAngleMax)
                            {                    
            #if ASSIGN_VIRTUAL_DISPARITY_WHEN_TRIANGULATING_LINES
                                if(!bStereo1)
                                {
                                    // assign depth and (virtual) disparity to left line end points 
                                    mpCurrentKeyFrame->mvDepthLineStart[idx1] = depthP1;
                                    const double disparity_p1 = mpCurrentKeyFrame->mbf/depthP1;                                
                                    mpCurrentKeyFrame->mvuRightLineStart[idx1] =  p1.at<float>(0) - disparity_p1;

                                    mpCurrentKeyFrame->mvDepthLineEnd[idx1] = depthQ1;
                                    const double disparity_q1 = mpCurrentKeyFrame->mbf/depthQ1;                         
                                    mpCurrentKeyFrame->mvuRightLineEnd[idx1] = q1.at<float>(0) - disparity_q1;
                                }
            #endif                         
                                bLineTriangulatedByIntersection = true; 
                                //std::cout << "triangulated line : " << x3DS << " , " << x3DE << std::endl; 
                            }
                        }
                    }                 
                }

                if(!bLineTriangulatedByIntersection)
                {
                    if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
                    {
                        //x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
                        if(!mpCurrentKeyFrame->UnprojectStereoLine(idx1, x3DS, x3DE)) continue;   
                        //std::cout << "unprojected1 line : " << x3DS << " , " << x3DE << std::endl; 
                    }
                    else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
                    {
                        //x3D = pKF2->UnprojectStereo(idx2);
                        if(!pKF2->UnprojectStereoLine(idx2, x3DS, x3DE)) continue; 
                        //std::cout << "unprojected2 line : " << x3DS << " , " << x3DE << std::endl;                         
                    }
                    else
                    {
                        continue; //No stereo and very low parallax
                    }


                    // N.B: if line is triangulated, the following block of checks are automatically satisfied by construction 
                    //(given the backprojection of the points p2 q2 from frame 2 on the plane corresponding to l1)

                    cv::Mat x3DSt = x3DS.t();
                    cv::Mat x3DEt = x3DE.t();

                    //Check triangulation in front of cameras
                    float sz1 = Rcw1.row(2).dot(x3DSt)+tcw1.at<float>(2);
                    if(sz1<=0)
                        continue;
                    float ez1 = Rcw1.row(2).dot(x3DEt)+tcw1.at<float>(2);
                    if(ez1<=0)
                        continue;                

                    float sz2 = Rcw2.row(2).dot(x3DSt)+tcw2.at<float>(2);
                    if(sz2<=0)
                        continue;
                    float ez2 = Rcw2.row(2).dot(x3DEt)+tcw2.at<float>(2);
                    if(ez2<=0)
                        continue;                

                    //Check reprojection error in first keyframe
                    const float &sigmaSquare1 = mpCurrentKeyFrame->mvLineLevelSigma2[kl1.octave];

                    const float sx1 = Rcw1.row(0).dot(x3DSt)+tcw1.at<float>(0);
                    const float sy1 = Rcw1.row(1).dot(x3DSt)+tcw1.at<float>(1);
                    const float sinvz1 = 1.0/sz1;

                    const float su1 = fx1*sx1*sinvz1+cx1;
                    const float sv1 = fy1*sy1*sinvz1+cy1;
                    const float dl1s = l1.at<float>(0) * su1 + l1.at<float>(1) * sv1 + l1.at<float>(2); // distance point-line
                    if((dl1s*dl1s)>3.84*sigmaSquare1)
                        continue;


                    const float ex1 = Rcw1.row(0).dot(x3DEt)+tcw1.at<float>(0);
                    const float ey1 = Rcw1.row(1).dot(x3DEt)+tcw1.at<float>(1);
                    const float einvz1 = 1.0/ez1;         

                    const float eu1 = fx1*ex1*einvz1+cx1;
                    const float ev1 = fy1*ey1*einvz1+cy1;
                    const float dl1e = l1.at<float>(0) * eu1 + l1.at<float>(1) * ev1 + l1.at<float>(2); // distance point-line
                    if((dl1e*dl1e)>3.84*sigmaSquare1)
                        continue;                

                    //Check reprojection error in second keyframe                                          
                    const float sigmaSquare2 = pKF2->mvLineLevelSigma2[kl2.octave];

                    const float sx2 = Rcw2.row(0).dot(x3DSt)+tcw2.at<float>(0);
                    const float sy2 = Rcw2.row(1).dot(x3DSt)+tcw2.at<float>(1);
                    const float sinvz2 = 1.0/sz2;         

                    const float su2 = fx2*sx2*sinvz2+cx2;
                    const float sv2 = fy2*sy2*sinvz2+cy2;
                    const float dl2s = l2.at<float>(0) * su2 + l2.at<float>(1) * sv2 + l2.at<float>(2);
                    if((dl2s*dl2s)>3.84*sigmaSquare2)
                        continue;


                    const float ex2 = Rcw2.row(0).dot(x3DEt)+tcw2.at<float>(0);
                    const float ey2 = Rcw2.row(1).dot(x3DEt)+tcw2.at<float>(1);
                    const float einvz2 = 1.0/ez2;       

                    float eu2 = fx2*ex2*einvz2+cx2;
                    float ev2 = fy2*ey2*einvz2+cy2;
                    const float dl2e = l2.at<float>(0) * eu2 + l2.at<float>(1) * ev2 + l2.at<float>(2);
                    if((dl2e*dl2e)>3.84*sigmaSquare2)
                        continue;                    
                }

                //Check scale consistency
                cv::Mat x3DM = 0.5*(x3DS+x3DE);
                
                cv::Mat normal1 = x3DM-Ow1;
                float dist1 = cv::norm(normal1);

                cv::Mat normal2 = x3DM-Ow2;
                float dist2 = cv::norm(normal2);

                if(dist1==0 || dist2==0)
                    continue;

                const float linesRatioDist = dist2/dist1;
                const float linesRatioOctave = mpCurrentKeyFrame->mvLineScaleFactors[kl1.octave]/pKF2->mvLineScaleFactors[kl2.octave];

                if(linesRatioDist*linesRatioFactor<linesRatioOctave || linesRatioDist>linesRatioOctave*linesRatioFactor)
                    continue;                
            
                // Triangulation is successful
                //MapLinePtr pML = new MapLine(x3DS,x3DE,mpMap,mpCurrentKeyFrame);
                MapLinePtr pML = MapLineNewPtr(x3DS,x3DE,mpMap,mpCurrentKeyFrame);
 
                if( pML->AddObservation(mpCurrentKeyFrame,idx1) )            
                    mpCurrentKeyFrame->AddMapLine(pML,idx1);                    
                
                if( pML->AddObservation(pKF2,idx2) )
                    pKF2->AddMapLine(pML,idx2);

                pML->ComputeDistinctiveDescriptors();

                pML->UpdateNormalAndDepth();

                mpMap->AddMapLine(pML);
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
        nn=20;
    const vector<KeyFramePtr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFramePtr> vpTargetKFs;
    for(vector<KeyFramePtr>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFramePtr pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId; // this is used to avoid pushing duplicates 

        // Extend to some second neighbors
        const vector<KeyFramePtr> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFramePtr>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFramePtr pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPointPtr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFramePtr>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFramePtr pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

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
        }
    
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

cv::Mat LocalMapping::ComputeF12(KeyFramePtr& pKF1, KeyFramePtr& pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R12*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::ComputeF12_H12(KeyFramePtr& pKF1, KeyFramePtr& pKF2, cv::Mat& F12, cv::Mat& H12, cv::Mat& e1)      
{
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    const cv::Mat R12 = R1w*R2w.t();
    const cv::Mat t12 = -R12*t2w+t1w;

    const cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    
    const cv::Mat K1invT = (cv::Mat_<float>(3,3) <<           pKF1->invfx,                     0,   0., 
                                                                        0,           pKF1->invfy,   0., 
                                                    -pKF1->cx*pKF1->invfx, -pKF1->cy*pKF1->invfy,   1.);
    
    const cv::Mat K2inv = (cv::Mat_<float>(3,3) <<  pKF2->invfx,           0,  -pKF2->cx*pKF2->invfx, 
                                                              0, pKF2->invfy,  -pKF2->cy*pKF2->invfy, 
                                                              0,           0,                     1.);
    
    const cv::Mat R12xK2inv = R12*K2inv;

    e1  = K1*t12;
    H12 = K1*R12xK2inv;
    F12 = K1invT*t12x*R12xK2inv; //K1.t().inv()*t12x*R12*K2.inv();
    
#if 0    
    const cv::Mat e1x = SkewSymmetricMatrix(e1);    
    cv::Mat F12_2 = e1x*H12;
    //F12 = F12/cv::norm(F12);
    
    std::cout << "dist: " << cv::norm(F12_2/cv::norm(F12_2) - F12/cv::norm(F12)) << std::endl;
#endif
    
#if 0    
    const cv::Mat &K2 = pKF2->mK;    
    cv::Mat F12_2 = K1.t().inv()*t12x*R12*K2.inv();
    std::cout << "dist: " << cv::norm(F12_2 - F12) << std::endl;    
#endif
    
}


void LocalMapping::ComputeF12_H21(KeyFramePtr& pKF1, KeyFramePtr& pKF2, cv::Mat& F12, cv::Mat& H21, cv::Mat& e2)      
{
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    const cv::Mat R12 = R1w*R2w.t();
    const cv::Mat t12 = -R12*t2w+t1w;

    const cv::Mat R21 = R2w*R1w.t();
    const cv::Mat t21 = -R21*t1w+t2w;    

    const cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;    
    
    const cv::Mat K1inv = (cv::Mat_<float>(3,3) <<  pKF1->invfx,           0,  -pKF1->cx*pKF1->invfx, 
                                                              0, pKF1->invfy,  -pKF1->cy*pKF1->invfy, 
                                                              0,           0,                     1.);                                                
    
    const cv::Mat K2inv = (cv::Mat_<float>(3,3) <<  pKF2->invfx,           0,  -pKF2->cx*pKF2->invfx, 
                                                              0, pKF2->invfy,  -pKF2->cy*pKF2->invfy, 
                                                              0,           0,                     1.);
    
    const cv::Mat R12xK2inv = R12*K2inv;    

    const cv::Mat R21xK1inv = R21*K1inv;

    e2  = K2*t21;
    H21 = K2*R21xK1inv;
    F12 = K1inv.t()*t12x*R12xK2inv; //K1.t().inv()*t12x*R12*K2.inv();
    
#if 0    
    const cv::Mat e1x = SkewSymmetricMatrix(e1);    
    cv::Mat F12_2 = e1x*H12;
    //F12 = F12/cv::norm(F12);
    
    std::cout << "dist: " << cv::norm(F12_2/cv::norm(F12_2) - F12/cv::norm(F12)) << std::endl;
#endif
    
#if 0    
    const cv::Mat &K2 = pKF2->mK;    
    cv::Mat F12_2 = K1.t().inv()*t12x*R12*K2.inv();
    std::cout << "dist: " << cv::norm(F12_2 - F12) << std::endl;    
#endif
    
}

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
    vector<KeyFramePtr> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFramePtr>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFramePtr pKF = *vit;
        if(pKF->mnId==0)
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
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFramePtr, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFramePtr, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFramePtr pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thPointObs)
                                    break;
                            }
                        }
                        if(nObs>=thPointObs)
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
                                (pKF->mvDepthLineStart[i] > pKF->mThDepth || pKF->mvDepthLineStart[i] < 0) || 
                                (pKF->mvDepthLineEnd[i] > pKF->mThDepth || pKF->mvDepthLineEnd[i] < 0)
                              )
                                continue;
                        }

                        nMLs++;
                        if(pML->Observations()>thLineObs)
                        {
                            const int &scaleLevel = pKF->mvKeyLinesUn[i].octave;
                            const map<KeyFramePtr, size_t> observations = pML->GetObservations();
                            int nObs=0;
                            for(map<KeyFramePtr, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                            {
                                KeyFramePtr pKFi = mit->first;
                                if(pKFi==pKF)
                                    continue;
                                const int &scaleLeveli = pKFi->mvKeyLinesUn[mit->second].octave;

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
        
        //if( (nRedundantPointObservations + nRedundantLineObservations) > 0.9*(nMPs + nMLs) )  
        if( (nRedundantPointObservations + Tracking::sknLineTrackWeigth*nRedundantLineObservations) > 0.9*(nMPs + Tracking::sknLineTrackWeigth*nMLs) )          
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             
                         0,  -v.at<float>(2),   v.at<float>(1),
             v.at<float>(2),               0,  -v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),               0);
}

void LocalMapping::RequestReset()
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
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mlpRecentAddedMapLines.clear();
        mbResetRequested=false;
    }
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


} //namespace PLVS
