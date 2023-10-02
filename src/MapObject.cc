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

#include "MapObject.h"
#include "ORBextractor.h"
#include "Sim3Utils.h"
#include "Logger.h"
#include "Converter.h"

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Atlas.h"
#include "Utils.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex>
#include <sstream> 


#define VERBOSE 1
#define USE_FLANN_MATCHER 1

namespace PLVS2
{

ObjectObservation::ObjectObservation():
nId(-1), fScale(1.0f), fDistance(std::numeric_limits<float>::max()),
nInliers(0),nMatchedMapPoints(0), fReprojectionError(0.f), fSim3Error(-1.f), bFromKF(false)
{};
    
// Copy constructor 
ObjectObservation::ObjectObservation(const ObjectObservation &other)
{
    *this = other; 
}

ObjectObservation& ObjectObservation::operator=(const ObjectObservation& other)
{
    if (this != &other) 
    { 
        nId = other.nId;
        Tkc = other.Tkc;
        //Rco = other.Rco.clone();
        //tco = other.tco.clone();   
        Tco = other.Tco;

        fScale = other.fScale;
        fDistance = other.fDistance;
        nInliers = other.nInliers;
        fReprojectionError = other.fReprojectionError; 
    
        nMatchedMapPoints = other.nMatchedMapPoints; 
        fSim3Error = other.fSim3Error; 
    
        bFromKF = other.bFromKF;
    }    
    return *this;    
}

// Get observation SE3 transformation Tko (from Object to KeyFrame)
Sophus::SE3f ObjectObservation::GetSE3() const 
{
    //cv::Mat_<float> Tco = cv::Mat_<float>::eye(4,4);    
    // this->Rco.copyTo( Tco.rowRange(0,3).colRange(0,3) );  
    // this->tco.copyTo( Tco.rowRange(0,3).col(3) );             

    if(bFromKF)
    {
        return this->Tco;  // here Tkc is the identity 
    }
    else
    {    
        return (this->Tkc)*(this->Tco);
    }
}

bool operator<(const ObjectObservation& l, const ObjectObservation& r)
{
    if ( fabs( l.fDistance - r.fDistance ) < 0.5 )
    {
        return l.fReprojectionError < r.fReprojectionError;
    }
    else
    {
        return l.fDistance < r.fDistance;
    }    
}

std::ostream &operator<<( std::ostream &out, const ObjectObservation &obs )
{
    out << "from KF: " << (int)obs.bFromKF << std::endl;   
    out << "object ID: " << obs.nId << std::endl;
    //out << "Tkc: " << obs.Tkc << std::endl;         
    // out << "Rco: " << obs.Rco << std::endl; 
    // out << "tco: " << obs.tco << std::endl;      
    //out << "Tco: " << obs.Tco << std::endl;  
    out << "scale: " << obs.fScale << std::endl; 
    out << "distance: " << obs.fDistance << std::endl; 
    out << "num inliers:" << obs.nInliers << std::endl; 
    out << "num matched points:" << obs.nMatchedMapPoints << std::endl; 
    out << "image reprojection error:" << obs.fReprojectionError << std::endl; 
    out << "sim3 error:" << obs.fSim3Error << std::endl;  

    return out;
} 
    
struct CompareKeyFrameObjectObservationPair
{
    inline bool operator()(const std::pair<KeyFramePtr,ObjectObservation>& a, const std::pair<KeyFramePtr,ObjectObservation>& b)
    {
        const ObjectObservation& obsa = a.second; 
        const ObjectObservation& obsb = b.second;
        return obsa < obsb;
    }
};

    
    

int MapObject::skNumMinInliers = 20;
float MapObject::skMaxImgReprojectionError = 1; // [pixels]
float MapObject::skMaxSim3Error = 0.01; // [m]    
float MapObject::skMatchRatio = 0.8f;

const float MapObject::kObjectSize = 1.f; // [m]  this is just an initial reference guess to move from pixels to meters world (and aim at better matrices conditioning) 
const int MapObject::kNumMinObvervationsForFiltering = 2; 

const std::string MapObject::kNameSaveLocFile = "MapObjectData";

const float MapObject::kFilteringQuadSize = 1.1; // percentage with respect to reference image width  

long unsigned int MapObject::nNextId=0;
mutex MapObject::mGlobalMutex;

MapObject::MapObject(Map* pMap, cv::Mat& imgObject, cv::Mat& K, cv::Mat& distCoef, float matchRatio): 
mnId(0), nObs(0), mnFirstFrame(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(-1),
mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),mnCorrectedReference(0), mnBAGlobalForKF(0),           
mpMap(pMap), mImgRef(imgObject.clone()), mK(K.clone()), mDistCoef(distCoef.clone()), mfNNmatchRatio(matchRatio), mnLastNumMatchedPoints(0),
mbObjectDetectedInCurrenFrame(false), mbObjectVisibleInCurrenFrame(false), mbLocalizedInCurrentFrame(false),
mbScaleFixed(false), mbLocalized(false), mpRefKF(static_cast<KeyFramePtr>(NULL)), mdScale(1.f), mbBad(false), mbActive(true)
{
    // parameters from https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html
    int table_number = 12; 
    int key_size = 20; 
    int multi_probe_level = 2;       
    
#if USE_FLANN_MATCHER     
    cv::Ptr<cv::flann::IndexParams> pIndexParams;        
    pIndexParams = cv::makePtr<cv::flann::LshIndexParams>(table_number, key_size, multi_probe_level);    
    mpMatcher = std::make_shared<cv::FlannBasedMatcher>(pIndexParams);     
#else
    mpMatcher = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING);
#endif
    
    //mSow = cv::Mat_<float>::eye(4,4);
    //mSwo = cv::Mat_<float>::eye(4,4);  
    
    unique_lock<mutex> lock(mpMap->mMutexObjectCreation);
    mnId=nNextId++;    
   
}

void MapObject::InitFeatures(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST)
{
    // Detect the keypoints using ORB Detector
    ORBextractor extractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
    vector<int> vLapping = {0,0};
    extractor( mImgRef, cv::Mat(), mvKeys, mDescriptors, vLapping);       
    
    std::cout << "MapObject - init num features: " << mvKeys.size() << std::endl;
    
    UndistortReferenceKeyPoints();
    InitImgCorners();
    Init3DRefPoints();
    
    mpMatcher->add(mDescriptors);
    mpMatcher->train();    
}


void MapObject::UndistortReferenceKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    int N = mvKeys.size();
    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void MapObject::InitImgCorners()
{
    mvImgCornersRef.resize(4);
    mvImgCornersRef[0] = cv::Point2f(0, 0);
    mvImgCornersRef[1] = cv::Point2f(mImgRef.cols, 0);
    mvImgCornersRef[2] = cv::Point2f(mImgRef.cols, mImgRef.rows);
    mvImgCornersRef[3] = cv::Point2f(0, mImgRef.rows);
    
    mvImCornersDetectedInCurrentFrame.resize(4);    
    mvImCornersReprojected.resize(4);
}    
    

void MapObject::Init3DRefPoints()
{
    unique_lock<mutex> lock(m3dCornersMutex);    
    
    const double scale = MapObject::kObjectSize/mImgRef.cols; // this is just an initial reference guess to move from pixels to meters world (and aim at better matrices conditioning)
    
    const float cx = 0.5*mImgRef.cols;
    const float cy = 0.5*mImgRef.rows;        
    
    size_t N = mvKeysUn.size();
    mv3dRefPoints.resize(N); 
    for(size_t jj=0; jj<N; jj++)
    {
        mv3dRefPoints[jj].x() =  (mvKeysUn[jj].pt.x - cx)*scale;
        mv3dRefPoints[jj].y() = -(mvKeysUn[jj].pt.y - cy)*scale;
        mv3dRefPoints[jj].z() = 0;
    }    
    
    mv3dRefCorners.resize(4);
    mv3dCorners.resize(4);
    
    // for(int jj=0;jj<4;jj++)
    // { 
    //     mv3dRefCorners[jj] = cv::Mat_<float>(3,1);
    //     mv3dCorners[jj] = cv::Mat_<float>(3,1);        
    // }    
    mv3dRefCorners[0](0) = (0. - cx)*scale;            mv3dRefCorners[0](1) = -(0. - cy)*scale;           mv3dRefCorners[0](2) = 0.;
    mv3dRefCorners[1](0) = (mImgRef.cols - cx)*scale;  mv3dRefCorners[1](1) = -(0. - cy)*scale;           mv3dRefCorners[1](2) = 0.;
    mv3dRefCorners[2](0) = (mImgRef.cols - cx)*scale;  mv3dRefCorners[2](1) = -(mImgRef.rows - cy)*scale; mv3dRefCorners[2](2) = 0.;
    mv3dRefCorners[3](0) = (0. - cx)*scale;            mv3dRefCorners[3](1) = -(mImgRef.rows - cy)*scale; mv3dRefCorners[3](2) = 0.;
    
    mvMPs = std::vector<MapPointPtr>(N,static_cast<MapPointPtr>(NULL));  
}


void MapObject::Detect(Frame* pFrame)
{
#if VERBOSE    
    std::cout << "object matching  ===========================================" << std::endl; 
#endif
    
    if(!mbActive) return; 
    
    // Reset current frame flags 
    mbObjectDetectedInCurrenFrame = mbObjectVisibleInCurrenFrame = mbLocalizedInCurrentFrame = false;
    
    // Reset current observation 
    mCurrentObservation = ObjectObservation();
    mCurrentObservation.nId = this->mnId; // id of the observed object 

    cv::Mat& frameDescriptors = pFrame->mDescriptors;
    const std::vector<cv::KeyPoint>& frameKeysUn = pFrame->mvKeysUn;
    const std::vector<MapPointPtr>& frameMapPoints = pFrame->mvpMapPoints;
    const int nRefPoints = frameKeysUn.size();
    
    std::vector<std::vector<cv::DMatch> > nn_matches; 
    
    std::vector<uchar> frameMask(nRefPoints,1);
    
    Project3DCorners(pFrame);    
        
    // if the object has been localized and is not visible exit 
    if(mbLocalized && !mbObjectVisibleInCurrenFrame) return; /// < EXIT POINT 
    
    // filter points outside reprojected frame 
    if(mbLocalized && mbObjectVisibleInCurrenFrame) FilterPoints(pFrame, frameMask);   
    
#if 0    
    std::vector< std::vector<uchar> > mask;
    mask.push_back(frameMask);
    mpMatcher->knnMatch( frameDescriptors, nn_matches, 2, mask); 
#else
    mpMatcher->knnMatch( frameDescriptors, nn_matches, 2);     
#endif
    
#if VERBOSE        
    std::cout << "object nn matches: " << nn_matches.size() << std::endl; 
#endif
    
    std::vector<cv::DMatch> matches;
    std::vector<bool> vbMatched(mvKeys.size(),false);
    std::vector<int> vMatchIndex(mvKeys.size(),-1);      
    
    for (size_t i = 0, iEnd=nn_matches.size(); i < iEnd; i++)
    {
        if(nn_matches[i].empty()) continue;
        
        const cv::DMatch& first = nn_matches[i][0];
        
        if(frameMask[first.queryIdx] == 0) continue; 
        
        if(nn_matches[i].size() == 2) 
        {
            const cv::DMatch& second = nn_matches[i][1];
            const float dist1 = first.distance;
            const int octave1 = frameKeysUn[first.queryIdx].octave;
            const float dist2 = second.distance;
            const int octave2 = frameKeysUn[second.queryIdx].octave;
            if ( ( octave1 == octave2 ) && (dist1 > mfNNmatchRatio * dist2) )
            {
                continue;
            }
        }
        //matched1.push_back(keypoints_object[first.queryIdx]);
        //matched2.push_back(keypoints_scene[first.trainIdx]);
#if 1      
        // check if we have already a match, if yes then check if we have a better distance and in case replace the match 
        //                                   if not then just add the new match 
        if(!vbMatched[first.trainIdx])
        {
            matches.push_back( first );
            vbMatched[first.trainIdx] = true;
            vMatchIndex[first.trainIdx] = matches.size()-1;
        }
        else
        {
            cv::DMatch& otherMatch = matches[vMatchIndex[first.trainIdx]];
            if( first.distance < otherMatch.distance )  
            {
                //std::cout << "updating match - replacing: (" << otherMatch.trainIdx<< ", "<< otherMatch.queryIdx <<"), dist: " << otherMatch.distance << std::endl;
                //std::cout << "                      with: (" << first.trainIdx<< ", "<< first.queryIdx <<"), dist: " << first.distance << std::endl;                   
                otherMatch = first;
            }
        }
#else
        matches.push_back( first );
#endif        
    }    
    
    mCurrentObservation.nInliers = matches.size(); // inliers after distance ratio filter
#if VERBOSE        
    std::cout << "num matches: " << matches.size() << " (after distance ratio filter) " << std::endl;  
#endif
    
    mbObjectDetectedInCurrenFrame = mCurrentObservation.nInliers > skNumMinInliers;
    if(!mbObjectDetectedInCurrenFrame) return;    
    
    
    // < Localize the object  in the image -------------------------------------
    
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for(size_t i=0, iEnd=matches.size(); i < iEnd; i++)
    {
        // get the keypoints from the good matches
        obj.push_back(mvKeysUn[matches[i].trainIdx].pt);
        scene.push_back(frameKeysUn[matches[i].queryIdx].pt);
    }
    
    const double ransacReprojThreshold = 3; // default value 3
    std::vector<uchar> outMask;        
    cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC, ransacReprojThreshold, outMask);    
    
    // < Update current corners ------------------------------------------------ 
    {
    unique_lock<mutex> lock(mMutexImgCorners);
    cv::perspectiveTransform(mvImgCornersRef, mvImCornersDetectedInCurrentFrame, H);    
    }
        
    //std::vector<cv::DMatch > inlier_matches;
    std::vector<cv::Point2f> objInliers;    
    std::vector<cv::Point2f> sceneInliers;   
    std::vector<cv::Point3f> obj3DRefInliers; 

    mCurrentObservation.nInliers = 0;
    {
    unique_lock<mutex> lock(mMutexPose); // for reserving exclusive access to mdScale
    for(size_t i=0, iEnd=matches.size(); i < iEnd; i++)
    {
        const cv::DMatch& match = matches[i];
        if( outMask[i])  // if inlier 
        {
            mCurrentObservation.nInliers++;
            //inlier_matches.push_back(good_matches[i]);
            //const cv::DMatch& match = matches[i];
            //inlier_matches.push_back(cv::DMatch(match.trainIdx, match.queryIdx, match.distance)); // swap query and train idxs
            objInliers.push_back( mvKeysUn[match.trainIdx].pt );
            sceneInliers.push_back( frameKeysUn[match.queryIdx].pt );
            
            const Eigen::Vector3f& refP3 = mv3dRefPoints[match.trainIdx];
            obj3DRefInliers.push_back(cv::Point3f(refP3.x()*mdScale,refP3.y()*mdScale,refP3.z()*mdScale));            
            
            MapPointPtr pMP = frameMapPoints[match.queryIdx];
            if(pMP)  mvMPs[match.trainIdx] = pMP; // update the matched point           
        }
#if 1       
        else
        {
            mvMPs[match.trainIdx] = static_cast<MapPointPtr>(NULL); // reset the match on the outliers
        }
#endif        
    }    
    }
    
    //mCurrentObservation.nInliers = inlier_matches.size();
    
#if VERBOSE        
    std::cout << "inliers: " << mCurrentObservation.nInliers << std::endl; 
#endif
    mbObjectDetectedInCurrenFrame = mCurrentObservation.nInliers > skNumMinInliers; // inliers after findHomography RANSAC
    if(!mbObjectDetectedInCurrenFrame) return;
    
    // < Compute image reprojection error --------------------------------------     
    
    std::vector<cv::Point2f> obj_reproj;
    mCurrentObservation.fReprojectionError = 0; 
    cv::perspectiveTransform(objInliers, obj_reproj, H);    
    for(size_t ii=0, iiEnd=sceneInliers.size(); ii<iiEnd; ii++)
    {
        mCurrentObservation.fReprojectionError += PLVS2::Utils::Pow2(sceneInliers[ii].x - obj_reproj[ii].x) + PLVS2::Utils::Pow2(sceneInliers[ii].y - obj_reproj[ii].y);
    }
    mCurrentObservation.fReprojectionError = sqrt( mCurrentObservation.fReprojectionError/(2.*sceneInliers.size()) ); // residual (error in one image)
#if VERBOSE        
    std::cout << "image reprojectionError: " << mCurrentObservation.fReprojectionError << std::endl;
#endif
    
    mbObjectDetectedInCurrenFrame = mCurrentObservation.fReprojectionError < skMaxImgReprojectionError;    
    if(!mbObjectDetectedInCurrenFrame) return;
    
    // < Scale estimation and map points update -------------------------------- 
    
    cv::Mat_<cv::Vec3d> points1;
    cv::Mat_<cv::Vec3d> points2;     
    points1.reserve(mnLastNumMatchedPoints); // use last match information to reserve memory 
    points2.reserve(mnLastNumMatchedPoints);
    std::vector<int> idxMap; 
    idxMap.reserve(mnLastNumMatchedPoints);
    
    cv::Mat_<double> Rswo;  
    cv::Mat_<double> tswo; 
    double scale = 1.;
    
    // fill in points1 and point2
    mCurrentObservation.nMatchedMapPoints = 0; 
    for(size_t ii=0, iiEnd=mvMPs.size(); ii<iiEnd; ii++)
    {
        MapPointPtr pMP = mvMPs[ii];
        if(pMP && !pMP->isBad())  
        {
            // check if points have been replaced 
            MapPointPtr pRep = pMP->GetReplaced();
            if(pRep) mvMPs[ii] = pRep;            
            
            // update reference descriptors 
            mDescriptors.row(ii) = mvMPs[ii]->GetDescriptor();
            
            Eigen::Vector3f p3Dmap = mvMPs[ii]->GetWorldPos();
            Eigen::Vector3f& p3Dref = mv3dRefPoints[ii];             
            points1.push_back( cv::Vec3d( p3Dref.x(), p3Dref.y(), p3Dref.z()) );
            points2.push_back( cv::Vec3d( p3Dmap(0), p3Dmap(1), p3Dmap(2) ) );
            idxMap.push_back(ii);
            mCurrentObservation.nMatchedMapPoints++;
        }        
    }
    mnLastNumMatchedPoints = mCurrentObservation.nMatchedMapPoints;
    
    // < Estimate object pose and scale ---------------------------------------- 
    
#if VERBOSE        
    std::cout << "computing scale, numPointsSim3: " << mCurrentObservation.nMatchedMapPoints << std::endl; 
#endif
    // here we minimize  sum_i || points2 - (scale*R*points1 + t) ||
    mbLocalizedInCurrentFrame = false;
    mCurrentObservation.fSim3Error = -1;
    if(mCurrentObservation.nMatchedMapPoints > skNumMinInliers)
    {
        std::vector<bool> mask(points1.rows,true);
        mCurrentObservation.fSim3Error = Sim3Utils::FindSimTransform(points1, points2, Rswo, tswo, scale, mask);
#if VERBOSE            
        std::cout << "sim3 alignment error1: " << mCurrentObservation.fSim3Error << std::endl;          
#endif
        // second time for recomputing without marked outliers 
        mCurrentObservation.fSim3Error = Sim3Utils::FindSimTransform(points1, points2, Rswo, tswo, scale, mask);   
        mCurrentObservation.fScale = (float) scale; 
#if VERBOSE            
        std::cout << "sim3 alignment error2: " << mCurrentObservation.fSim3Error << std::endl;         
        std::cout << "scale: " << mCurrentObservation.fScale << std::endl;
#endif
        if( mCurrentObservation.fSim3Error < skMaxSim3Error ) 
        {
            mbScaleFixed = true; 
            mbLocalizedInCurrentFrame = true; 
            //ApplyScaleTo3DRef(mCurrentObservation.fScale); // we do not modify anymore the scale of the reference object 
#if VERBOSE                
            std::cout << "object width: " << ( scale*(mv3dRefCorners[0] - mv3dRefCorners[1]) ).norm() << std::endl;            
#endif
        }
        
        // reset outlier map points 
        int numOutliers = 0; 
        for(size_t ii=0,iiEnd=mask.size();ii<iiEnd;ii++)
        {
            if(!mask[ii])  
            {
                mvMPs[idxMap[ii]] = static_cast<MapPointPtr>(NULL);  
                numOutliers++;
            }
        }
        mCurrentObservation.nMatchedMapPoints -= numOutliers;
#if VERBOSE            
        std::cout << "num sim3 outliers: " << numOutliers << std::endl;        
#endif        
    }

    // < Set estimated sim, update ref KF and setup observation 
    if(mbLocalizedInCurrentFrame) 
    {
        // transform current corner 
        const Eigen::Matrix3f Rwc = pFrame->GetRotationInverse();
        const Eigen::Vector3f twc = pFrame->GetCameraCenter();
                
        // use the current scale estimation 
        //Rswo.convertTo(Rwo,CV_32F);
        //tswo.convertTo(two,CV_32F);   

        Eigen::Matrix3f Rwo = Converter::toMatrix3f(Rswo);
        Eigen::Vector3f two = Converter::toVector3f(tswo);


        const Eigen::Matrix3f Rwct = Rwc.transpose();
        Eigen::Matrix3f Rco;
        Eigen::Vector3f tco;
        Rco = Rwct * Rwo; 
        tco = Rwct * (two - twc);

#if 0
        {
            // use PnP to estimate the pose 
            cv::Mat rvec;
            cv::Mat cvRco; 
            cv::Mat cvtco;

            cv::solvePnP(obj3DRefInliers, sceneInliers, mK, mDistCoef, rvec, cvtco, /*useExtrinsicGuess*/ false, cv::SOLVEPNP_ITERATIVE);    

            cv::Rodrigues(rvec, cvRco);
            Rco.convertToRco, CV_32F);  
            tco.convertTotco, CV_32F);

            Rwo = Rwc * Converter::Matrix3f(Rco);
            two = Rwc * Converter::Vector3f(tco) + twc;
        }
#endif        
        mCurrentObservation.Tco = Sophus::SE3f(Rco, tco);

        mCurrentObservation.fDistance = two.norm();
        
        KeyFramePtr pRefKF;
        {
        unique_lock<mutex> lock(mMutexObservations);            
        if(!mpRefKF) mpRefKF = pFrame->mpReferenceKF;
        pRefKF = mpRefKF; // make a copy for using it outside the lock 
        }
        if(mnFirstFrame<0) mnFirstFrame = pFrame->mnId;
        mnLastFrameSeen = pFrame->mnId;
        
        // the first time we enter here, we have localized the object 
        if(!mbLocalized)
        {
            SetSim3InversePose(Rwo, two, scale);
            mbLocalized = true; // this is already set in SetSim3InversePose(), we put it here just for sake of clarity 
            
            /* just for testing forward and inverse sim set
            cv::Mat Row2 = GetRotation();
            cv::Mat tow2 = GetTranslation();            
            double scale = GetScale();
            std::cout << "Row2: " << Row2 << std::endl; 
            std::cout << "tow2: " << tow2 << std::endl;             
            cv::Mat Sow2 = cv::Mat_<float>::eye(4,4);
            Row2/=scale;
            Row2.copyTo(Sow2.rowRange(0,3).colRange(0,3));
            tow2.copyTo(Sow2.rowRange(0,3).col(3));
            SetSim3Pose(Sow2);*/
        }
                
        // cv::Mat Twc = cv::Mat_<float>::eye(4,4);
        // Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        // twc.copyTo(Twc.rowRange(0,3).col(3));    
        Sophus::SE3f Twc = Sophus::SE3f(Rwc, twc); 
        Sophus::SE3f Tkw = pRefKF->GetPose();
        mCurrentObservation.Tkc = Tkw*Twc; 
        
        this->AddObservation(pRefKF, mCurrentObservation);
                
        nObs++;
#if VERBOSE            
        std::cout << "num observations: " << nObs << std::endl;         
#endif
    }
    
}

std::vector<cv::Point2f> MapObject::GetCurrentImgCorners()
{
    unique_lock<mutex> lock(mMutexImgCorners);
    return vector<cv::Point2f>(mvImCornersDetectedInCurrentFrame.begin(),mvImCornersDetectedInCurrentFrame.end());    
}

std::vector<cv::Point2f> MapObject::GetCurrentImgCornersReprojected()
{
    unique_lock<mutex> lock(mMutexImgCornersReprojected);
    return vector<cv::Point2f>(mvImCornersReprojected.begin(),mvImCornersReprojected.end());    
}
    
std::vector<cv::Point3f> MapObject::GetCurrent3DCorners()
{
    unique_lock<mutex> lock(m3dCornersMutex); 
    std::vector<cv::Point3f> points(4);
    for(size_t ii=0;ii<4;ii++)
    {
        const Eigen::Vector3f& pointMat = mv3dCorners[ii];
        points[ii] = cv::Point3f(pointMat(0),pointMat(1),pointMat(2));
    }
    return points; 
}

void MapObject::CheckMapPointsReplaced()
{
    for(size_t i=0, iEnd=mvMPs.size(); i<iEnd; i++)
    {
        MapPointPtr pMP = mvMPs[i];

        if(pMP)
        {
            MapPointPtr pRep = pMP->GetReplaced();
            if(pRep)
            {
                mvMPs[i] = pRep;
            }
        }
    }
}

void MapObject::ApplyScaleTo3DRef(double scale)
{
    size_t N = mv3dRefPoints.size();
    for(size_t jj=0; jj<N; jj++)
    {
        mv3dRefPoints[jj].x() *= scale;
        mv3dRefPoints[jj].y() *= scale;
        //mv3dRefPoints[jj].z = 0;
    }    
   
    mv3dRefCorners[0] *= scale;
    mv3dRefCorners[1] *= scale;
    mv3dRefCorners[2] *= scale;
    mv3dRefCorners[3] *= scale;                   
}

void MapObject::Project3DCorners(Frame* pFrame)
{
    if(!mbLocalized) return; // do not project until the object is localized 
    
    mbObjectVisibleInCurrenFrame = false; 
        
    const float fx = pFrame->fx;
    const float fy = pFrame->fy;
    const float cx = pFrame->cx;
    const float cy = pFrame->cy;
    
    // const cv::Mat Rcw = pFrame->GetRotation();
    // const cv::Mat tcw = pFrame->GetTranslation();
    const Sophus::SE3f Tcw = pFrame->GetPose();
        
    unique_lock<mutex> lock(mMutexImgCornersReprojected);  
    unique_lock<mutex> lock2(m3dCornersMutex);
    for(size_t ii=0; ii<4; ii++)
    {
        const Eigen::Vector3f& p3Dw = mv3dCorners[ii];    
    
        // 3D in camera coordinates
        //const cv::Mat p3Dc = Rcw*p3Dw+tcw;
        const Eigen::Vector3f p3Dc = Tcw*p3Dw;
        const float &pcX = p3Dc(0);
        const float &pcY = p3Dc(1);
        const float &pcZ = p3Dc(2);
    
        const float invz = 1.0f/pcZ;
        const double u = fx*pcX*invz+cx;
        const double v = fy*pcY*invz+cy;          
        mvImCornersReprojected[ii].x = u;
        mvImCornersReprojected[ii].y = v;
               
        mbObjectVisibleInCurrenFrame = mbObjectVisibleInCurrenFrame || 
            ( (pcZ > 0) && (u >= pFrame->mnMinX) && (u < pFrame->mnMaxX) && (v >= pFrame->mnMinY) && (v < pFrame->mnMaxY) );
    }

}

void MapObject::FilterPoints(Frame* pFrame, std::vector<uchar>& frameMask)
{ 
    if( !(mbLocalized && mbObjectVisibleInCurrenFrame) ) return; // do not filter until the object is localized and if not visible  
  
    std::vector<cv::Point2f> vImCornersFilter; // this represents an enlarged corners quad reprojection 
    
    {
    unique_lock<mutex> lock(mMutexImgCornersReprojected);
    
    vImCornersFilter = mvImCornersReprojected;    
    const float deltaFilter = (kFilteringQuadSize-1.0f)*mImgRef.cols;
    
    for(size_t ii=0; ii < 4; ii++)
    {        
        cv::Point2f plus(0.f,0.f);
        for(size_t hh=0; hh < 4; hh++)
        {
            if(hh!=ii)
            {
                plus.x += mvImCornersReprojected[ii].x - mvImCornersReprojected[hh].x;
                plus.y += mvImCornersReprojected[ii].y - mvImCornersReprojected[hh].y;                        
            }
        }    
        // plus is the sum of delta from other corners 
        float modulus = sqrt(plus.x*plus.x + plus.y*plus.y); // let's normalize it 
        plus.x /=modulus; plus.y /=modulus;
        vImCornersFilter[ii].x = mvImCornersReprojected[ii].x +  plus.x * deltaFilter;       
        vImCornersFilter[ii].y = mvImCornersReprojected[ii].y +  plus.y * deltaFilter;
    }
#if 0    
    mvImCornersReprojected = vImCornersFilter; // enable this line to visualize the filtering quad 
#endif
    } // end lock 
    
    std::vector<cv::KeyPoint>& frameKeysUn = pFrame->mvKeysUn;  
    int numFilteredPoints = 0; 
    for(size_t jj=0, jjEnd=frameKeysUn.size(); jj<jjEnd; jj++)
    {
        cv::Point2f& pt = frameKeysUn[jj].pt;
        if( cv::pointPolygonTest(vImCornersFilter, pt, false) < 0 )
        {
            numFilteredPoints++;
            frameMask[jj] = 0;
        }
    }
#if VERBOSE        
    std::cout << "num filtered points " << numFilteredPoints << ", remaining: " << frameKeysUn.size() - numFilteredPoints << std::endl; 
#endif

}

ObjectObservation MapObject::GetCurrentObjectObservation()
{
    unique_lock<mutex> lock(mMutexCurrentObservation);
    ObjectObservation observation = mCurrentObservation;
    return observation; 
}

// this is used to add a temporary frame-observation till a proper keyframe observation is not added 
bool MapObject::AddObservation(const KeyFramePtr& pKF, const ObjectObservation& observation)
{
    if( mbObjectDetectedInCurrenFrame && mbLocalized)
    {
        unique_lock<mutex> lock(mMutexObservations);
        // add a frame-observation only in case a proper keyframe-observation does not exist yet
        if(!mObservations.count(pKF))
        {
            mObservations[pKF]=observation; 
            return true;
        }   
    }
    return false; 
}

void MapObject::AddKeyFrameObservation(const KeyFramePtr& pKF)
{
    if( mbObjectDetectedInCurrenFrame && mbLocalized)
    {
        //ObjectObservation observation = GetCurrentObservation();  
        // N.B.: this is used in the Tracking thread, the same which calls Detect() => no need to copy the current observation 
        unique_lock<mutex> lock(mMutexCurrentObservation);
        mCurrentObservation.Tkc = Sophus::SE3f();//cv::Mat_<float>::eye(4,4);
        mCurrentObservation.bFromKF = true;

        UpdateRefKeyFrame(pKF);     
        
#if VERBOSE
        std::cout << "MapObject::AddKeyFrameObservation() (" << pKF->mnId << ", " <<  mCurrentObservation << ")" << std::endl; 
#endif        
        AddObservation(pKF,mCurrentObservation);   
    }
}

void MapObject::EraseObservation(const KeyFramePtr& pKF)
{
    bool bBad=false;
    
    unique_lock<mutex> lock(mMutexObservations);
    if(mObservations.count(pKF))
    {
        nObs--;
        
        mObservations.erase(pKF);

        if(mpRefKF==pKF)
            mpRefKF=mObservations.begin()->first;
        
        if(nObs<2)
           bBad=true;        
    }
    
    if(bBad)
        SetBadFlag();    
}

std::map<KeyFramePtr,ObjectObservation> MapObject::GetObservations()
{
    unique_lock<mutex> lock(mMutexObservations);
    return mObservations;
}

bool MapObject::IsInKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexObservations);
    return (mObservations.count(pKF));
}

KeyFramePtr MapObject::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexObservations);
    return mpRefKF;
}

void MapObject::UpdateRefKeyFrame(const KeyFramePtr& pKF)
{ 
    unique_lock<mutex> lock(mMutexObservations);    
    if(!mpRefKF) 
    {
        mpRefKF = pKF;
        return;
    }
    else
    {
        if(mpRefKF != pKF)
        {
            if(mObservations.count(mpRefKF))
            {
                const ObjectObservation& observationRef = mObservations[mpRefKF];
                if(mCurrentObservation < observationRef) mpRefKF = pKF;
            }
            else
            {
                std::cerr << "MapPlanarObject::UpdatetRefKeyFrame() - error - this should not happen" << std::endl;
                quick_exit(-1);
            }
        }
    }
}

void MapObject::SaveRefObservations()
{
    unique_lock<mutex> lock(mMutexObservations);       
    
    std::stringstream ssFilename;
    ssFilename << kNameSaveLocFile << mnId << ".txt";
    Logger logger(ssFilename.str());
    std::stringstream ss; 
    
    if(mbLocalized)
    {
        // save Swo = [s*Rwo, two; 0, 1] 
        //ss << "Swo: " << mSwo << std::endl;
        Eigen::Matrix3f Rwo = GetInverseRotation();
        Eigen::Vector3f two = GetInverseTranslation();    
        ss << "Rwo: " << Rwo << std::endl;
        ss << "two: " << two << std::endl;  
        ss << "scale: " << mdScale << std::endl; 
        
        //Eigen::Matrix<double,3,3> eigRot = Converter::toMatrix3d(Rwo);        
        //Eigen::Vector3d ea = Rwo.eulerAngles(0, 1, 2); 
        //ss << "Euler angles (X,Y,Z) from Rwo:" << endl;
        //ss << ea << endl;
                
        // now save all the observation 
        std::vector< std::pair<KeyFramePtr,ObjectObservation> > vObjectObservations; 
        std::map<KeyFramePtr,ObjectObservation>::iterator it; 
        for(it=mObservations.begin(); it!=mObservations.end(); it++)
        {
            vObjectObservations.push_back(*it);
        }
        ss << "num observations: " << vObjectObservations.size() << std::endl;         

        // sort the observations 
        std::sort(vObjectObservations.begin(), vObjectObservations.end(), CompareKeyFrameObjectObservationPair());

        for(size_t ii=0, iiEnd=vObjectObservations.size(); ii<iiEnd; ii++)
        {
            std::pair<KeyFramePtr,ObjectObservation>& obs = vObjectObservations[ii];
            KeyFramePtr pKF = obs.first; 
            ObjectObservation& observation = obs.second; 

            const Sophus::SE3f Twk = pKF->GetPoseInverse();
            Sophus::SE3f Two = Twk * observation.GetSE3();

            ss << "----------------------------------------------" << std::endl;
            if(pKF == mpRefKF) ss << " **** reference KeyFramePtr ** " << std::endl;        
            ss << "KF id: " << pKF->mnId << std::endl;  
            ss << "bad: " << (int)mbBad << std::endl;
            ss << observation << std::endl;
            //ss << "Two: " << Two << std::endl; 

        }
    }
    else
    {
        ss << "object not localized" << std::endl;
    }

    logger << ss.str();  

    std::cout << "Saved map object info in " << ssFilename.str() << std::endl; 
            
}

void MapObject::Update3DCorners()
{
    if(!mbLocalized) return; 
        
    // transform current corners
    
    unique_lock<mutex> lock(mMutexPose);     
    // const cv::Mat Rwo = mSwo.rowRange(0,3).colRange(0,3);
    // const cv::Mat two = mSwo.rowRange(0,3).col(3);    

    {
    unique_lock<mutex> lock2(m3dCornersMutex);
    for(int jj=0;jj<4;jj++)
    {
        // < N.B.: here the scale is taken into account by Rwo (coming from a Sim3 configuration)
        //mv3dCorners[jj] =  Rwo * mv3dRefCorners[jj] + two;                      
        mv3dCorners[jj] =  mSwo * mv3dRefCorners[jj];     
    }
    }    
    
}


//  Sow = [Row/s, tow; 0, 1]  where tow = -(Row* two)/s  (when comparing Sow with Swo)
void MapObject::SetSim3Pose(const Sophus::Sim3f &Sow)
{
    {
    unique_lock<mutex> lock(mMutexPose);
        
#if 0    
    std::cout << "MapObject::SetSim3Pose() - mdScale: " << mdScale << std::endl;     
    std::cout << "MapObject::SetSim3Pose() - before: " << sqrt(3.0)/cv::norm(mSow.rowRange(0,3).colRange(0,3)) << std::endl; 
#endif
    
    mSow = Sow;
    
    //float m33 = mSow.at<float>(3,3);
    //if(m33 != 1.f)  mSow/=m33; // Sow = [Row/s, tow; 0, 1] 
    
    //const cv::Mat Rsow = mSow.rowRange(0,3).colRange(0,3);  // Row/s
    //mdScale = sqrt(3.0)/cv::norm(Rsow); 
    //std::cout << "MapObject::SetSim3Pose() - after: " << mdScale << std::endl;    
    //const cv::Mat Row = Rsow*mdScale;
    mdScale = 1.0/mSow.scale();
    
    // const cv::Mat tow = mSow.rowRange(0,3).col(3);
    // const cv::Mat Rswo = Row.t()*mdScale; // s*Rwo
    // Eigen::Vector3f tow = mSow.translation();
    // Eigen::Matrix3f Rswo = mSow.getRotationMatrix().transpose()*mdScale; // s*Rwo
    // mOw = -Rswo*tow; // two

    // mSwo = cv::Mat::eye(4,4,mSow.type()); // Swo = [s*Rwo, two; 0, 1] 
    // Rswo.copyTo(mSwo.rowRange(0,3).colRange(0,3));
    // mOw.copyTo(mSwo.rowRange(0,3).col(3));
    mSwo = mSow.inverse();
    mOw = mSwo.translation(); // two
    
#if 0    
    std::cout << "MapObject::SetSim3Pose() - mSwo: " << mSwo << std::endl;        
    std::cout << "MapObject::SetSim3Pose() - mSow: " << mSow << std::endl;      
        
    std::cout << "MapObject::SetSim3Pose() - norm(nSwo*Sow): " << cv::norm(mSwo*mSow) << std::endl;     
#endif
    
    mbLocalized = true;    
    }
    Update3DCorners();
}

 // Swo = [s*Rwo, two; 0, 1]  
void MapObject::SetSim3InversePose(const Eigen::Matrix3f &Rwo, const Eigen::Vector3f &two, const double scale) 
{
    {
    unique_lock<mutex> lock(mMutexPose);
            
    mdScale = scale;     
    
    //mSwo = cv::Mat::eye(4,4,mSwo.type()); // Swo = [s*Rwo, two; 0, 1] 
    mSwo = Sophus::Sim3f(Sophus::RxSO3f(scale, Rwo), two);

    //Eigen::Matrix3f Rswo = Rwo*scale; 
    //Rswo.copyTo(mSwo.rowRange(0,3).colRange(0,3));
    //two.copyTo(mSwo.rowRange(0,3).col(3));    
    //two.copyTo(mOw);
    mOw = two;
    
    // mSow = cv::Mat::eye(4,4,mSow.type()); // Sow = [Row/s, tow; 0, 1] 
    
    // const cv::Mat Rsow = Rwo.t()/scale;
    // const cv::Mat tow = -Rsow*two;    
    // Rsow.copyTo(mSow.rowRange(0,3).colRange(0,3));
    // tow.copyTo(mSow.rowRange(0,3).col(3));    

    mSow = mSwo.inverse();

#if 0    
    std::cout << "MapObject::SetSim3InversePose() - mSwo: " << mSwo << std::endl;        
    std::cout << "MapObject::SetSim3InversePose() - mSow: " << mSow << std::endl;      
        
    std::cout << "MapObject::SetSim3InversePose() - norm(nSwo*Sow): " << cv::norm(mSwo*mSow) << std::endl; 
    
    std::cout << "MapObject::SetSim3InversePose() - mdScale: " << mdScale << std::endl;      
#endif 
    
    mbLocalized = true;     
    }
    Update3DCorners();
}

Sophus::Sim3f MapObject::GetSim3Pose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mSow;
}

Sophus::Sim3f MapObject::GetSim3PoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mSwo;
}

Eigen::Vector3f MapObject::GetObjectCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return mOw; // two
}
 
Eigen::Matrix3f MapObject::GetRotation()
{
    // Sow = [Row/s, tow; 0, 1]
    unique_lock<mutex> lock(mMutexPose);    
    // cv::Mat Row = mSow.rowRange(0,3).colRange(0,3)*mdScale;
    // return Row; 
    return mSow.rotationMatrix(); 
}

// Sow = [Row/s, tow; 0, 1]   ("object" frame contains mv3dRefPoints)   NOTE: tow = -(Row* two)/s  (when comparing Sow with Swo)
Eigen::Vector3f MapObject::GetTranslation()
{
    // Sow = [Row/s, tow; 0, 1]    
    unique_lock<mutex> lock(mMutexPose);    
    // cv::Mat tow = mSow.rowRange(0,3).col(3).clone();
    // return tow; 
    return mSow.translation(); 
}

double MapObject::GetScale()
{
    unique_lock<mutex> lock(mMutexPose);    
    return mdScale;
}    

Eigen::Matrix3f MapObject::GetInverseRotation()
{
    // Swo = [s*Rwo, tow; 0, 1]
    // unique_lock<mutex> lock(mMutexPose);    
    // cv::Mat Rwo = mSwo.rowRange(0,3).colRange(0,3).clone()/mdScale;
    // return Rwo; 
    return mSwo.rotationMatrix(); 
}

Eigen::Vector3f MapObject::GetInverseTranslation()
{
    // Swo = [s*Rwo, tow; 0, 1]  
    unique_lock<mutex> lock(mMutexPose);    
    // cv::Mat two = mSwo.rowRange(0,3).col(3).clone();
    // return two; 
    return mSwo.translation(); 
}  

bool MapObject::isBad()
{
    unique_lock<mutex> lock(mMutexObservations);
    unique_lock<mutex> lock2(mMutexPose);
    return mbBad || !mbLocalized; // the object is bad at least till it is not localized 
}


void MapObject::SetBadFlag()
{
    std::map<KeyFramePtr,ObjectObservation> obs;
    {
        unique_lock<mutex> lock1(mMutexObservations);
        unique_lock<mutex> lock2(mMutexPose);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();        
    }
    for(map<KeyFramePtr,ObjectObservation>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;
        pKF->EraseMapObjectMatch(mit->second.nId);
    }

    mpMap->EraseMapObject(WrapPtr(this)); // N.B.: 1) we use a wrap pointer (empty deleter) since the raw ptr 'this' has not been created with the wrap pointer 
                                          //       2) the comparison operators for shared_ptr simply compare pointer values
}

Map* MapObject::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapObject::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

} //namespace PLVS2
