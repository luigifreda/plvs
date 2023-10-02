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



#include<iostream>

#include<mutex>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#ifdef USE_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "Tracking.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include "LineExtractor.h"
#include "LineMatcher.h"
#include "MapLine.h"
#include "Utils.h"
#include "Stopwatch.h"
#include "MapObject.h"

#define CREATE_NEW_LINES_ON_TRACKING 1   // set this to zero in order to check how local mapping is able to triangulate lines 
#define USE_LINE_MATCHING_BY_KNN 0       // just for testing how search-by-projection is better w.r.t. KNN-based search 

using namespace std;

namespace PLVS
{

int Tracking::skNumMinFeaturesStereoInitialization = 500; 
int Tracking::skNumMinFeaturesRGBDInitialization   = 500;
const int Tracking::kNumMinFeaturesMonoInitialization   = 100;

const int Tracking::kMaxNumOfKeyframesInLocalMap = 80; // originally it was 80
const int Tracking::kNumBestCovisibilityKeyFrames = 10; // originally it was 10

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
    LOST=3,
    RELOCALIZE_IN_LOADED_MAP=4,
    NUM_TRACKING_STATES_1
};*/
std::vector<std::string> Tracking::vTrackingStateStrings = {
    "SYSTEM_NOT_READY",
    "NO_IMAGES_YET",
    "NOT_INITIALIZED",
    "OK",
    "LOST",
    "RELOCALIZE_IN_LOADED_MAP"
}; // must be kept in sync with eTrackingState

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0), mEnableDepthFilter(false)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

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

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;
    
    //
    cout << endl  << "Features Parameters: " << endl;    
    bool bBalanceTotalNumFeatures = static_cast<int> (Utils::GetParam(fSettings, "Features.balanceTotalNumFeatures", 0)) != 0;   

    int nFeatures = Utils::GetParam(fSettings, "ORBextractor.nFeatures", 1000, false);    
    
    mbLineTrackerOn = static_cast<int> (Utils::GetParam(fSettings, "Line.on", 0, false)) != 0;    
    int numLineFeatures = Utils::GetParam(fSettings, "Line.nfeatures", 100, false);     
    if( bBalanceTotalNumFeatures && mbLineTrackerOn )
    {
        nFeatures = std::max( 500, nFeatures - 2 * numLineFeatures);
        std::cout << "balancing num ORB features: " << nFeatures << std::endl; 
    }

    // Load ORB parameters

    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

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
                MapObjectPtr pMapObject = MapObjectNewPtr(pMap, imgObject, mK, mDistCoef);
                
                pMapObject->InitFeatures(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
                mpMap->AddMapObject(pMapObject);
                numImgsOpened++;
            }
            else
            {
                std::cerr << "could not open object image file: " << imgFilenames[ii] << std::endl; 
            }
        }
        mbObjectTrackerOn = numImgsOpened > 0;
        
        mvpLocalMapObjects = mpMap->GetAllMapObjects();
        mpMap->SetReferenceMapObjects(mvpLocalMapObjects);
    }
    
    
    // ---- ---- ---- 
    
    if(mbLineTrackerOn)
    {
        cout  << endl << "Using Line tracking" << endl; 
        mpLineExtractorLeft =  std::make_shared<LineExtractor>(numLineFeatures, lsdOptions); 
        
        if(sensor==System::STEREO)
            mpLineExtractorRight =  std::make_shared<LineExtractor>(numLineFeatures, lsdOptions);        
    }
    
    // ---- ---- ---- 

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

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

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    mImgGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
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

    mCurrentFrame = Frame(mImGray,mImgGrayRight,timestamp,mpLineExtractorLeft,mpLineExtractorRight,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();
    
    TOCKTRACK("Track");
    
    SENDALLTRACK;    

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
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
        
    mCurrentFrame = Frame(mImGray,mImDepth,timestamp,mpLineExtractorLeft,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
   
    Track();
    
    TOCKTRACK("Track");
    
    SENDALLTRACK;
    
    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
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

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    
    if(mState==RELOCALIZE_IN_LOADED_MAP)
    {
        InitForRelocalizationInMap();
    }

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization(); /// < TODO: integrate line initialization ?

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            /// < SLAM Mode: Local Mapping is active
            
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame(); /// < OKL

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame(); /// < OKL
                }
                else
                {
                    bOK = TrackWithMotionModel();  /// < OKL
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame(); /// < OKL
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            /// < Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints and MapLines in the map

                    if(!mVelocity.empty())
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
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        if(mbLineTrackerOn)
                        {
                            vpMLsMM = mCurrentFrame.mvpMapLines;                        
                            vbOutLinesMM = mCurrentFrame.mvbLineOutlier;
                        }
                        TcwMM = mCurrentFrame.mTcw.clone();
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
        

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap(); /// < OKL
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
            const std::vector<MapObjectPtr > &vpMObjs = mpMap->GetAllMapObjects();  // check all objects in the map          
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
            mState = OK;
        else
            mState = LOST;

        // Update drawer
        mpFrameDrawer->Update(this); /// < OKL

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            /// < Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

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

            /// < Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
            {
                CreateNewKeyFrame();  /// < OKL
                
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
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, resetting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
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


void Tracking::StereoInitialization()
{
    const int numMinFeaturesForInit = (mSensor==System::RGBD)? skNumMinFeaturesRGBDInitialization : skNumMinFeaturesStereoInitialization;
    
    if( (mCurrentFrame.N + mCurrentFrame.Nlines) > numMinFeaturesForInit)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        //KeyFramePtr pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB
        KeyFramePtr pKFini = KeyFrameNewPtr(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);
        
        PushKeyFrameInPointCloudMapping(pKFini);

        // Create MapPoints and associate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            const float& z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                
                //MapPointPtr pNewMP = new MapPoint(x3D,pKFini,mpMap);
                MapPointPtr pNewMP = MapPointNewPtr(x3D,pKFini,mpMap);
                
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

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
                    cv::Mat xs3D, xe3D; // start and end points 
                    if(mCurrentFrame.UnprojectStereoLine(i,xs3D,xe3D))
                    {
                        //MapLinePtr pNewLine = new MapLine(xs3D,xe3D,mpMap,pKFini);
                        MapLinePtr pNewLine = MapLineNewPtr(xs3D,xe3D,mpMap,pKFini);
                        
                        pNewLine->AddObservation(pKFini,i);
                        pKFini->AddMapLine(pNewLine,i);
                        pNewLine->ComputeDistinctiveDescriptors();
                        pNewLine->UpdateNormalAndDepth();
                        mpMap->AddMapLine(pNewLine);

                        mCurrentFrame.mvpMapLines[i]=pNewLine;
                    }
                }
            }            
        }
#endif
        cout << "New map created with " << mpMap->MapPointsInMap() << " points and " << mpMap->MapLinesInMap() << " lines" << endl;
        
        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        if(mbLineTrackerOn)
            mvpLocalMapLines = mpMap->GetAllMapLines();
        
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        if(mbLineTrackerOn)
            mpMap->SetReferenceMapLines(mvpLocalMapLines);
        
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{
    /// < TODO: add support for line segments init 
    
    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>kNumMinFeaturesMonoInitialization)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=kNumMinFeaturesMonoInitialization)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
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
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    //KeyFramePtr pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFramePtr pKFini = KeyFrameNewPtr(mInitialFrame,mpMap,mpKeyFrameDB);
    //KeyFramePtr pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
    KeyFramePtr pKFcur = KeyFrameNewPtr(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and associate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        //MapPointPtr pMP = new MapPoint(worldPos,pKFcur,mpMap);
        MapPointPtr pMP = MapPointNewPtr(worldPos,pKFcur,mpMap);

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
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, resetting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
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

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::InitForRelocalizationInMap()
{
    // Set Frame pose to the origin
    mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F)); 

    std::vector<KeyFramePtr> keyframes = mpMap->GetAllKeyFrames();

    KeyFramePtr pKFini = keyframes[0];
    //mpLocalMapper->InsertKeyFrame(pKFini);

    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;

    //mvpLocalKeyFrames.push_back(pKFini);

    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    if(mbLineTrackerOn)
        mvpLocalMapLines = mpMap->GetAllMapLines();

    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    if(mbLineTrackerOn)
        mpMap->SetReferenceMapLines(mvpLocalMapLines);

    //mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);    
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

//    if(nmatches<15)
//        return false;
    

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
    mCurrentFrame.SetPose(mLastFrame.mTcw);
    
    // Pose optimization  
    
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard points outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPointPtr>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
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

    if(  nmatchesMapFeatures < 10 )
    {
        std::cout << "Tracking::TrackReferenceKeyFrame() - relying on " << nmatchesMap << " map point matches and " << nmatchesMapLines << " map line matches " << std::endl; 
        std::cout << "Tracking::TrackReferenceKeyFrame() - FAILURE - (nmatchesMap + mnLineTrackWeigth*nmatchesMapLines)<10: " << nmatchesMapFeatures << std::endl;
        return false;
    }
    
    return ( nmatchesMapFeatures >= 10 );
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFramePtr pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
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
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            
            //MapPointPtr pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);
            MapPointPtr pNewMP = MapPointNewPtr(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
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
        cv::Mat Tlr = mlRelativeFramePoses.back();

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
            cv::Mat x3DStart,x3DEnd;
            if(mLastFrame.UnprojectStereoLine(i,x3DStart,x3DEnd))
            {
                //MapLinePtr pNewML = new MapLine(x3DStart,x3DEnd,mpMap,&mLastFrame,i);
                MapLinePtr pNewML = MapLineNewPtr(x3DStart,x3DEnd,mpMap,&mLastFrame,i);

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

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    

    // track points 
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPointPtr>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nPointmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    bool bLargerLineSearch = false; 

    // If few matches, uses a wider window search
    if(nPointmatches<20)
    {
        bLargerLineSearch = true; 

        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPointPtr>(NULL));
        nPointmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    //if(nPointmatches<20)
    //    return false;
        
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
        std::cout << "Tracking::TrackWithMotionModel() - FAILURE - nPointmatches + sknLineTrackWeigth*nLineMatches<20: " << nPointmatches + sknLineTrackWeigth*nLineMatches << std::endl; 
        return false;
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
                pMP->mbTrackInView = false;
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
    
    if( nMatchesMapFeatures <10 )
    {
        std::cout << "Tracking::TrackWithMotionModel() - FAILURE - (nMatchesMapPoints + sknLineTrackWeigth*nMatchesMapLines)<10: " << nMatchesMapPoints + sknLineTrackWeigth*nMatchesMapLines << std::endl; 
        return false;
    }
    
    return nMatchesMapFeatures>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points and map lines tracked in the frame.
    // We retrieve the local map and try to find matches to points and lines in the local map.

    UpdateLocalMap();

    SearchLocalPoints();
    
    if(mbLineTrackerOn)
        SearchLocalLines();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    
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
    if( ( mCurrentFrame.mnId < mnLastRelocFrameId+mMaxFrames ) && ( nFeatureMatchesInliers < 50 ) )
    {
        std::cout << "Tracking::TrackLocalMap() - FAILURE - mnMatchesInliers+mnLineTrackWeigth*sknLineTrackWeigth<50): " << nFeatureMatchesInliers << std::endl;        
        std::cout << "Tracking::TrackLocalMap() - FAILURE - relying on mnMatchesInliers: " << mnMatchesInliers << ", mnLineMatchesInliers: " << mnLineMatchesInliers << std::endl;
        return false;
    }

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

#define USE_LINES_FOR_NEW_KEYFRAMES_GEN 1
#define USE_FOV_CENTER_CRIERION 1

bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

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
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nPointsTrackedClose++;
                else
                    nPointsNonTrackedClose++;
            }
        }
        
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
    
//    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
//    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
//    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
//    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
//    //Condition 1c: tracking is weak
//    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
//    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
//    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);
    
    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (nFeaturesMatchesInliers<nRefFeaturesMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked features compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ( (nFeaturesMatchesInliers<nRefFeaturesMatches*thRefRatio || bNeedToInsertClose) && (nFeaturesMatchesInliers>15) );    

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
#if USE_FOV_CENTER_CRIERION    
    if(mbUseFovCentersKfGenCriterion)
    {
        const cv::Mat currentFovCenter = mCurrentFrame.GetFovCenter();
        const cv::Mat lastKfFovCenter = mpLastKeyFrame->GetFovCenter();
        float distanceFovCs = cv::norm(currentFovCenter - lastKfFovCenter);
        cFovCs = (distanceFovCs > skMaxDistFovCenters) && (nFeaturesMatchesInliers>15);
    }
#endif    
    
    //std::cout << "F: " << mCurrentFrame.mnId << ", nFeaturesMatchesInliers: " << nFeaturesMatchesInliers << ", KF: " << mpReferenceKF->mnFrameId << ", RefFeaturesMatches: " << nRefFeaturesMatches << std::endl; 
        
    //if( (c1a||c1b||c1c)&&c2 ) 
    if( ((c1a||c1b||c1c)&&c2) || cFovCs )
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                //if(mpLocalMapper->KeyframesInQueue()<3)
                if(mpLocalMapper->KeyframesInQueue()<maxNumOfKeyFramesInLocalMapperQueue)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    //KeyFramePtr pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
    KeyFramePtr pKF = KeyFrameNewPtr(mCurrentFrame,mpMap,mpKeyFrameDB);
    
    //std::cout << "new KF " << pKF->mnFrameId << std::endl; 

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are more than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
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
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    
                    //MapPointPtr pNewMP = new MapPoint(x3D,pKF,mpMap);
                    MapPointPtr pNewMP = MapPointNewPtr(x3D,pKF,mpMap);
                    
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
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

                    if(bCreateNew)
                    {
                        cv::Mat x3DStart,x3DEnd;
                        if(mCurrentFrame.UnprojectStereoLine(i,x3DStart,x3DEnd))
                        {
                            //MapLinePtr pNewML = new MapLine(x3DStart,x3DEnd,mpMap,pKF);
                            MapLinePtr pNewML = MapLineNewPtr(x3DStart,x3DEnd,mpMap,pKF);
                            
                            pNewML->AddObservation(pKF,i);
                            pKF->AddMapLine(pNewML,i);
                            pNewML->ComputeDistinctiveDescriptors();
                            pNewML->UpdateNormalAndDepth();
                            mpMap->AddMapLine(pNewML);

                            mCurrentFrame.mvpMapLines[i]=pNewML;
                            nLines++;
                        }
                    }
                    else
                    {
                        nLines++;
                    }

                    if(vDepthIdx[j].first>mThDepth && nLines>100)
                        break;
                }
            }
            
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);
    
    /// < Push new frame in PointCloudMapping
    PushKeyFrameInPointCloudMapping(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

// if not clear, push new frame in PointCloudMapping ;-)
void Tracking::PushKeyFrameInPointCloudMapping(KeyFramePtr& pKF)
{
    if(mpPointCloudMapping)
    {
        switch(mSensor) 
        {
            case System::RGBD:
                {
                PointCloudKeyFrame<PointCloudMapping::PointT>::Ptr pcKeyframe(new PointCloudKeyFrame<PointCloudMapping::PointT>(pKF, this->mImRGB, this->mImDepth));
                mpPointCloudMapping->InsertKeyFrame(pcKeyframe);
                }
                break;
            case System::STEREO:
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
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}


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
    }

    bool bLargerLineSearch = false; 

    if(mSensor==System::RGBD)
        bLargerLineSearch=true;

    // If the camera has been relocalised recently, perform a coarser search
    if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
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
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    
    if(mbLineTrackerOn)
        mpMap->SetReferenceMapLines(mvpLocalMapLines);
    
    if(mbObjectTrackerOn)
        mpMap->SetReferenceMapObjects(mvpLocalMapObjects);    

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalFeatures();
    
}

void Tracking::UpdateLocalFeatures()
{
    mvpLocalMapPoints.clear();
    
    if(mbLineTrackerOn) mvpLocalMapLines.clear();
    if(mbObjectTrackerOn) mvpLocalMapObjects.clear();

    for(vector<KeyFramePtr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
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
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPointPtr pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFramePtr,size_t> observations = pMP->GetObservations();
                for(map<KeyFramePtr,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first] += 1;
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
                if(!pML->isBad())
                {
                    const map<KeyFramePtr,size_t> observations = pML->GetObservations();
                    for(map<KeyFramePtr,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
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
    
    // < TODO: add objects votes?

    if(keyframeCounter.empty())
        return;

    float max=0;
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

        mvpLocalKeyFrames.push_back(it->first);
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
        if( (pParent) ) //&& (!pParent->isBad()) )  
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
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
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFramePtr> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
    {
        std::cout << "Relocalized FAILURE - no condidate keyframes!" << std::endl; 
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

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
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    if( nCandidates>0 )
    {
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

                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

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
    }
    
    if(!bMatch)
    {
        std::cout << "Relocalization FAILURE!" << std::endl; 
        return false;
    }
    else
    {
        std::cout << "Relocalized" << std::endl; 
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    cout << "Tracking Resetting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Resetting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Resetting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Resetting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints, MapLines and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

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
}

void Tracking::SetCalibration(const float fx, const float fy, const float cx, const float cy, const cv::Mat& DistCoef, const float bf)
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
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace PLVS
