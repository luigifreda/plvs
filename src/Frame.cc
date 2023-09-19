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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "KeyFrame.h"
#include "MapLine.h"
#include "Utils.h"
#include "LineMatcher.h"
#include "Tracking.h"
#include "Geom2DUtils.h"
#include "Stopwatch.h"
#include <thread>
#include <limits>


#define LOG_ASSIGN_FEATURES 0

#define CHECK_RGBD_ENDPOINTS_DEPTH_CONSISTENCY 1
#define USE_PARALLEL_POINTS_LINES_EXTRACTION 1
#define USE_UNFILTERED_PYRAMID_FOR_LINES 1 // N.B.: AT PRESENT TIME, it's better not to use the filtered pyramid!
                                           //       This is because a Gaussian filter is added in keylines extraction when a pre-computed pyramid is set.
                                           //       In fact, the keypoint Gaussian filter is too strong for keyline extraction (it blurs the image too much!)

#define DISABLE_STATIC_LINE_TRIANGULATION 0  // set this to 1 to check how local mapping is able to triangulate lines without using any static stereo line triangulation

#if LOG_ASSIGN_FEATURES
#include "Logger.h"
static const string logFileName = "assign_feature.log";
static Logger logger(logFileName);
#endif

namespace PLVS
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
bool Frame::mbUseFovCentersKfGenCriterion = false;   
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;
float Frame::mfLineGridElementThetaInv, Frame::mfLineGridElementDInv;
float Frame::mnMaxDiag;

const std::vector<size_t> Frame::kEmptyVecSizet = std::vector<size_t>();

const float Frame::kDeltaTheta = 10 * M_PI/180.f;// [rad]   10
const float Frame::kDeltaD = 100; //[pixels]                100

const float Frame::kTgViewZAngleMin = tan(30. *M_PI/180.f);
const float Frame::kCosViewZAngleMax = cos(30. *M_PI/180.f);
const float Frame::kDeltaZForCheckingViewZAngleMin = 0.02; // [m]
const float Frame::kDeltaZThresholdForRefiningEndPointsDepths = 0.0; // [m]

const float Frame::kLinePointsMaxMisalignment = 0.03; // [m]
const int Frame::kDeltaSizeForSearchingDepthMinMax = 1;
const float Frame::kMinStereoLineOverlap = 2; // [pixels]  (for pure horizontal lines one has minimum = 1) 
const float Frame::kLineNormalsDotProdThreshold = 0.05; //0.03f; // this a percentage over unitary modulus
const float Frame::kMinVerticalLineSpan = 2; // [pixels] should be startY-endY>kMinVerticalLineSpan in order to avoid line too close to an epipolar plane (degeneracy in line triangulation)

const int Frame::kMaxInt = std::numeric_limits<int>::max(); 

float Frame::skMinLineLength3D = 0.01; // [m]

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mbfInv(frame.mbfInv), mb(frame.mb), mThDepth(frame.mThDepth), 
     N(frame.N), 
     mvKeys(frame.mvKeys), mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), 
     mvuRight(frame.mvuRight), mvDepth(frame.mvDepth), 
     mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),   // clone point descriptors 
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), 
     mnId(frame.mnId),
     Nlines(frame.Nlines),
     mvKeyLines(frame.mvKeyLines), mvKeyLinesRight(frame.mvKeyLinesRight), mvKeyLinesUn(frame.mvKeyLinesUn),
     mvuRightLineStart(frame.mvuRightLineStart),  mvDepthLineStart(frame.mvDepthLineStart), mvuRightLineEnd(frame.mvuRightLineEnd), mvDepthLineEnd(frame.mvDepthLineEnd),
     mLineDescriptors(frame.mLineDescriptors.clone()), mLineDescriptorsRight(frame.mLineDescriptorsRight.clone()), // clone line descriptors 
     mvpMapLines(frame.mvpMapLines), mvbLineOutlier(frame.mvbLineOutlier), mvuNumLinePosOptFailures(frame.mvuNumLinePosOptFailures), 
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mnLineScaleLevels(frame.mnLineScaleLevels),
     mfLineScaleFactor(frame.mfLineScaleFactor), mfLineLogScaleFactor(frame.mfLineLogScaleFactor),        
     mvLineScaleFactors(frame.mvLineScaleFactors), 
     mvLineLevelSigma2(frame.mvLineLevelSigma2), mvLineInvLevelSigma2(frame.mvLineInvLevelSigma2),
     mMedianDepth(frame.mMedianDepth)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];
    
    if(frame.mpLineExtractorLeft)
    {
        for(int i=0;i<LINE_D_GRID_COLS;i++)
            for(int j=0; j<LINE_THETA_GRID_ROWS; j++)
                mLineGrid[i][j]=frame.mLineGrid[i][j];
    }
    
    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

/// <  STEREO 
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, 
	std::shared_ptr<LineExtractor>& lineExtractorLeft, std::shared_ptr<LineExtractor>& lineExtractorRight, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpLineExtractorLeft(lineExtractorLeft),mpLineExtractorRight(lineExtractorRight),
     mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFramePtr>(NULL)), Nlines(0),
     mMedianDepth(KeyFrame::skFovCenterDistance)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
    
    if(mpLineExtractorLeft)
    {
        //mfLineSigma2    = mpLineExtractorLeft->GetSigmaScale2();
        //mfInvLineSigma2 = mpLineExtractorLeft->GetInvSigmaScale2();
        
        mnLineScaleLevels    = mpLineExtractorLeft->GetLevels();
        mfLineScaleFactor    = mpLineExtractorLeft->GetScaleFactor();    
        mfLineLogScaleFactor = log(mfLineScaleFactor);        
        mvLineScaleFactors   = mpLineExtractorLeft->GetScaleFactors();
        mvLineLevelSigma2    = mpLineExtractorLeft->GetScaleSigmaSquares();
        mvLineInvLevelSigma2 = mpLineExtractorLeft->GetInverseScaleSigmaSquares();
    }    
    
    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);
        
        mfLineGridElementThetaInv=static_cast<float>(LINE_THETA_GRID_ROWS)/static_cast<float>(LINE_THETA_SPAN);
        mfLineGridElementDInv=static_cast<float>(LINE_D_GRID_COLS)/(2.0f*mnMaxDiag);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx; 
    mbfInv = 1.f/mbf;

    if(mpLineExtractorLeft)
    {            
#ifndef USE_CUDA          
        if( Tracking::skUsePyramidPrecomputation )
        {
            // pre-compute Gaussian pyramid to be used for the extraction of both keypoints and keylines        
            std::thread threadGaussianLeft(&Frame::PrecomputeGaussianPyramid,this,0,imLeft);
            std::thread threadGaussianRight(&Frame::PrecomputeGaussianPyramid,this,1,imRight);
            threadGaussianLeft.join();
            threadGaussianRight.join();              
        }
#else 
        if( Tracking::skUsePyramidPrecomputation )
        {
            std::cout << IoColor::Yellow() << "Can't use pyramid precomputation when CUDA is active!" << std::endl;         
        }
#endif         
        
        // ORB extraction
        std::thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
        std::thread threadRight(&Frame::ExtractORB,this,1,imRight);        
        // Line extraction
        std::thread threadLinesLeft(&Frame::ExtractLSD,this,0,imLeft);
        std::thread threadLinesRight(&Frame::ExtractLSD,this,1,imRight);        
        
        threadLeft.join();
        threadRight.join();
        threadLinesLeft.join();     
        threadLinesRight.join();            
    } 
    else
    {
        // ORB extraction
        std::thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
        std::thread threadRight(&Frame::ExtractORB,this,1,imRight);             
        
        threadLeft.join();
        threadRight.join();          
    }
   

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPointPtr>(N,static_cast<MapPointPtr>(NULL));    
    mvbOutlier = vector<bool>(N,false);
    
    
    // LSD line segments extraction 
    if(mpLineExtractorLeft)
    {
                
        Nlines = mvKeyLines.size();
        
        if(mvKeyLines.empty())
        {
            std::cout << "frame " << mnId << " no lines dectected!" << std::endl; 
        }
        else
        {                        
            UndistortKeyLines();
            
            ComputeStereoLineMatches();
        }
                      
        mvpMapLines = vector<MapLinePtr>(Nlines,static_cast<MapLinePtr>(NULL));
        mvbLineOutlier = vector<bool>(Nlines,false);
        mvuNumLinePosOptFailures = vector<unsigned int>(Nlines,0);
    }    

    AssignFeaturesToGrid();
}

/// <  RGBD 
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, std::shared_ptr<LineExtractor>& lineExtractor, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpLineExtractorLeft(lineExtractor),mpLineExtractorRight(0),
     mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), Nlines(0),
     mMedianDepth(KeyFrame::skFovCenterDistance)   
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
    
    if(mpLineExtractorLeft)
    {
        //mfLineSigma2    = mpLineExtractorLeft->GetSigmaScale2();
        //mfInvLineSigma2 = mpLineExtractorLeft->GetInvSigmaScale2();
        
        mnLineScaleLevels    = mpLineExtractorLeft->GetLevels();
        mfLineScaleFactor    = mpLineExtractorLeft->GetScaleFactor();    
        mfLineLogScaleFactor = log(mfLineScaleFactor);        
        mvLineScaleFactors   = mpLineExtractorLeft->GetScaleFactors();
        mvLineLevelSigma2    = mpLineExtractorLeft->GetScaleSigmaSquares();
        mvLineInvLevelSigma2 = mpLineExtractorLeft->GetInverseScaleSigmaSquares();
    }
    
   
    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        mfLineGridElementThetaInv=static_cast<float>(LINE_THETA_GRID_ROWS)/static_cast<float>(LINE_THETA_SPAN);
        mfLineGridElementDInv=static_cast<float>(LINE_D_GRID_COLS)/(2.0f*mnMaxDiag);
        
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;
    mbfInv = 1.f/mbf;    
   
    // extraction of keypoints and keylines 
    
    TICKTRACK("ExtractFeatures");             
   
#ifndef USE_CUDA    
    if( Tracking::skUsePyramidPrecomputation )
    {
        // pre-compute Gaussian pyramid to be used for the extraction of both keypoints and keylines 
        TICKTRACK("PyramidPrecompute");         
        this->PrecomputeGaussianPyramid(0,imGray);
        TOCKTRACK("PyramidPrecompute");
    }
#else 
    if( Tracking::skUsePyramidPrecomputation )
    {
        std::cout << IoColor::Yellow() << "Can't use pyramid precomputation when CUDA is active!" << std::endl;         
    }
#endif     
    
#if USE_PARALLEL_POINTS_LINES_EXTRACTION    
           
    if(mpLineExtractorLeft)
    { 
        TICKTRACK("ExtractKeyPoints*");                  
        std::thread threadPoints(&Frame::ExtractORB,this,0,imGray);  
        TICKTRACK("ExtractKeyLines-");          
        std::thread threadLines(&Frame::ExtractLSD,this,0,imGray); 
        threadPoints.join();   
        TOCKTRACK("ExtractKeyPoints*");           
        threadLines.join();               
        TOCKTRACK("ExtractKeyLines-");           
    } 
    else
    {       
        TOCKTRACK("ExtractKeyPoints*");         
        // ORB extraction
        ExtractORB(0,imGray); 
        TOCKTRACK("ExtractKeyLines-");            
    }
    
    
#else

    TICKTRACK("ExtractKeyPoints*");       
    // ORB extraction
    ExtractORB(0,imGray);
    TOCKTRACK("ExtractKeyPoints*");    
    
    if(mpLineExtractorLeft)
    {        
        TICKTRACK("ExtractKeyLines-");                             
        ExtractLSD(0, imGray);     
        TOCKTRACK("ExtractKeyLines-");        
    }    
    
#endif 
    
    TOCKTRACK("ExtractFeatures");         
    
    N = mvKeys.size();

    if(mvKeys.empty()) 
        return;
    
    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPointPtr>(N,static_cast<MapPointPtr>(NULL));
    mvbOutlier = vector<bool>(N,false);
    
    
    // LSD line segments extraction 
    if(mpLineExtractorLeft)
    {        
        Nlines = mvKeyLines.size();
        
        if(mvKeyLines.empty())
        {
            std::cout << "frame " << mnId << " no lines dectected!" << std::endl; 
        }
        else
        {
            UndistortKeyLines();
            
            //TICKTRACK("ComputeStereoLinesFromRGBD");              
            ComputeStereoLinesFromRGBD(imDepth);
            //TOCKTRACK("ComputeStereoLinesFromRGBD");              

        }
                      
        mvpMapLines = vector<MapLinePtr>(Nlines,static_cast<MapLinePtr>(NULL));
        mvbLineOutlier = vector<bool>(Nlines,false);
        mvuNumLinePosOptFailures = vector<unsigned int>(Nlines,0);
    }

    AssignFeaturesToGrid();
}

/// < MONOCULAR 
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), Nlines(0),
     mMedianDepth(KeyFrame::skFovCenterDistance)   
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPointPtr>(N,static_cast<MapPointPtr>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        mfLineGridElementThetaInv=static_cast<float>(LINE_THETA_GRID_ROWS)/static_cast<float>(LINE_THETA_SPAN);
        mfLineGridElementDInv=static_cast<float>(LINE_D_GRID_COLS)/(2.0f*mnMaxDiag);
        
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;
    mbfInv = 1.f/mbf;

    AssignFeaturesToGrid();

    if(mpLineExtractorLeft)
    {
        mvuRightLineStart = vector<float>(Nlines,-1);
        mvDepthLineStart  = vector<float>(Nlines,-1);
        mvuRightLineEnd   = vector<float>(Nlines,-1);
        mvDepthLineEnd    = vector<float>(Nlines,-1);
    } 
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
    
    if(mpLineExtractorLeft)
    {
        int nReserve = 0.5f*Nlines/(LINE_D_GRID_COLS*LINE_THETA_GRID_ROWS);
        for(unsigned int i=0; i<LINE_D_GRID_COLS;i++)
            for (unsigned int j=0; j<LINE_THETA_GRID_ROWS;j++)
                mLineGrid[i][j].reserve(nReserve);

        for(int i=0;i<Nlines;i++)
        {
            const cv::line_descriptor_c::KeyLine& kl = mvKeyLinesUn[i];

            int nGridPosX, nGridPosY;
            if(PosLineInGrid(kl,nGridPosX,nGridPosY))
            {
                mLineGrid[nGridPosX][nGridPosY].push_back(i);
            }
        }
    
#if LOG_ASSIGN_FEATURES
        logger << "================================================================" << std::endl;
        logger << "frame: " << mnId << std::endl; 
        for(int nGridPosX=0; nGridPosX < LINE_D_GRID_COLS; nGridPosX++)
        for(int nGridPosY=0; nGridPosY < LINE_THETA_GRID_ROWS; nGridPosY++)
        {
            std::vector<std::size_t>& vec = mLineGrid[nGridPosX][nGridPosY];
            for(int kk=0; kk<vec.size(); kk++)
            {
                const cv::line_descriptor_c::KeyLine& kl = mvKeyLinesUn[vec[kk]];
                const float &xs = kl.startPointX;
                const float &ys = kl.startPointY;
                const float &xe = kl.endPointX; 
                const float &ye = kl.endPointY;

                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentation(xs, ys, xe, ye, lineRepresentation);
    
                logger << "line " << vec[kk] << ", theta: " << lineRepresentation.theta*180/M_PI << ", d: " << lineRepresentation.d << ", posX: " << nGridPosX <<", posY: " << nGridPosY << std::endl;
            }
        }
#endif
    
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::ExtractLSD(int flag, const cv::Mat &im)
{
    if(flag==0)
    {
        if(mpLineExtractorLeft)
            (*mpLineExtractorLeft)(im,mvKeyLines,mLineDescriptors);
    }
    else
    {
        if(mpLineExtractorRight)
            (*mpLineExtractorRight)(im,mvKeyLinesRight,mLineDescriptorsRight);
    }
}

static void clonePyramid(const std::vector<cv::Mat>& imgsIn, std::vector<cv::Mat>& imgsClone)
{
    imgsClone.resize(imgsIn.size());
    for(size_t ii=0;ii<imgsIn.size();ii++) imgsClone[ii] = imgsIn[ii].clone();
}

void Frame::PrecomputeGaussianPyramid(int flag, const cv::Mat &im)
{
    if(flag==0)
    {
        mpORBextractorLeft->PrecomputeGaussianPyramid(im);
#ifndef USE_CUDA        
    #if USE_UNFILTERED_PYRAMID_FOR_LINES        
        if(mpLineExtractorLeft) mpLineExtractorLeft->SetGaussianPyramid(mpORBextractorLeft->mvImagePyramid, mpLineExtractorLeft->GetLevels(), mpORBextractorLeft->GetScaleFactor());
    #else         
        if(mpLineExtractorLeft) mpLineExtractorLeft->SetGaussianPyramid(mpORBextractorLeft->mvImagePyramidFiltered, mpLineExtractorLeft->GetLevels(), mpORBextractorLeft->GetScaleFactor());        
    #endif        
#endif 
    }
    else
    {
        mpORBextractorRight->PrecomputeGaussianPyramid(im);
#ifndef USE_CUDA          
    #if USE_UNFILTERED_PYRAMID_FOR_LINES            
        if(mpLineExtractorRight) mpLineExtractorRight->SetGaussianPyramid(mpORBextractorRight->mvImagePyramid, mpLineExtractorRight->GetLevels(), mpORBextractorRight->GetScaleFactor());
    #else          
        if(mpLineExtractorRight) mpLineExtractorRight->SetGaussianPyramid(mpORBextractorRight->mvImagePyramidFiltered, mpLineExtractorRight->GetLevels(), mpORBextractorRight->GetScaleFactor());        
    #endif        
#endif     
    }
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
    
    mFovCw = mOw + mRwc.col(2) * mMedianDepth;    
}

bool Frame::isInFrustum(MapPointPtr& pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY = Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}


bool Frame::isInFrustum(MapLinePtr& pML, float viewingCosLimit)
{    
    pML->mbTrackInView = false;
    pML->mTrackProjStartX = -1; pML->mTrackProjStartY = -1; 
    pML->mTrackProjEndX = -1; pML->mTrackProjEndY = -1;
    pML->mTrackProjMiddleX = -1; pML->mTrackProjMiddleY = -1; 
    
    // 3D in absolute coordinates
    //const cv::Mat p3DStart = pML->GetWorldPosStart(); 
    //const cv::Mat p3DEnd   = pML->GetWorldPosEnd();
    cv::Mat p3DStart, p3DEnd;
    pML->GetWorldEndPoints(p3DStart, p3DEnd);
    
    // consider the middle point  
    const cv::Mat p3DMiddle = 0.5*(p3DStart+p3DEnd);

    // 3D in camera coordinates
    const cv::Mat p3DMc = mRcw*p3DMiddle+mtcw;
    const float &pMcX = p3DMc.at<float>(0);
    const float &pMcY = p3DMc.at<float>(1);
    const float &pMcZ = p3DMc.at<float>(2);

    // Check positive depth
    if(pMcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invMz = 1.0f/pMcZ;
    pML->mTrackProjMiddleX = fx*pMcX*invMz+cx;
    pML->mTrackProjMiddleY = fy*pMcY*invMz+cy;
    pML->mTrackProjMiddleXR = pML->mTrackProjMiddleX - mbf*invMz;    

    const float& u = pML->mTrackProjMiddleX;
    const float& v = pML->mTrackProjMiddleY;
    
    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;    
    
    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();
    const cv::Mat PO = p3DMiddle-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pML->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;
    
    // Now compute other data used by the tracking
    
    // 3D in camera coordinates
    const cv::Mat p3DSc = mRcw*p3DStart+mtcw;
    const float &p3DScX = p3DSc.at<float>(0);
    const float &p3DScY = p3DSc.at<float>(1);
    const float &p3DScZ = p3DSc.at<float>(2);

    const cv::Mat p3DEc = mRcw*p3DEnd+mtcw;
    const float &p3DEcX = p3DEc.at<float>(0);
    const float &p3DEcY = p3DEc.at<float>(1);
    const float &p3DEcZ = p3DEc.at<float>(2);
   
    // Project in image 
    const float invSz = 1.0f/p3DScZ;
    pML->mTrackProjStartX = fx*p3DScX*invSz+cx;
    pML->mTrackProjStartY = fy*p3DScY*invSz+cy;
    pML->mTrackStartDepth = p3DScZ;    
    pML->mTrackProjStartXR = pML->mTrackProjStartX - mbf*invSz;
    
    // Project in image 
    const float invEz = 1.0f/p3DEcZ;
    pML->mTrackProjEndX = fx*p3DEcX*invEz+cx;
    pML->mTrackProjEndY = fy*p3DEcY*invEz+cy;
    pML->mTrackEndDepth = p3DEcZ;         
    pML->mTrackProjEndXR = pML->mTrackProjEndX - mbf*invEz;

    /*const float deltaDepth = fabs(p3DScZ-p3DEcZ);
    if(deltaDepth > kDeltaZForCheckingViewZAngleMin)
    {
        const float tgViewZAngle = sqrt( Utils::Pow2(p3DScX-p3DEcX) + Utils::Pow2(p3DScY-p3DEcY) )/deltaDepth;
        if(tgViewZAngle<kTgViewZAngleMin)
            return false;    
    }*/
    
    // Predict scale in the image
    const int nPredictedLevel = pML->PredictScale(dist,this);

    pML->mbTrackInView = true;
    pML->mnTrackScaleLevel= nPredictedLevel;
    pML->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    //const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);
    const bool bCheckLevels = (minLevel>0) || (maxLevel<kMaxInt);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t>& vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    //if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    /// < N.B: shouldn't we use floor here? NO! since the features are searched with radius
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

vector<size_t> Frame::GetLineFeaturesInArea(const float &xs, const float  &ys, const float &xe, const float  &ye, 
                                            const float& dtheta, const float& dd, const int minLevel, const int maxLevel) const
{    
    Line2DRepresentation lineRepresentation;
    Geom2DUtils::GetLine2dRepresentation(xs, ys, xe, ye, lineRepresentation);
    return GetLineFeaturesInArea(lineRepresentation,dtheta,dd,minLevel,maxLevel);
}

vector<size_t> Frame::GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, 
                                            const float& dtheta, const float& dd, const int minLevel, const int maxLevel) const
{       
    vector<size_t> vIndices;
    vIndices.reserve(Nlines);
  
    const bool bCheckLevels = (minLevel>0) || (maxLevel<kMaxInt);   
            
    const float thetaMin = lineRepresentation.theta - dtheta;
    const float thetaMax = lineRepresentation.theta + dtheta;
        
    if( fabs(thetaMin - thetaMax) > M_PI )
    {
        std::cout << "Frame::GetLineFeaturesInArea() - ERROR - you are searching over the full theta interval!" << std::endl; 
        quick_exit(-1);
        return vIndices; 
    }      
    
    const float dMin = lineRepresentation.d - dd;
    const float dMax = lineRepresentation.d + dd;    
    
    GetLineFeaturesInArea(thetaMin, thetaMax, dMin, dMax, bCheckLevels, minLevel, maxLevel, vIndices);

    return vIndices;
}

void Frame::GetLineFeaturesInArea(const float thetaMin, const float thetaMax, const float dMin, const float dMax, const bool bCheckLevels, const int minLevel, const int maxLevel, vector<size_t>& vIndices) const
{                    
    //std::cout << "GetLineFeaturesInArea - thetaMin: " << thetaMin << ", thetaMax: " << thetaMax << ", dMin: " << dMin << ", dMax: " << dMax << std::endl; 
    
    if( thetaMin < -M_PI_2 )
    {
        // let's split the search interval in two intervals (we are searching over a manifold here)
        // 1) bring theta angles within [-pi/2, pi/2]
        // 2) if you wrap around +-pi/2 (i.e. add +-pi) then you have to invert the sign of d 
     
        //std::cout << "thetaMin: " << thetaMin << " < -M_PI_2" << std::endl; 
        //std::cout << "start split " << std::endl; 
        
        const float thetaMin1 = thetaMin + M_PI; 
        const float thetaMax1 = M_PI_2 - std::numeric_limits<float>::epsilon();
        const float dMin1 = -dMax; 
        const float dMax1 = -dMin; 
        GetLineFeaturesInArea(thetaMin1,thetaMax1,dMin1,dMax1,bCheckLevels,minLevel,maxLevel,vIndices);
        
        const float thetaMin2 = -M_PI_2 + std::numeric_limits<float>::epsilon(); 
        const float thetaMax2 = thetaMax;
        const float dMin2 = dMin; 
        const float dMax2 = dMax; 
        GetLineFeaturesInArea(thetaMin2,thetaMax2,dMin2,dMax2,bCheckLevels,minLevel,maxLevel,vIndices);
        
        //std::cout << "end split " << std::endl;        
        
        return; // < EXIT 
    }
    
    if( thetaMax > M_PI_2 )
    {
        // let's split the search interval in two intervals (we are searching over a manifold here)
        // 1) bring theta angles within [-pi/2, pi/2]
        // 2) if you wrap around +-pi/2 (i.e. add +-pi) then you have to invert the sign of d 
        
        //std::cout << "thetaMax: " << thetaMax <<  " > M_PI_2" << std::endl; 
        //std::cout << "start split " << std::endl;         
        
        const float thetaMin1 = -M_PI_2 + std::numeric_limits<float>::epsilon(); 
        const float thetaMax1 = thetaMax - M_PI;
        const float dMin1 = -dMax; 
        const float dMax1 = -dMin; 
        GetLineFeaturesInArea(thetaMin1,thetaMax1,dMin1,dMax1,bCheckLevels,minLevel,maxLevel,vIndices);
        
        const float thetaMin2 = thetaMin; 
        const float thetaMax2 = M_PI_2 - std::numeric_limits<float>::epsilon();
        const float dMin2 = dMin; 
        const float dMax2 = dMax; 
        GetLineFeaturesInArea(thetaMin2,thetaMax2,dMin2,dMax2,bCheckLevels,minLevel,maxLevel,vIndices);
        
        //std::cout << "end split " << std::endl;           
        
        return; // < EXIT         
        
    }    
    
    //const int nMinCellThetaRow = (int)floor((thetaMin -LINE_THETA_MIN)*mfLineGridElementThetaInv);
    //const int nMaxCellThetaRow = (int)floor((thetaMax -LINE_THETA_MIN)*mfLineGridElementThetaInv);
    
    const int nMinCellThetaRow = std::max(0,(int)floor((thetaMin -LINE_THETA_MIN)*mfLineGridElementThetaInv));
    if( nMinCellThetaRow >= LINE_THETA_GRID_ROWS)
    {
        return; 
    }    
    const int nMaxCellThetaRow = std::min(LINE_THETA_GRID_ROWS-1, (int)floor((thetaMax -LINE_THETA_MIN)*mfLineGridElementThetaInv));    
    if( nMaxCellThetaRow < 0)
    {
        return; 
    }
    
    const int nMinCellDCol = std::max(0,(int)floor((dMin + mnMaxDiag)*mfLineGridElementDInv));  // + mnMaxDiag = - mnMinDiag
    if( nMinCellDCol >= LINE_D_GRID_COLS)
    {
        return; 
    }
    const int nMaxCellDCol = std::min(LINE_D_GRID_COLS-1,(int)floor((dMax + mnMaxDiag)*mfLineGridElementDInv)); // + mnMaxDiag = - mnMinDiag
    if( nMaxCellDCol < 0)
    {
        return; 
    }
        
    for(int ix = nMinCellDCol; ix<=nMaxCellDCol; ix++)
    {
        for(int iy = nMinCellThetaRow; iy<=nMaxCellThetaRow; iy++)
        {
            //const int iyW = Utils::Modulus(iy,LINE_THETA_GRID_ROWS);
            const int iyW = iy;
            const vector<size_t>& vCell = mLineGrid[ix][iyW];
            if(vCell.empty())
                continue;

#if 0            
            vIndices.insert(vIndices.end(),vCell.begin(),vCell.end());
#else
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::line_descriptor_c::KeyLine &klUn = mvKeyLinesUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(klUn.octave<minLevel)
                        continue;
                    if(klUn.octave>maxLevel)
                        continue;
                }

                //const float distx = kpUn.pt.x-x;
                //const float disty = kpUn.pt.y-y;

                //if(fabs(distx)<r && fabs(disty)<r)
                //    vIndices.push_back(vCell[j]);
                
                vIndices.push_back(vCell[j]);                
            }     
#endif            
        }
    }
}

bool Frame::PosLineInGrid(const cv::line_descriptor_c::KeyLine &kl, int &posXcol, int &posYrow)
{
    const float &xs = kl.startPointX;
    const float &ys = kl.startPointY;
    const float &xe = kl.endPointX; 
    const float &ye = kl.endPointY;
    
    Line2DRepresentation lineRepresentation;
    Geom2DUtils::GetLine2dRepresentation(xs, ys, xe, ye, lineRepresentation);
            
    posYrow = round((lineRepresentation.theta-LINE_THETA_MIN)*mfLineGridElementThetaInv);  // row
    posXcol = round((lineRepresentation.d+mnMaxDiag)*mfLineGridElementDInv); // col     +mnMaxDiag=-mnMinDiag

    //Keyline coordinates are undistorted, which could cause to go out of the image
    if(posYrow<0 || posYrow>=LINE_THETA_GRID_ROWS || posXcol<0 || posXcol>=LINE_D_GRID_COLS)
        return false;

    return true;
}
    

void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

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


void Frame::UndistortKeyLines()
{
    if(Nlines==0) return; 
    
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeyLinesUn = mvKeyLines;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(2*Nlines,2,CV_32F);
    for(int i=0, j=0; i<Nlines; i++, j+=2)
    {
        mat.at<float>(j,0)  = mvKeyLines[i].startPointX;
        mat.at<float>(j,1)  = mvKeyLines[i].startPointY;
        mat.at<float>(j+1,0)= mvKeyLines[i].endPointX;
        mat.at<float>(j+1,1)= mvKeyLines[i].endPointY;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeyLinesUn.resize(Nlines);
    for(int i=0, j=0; i<Nlines; i++, j+=2)
    {
        cv::line_descriptor_c::KeyLine kl = mvKeyLines[i];
        kl.startPointX = mat.at<float>(j,0);
        kl.startPointY = mat.at<float>(j,1);
        kl.endPointX   = mat.at<float>(j+1,0);
        kl.endPointY   = mat.at<float>(j+1,1);
        mvKeyLinesUn[i]=kl;
        //std::cout << "kl[" << i <<  "]: " << kl.startPointX <<", "<< kl.startPointY <<"  -  " << kl.endPointX << ", " << kl.endPointY << std::endl;  
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
    mnMaxDiag = sqrt( pow(mnMaxX-mnMinX,2) + pow(mnMaxY-mnMinY,2) );
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    mMedianDepth = KeyFrame::skFovCenterDistance;  
        
    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR]; // we are assuming images are already rectified 
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search (here we use the fact that uR = uL - fx*b/zL )
    const float minZ = mb; // baseline in meters 
    const float minD = 0;
    const float maxD = mbf/minZ; // here maxD = fx 

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            //cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
#ifdef USE_CUDA
            cv::cuda::GpuMat gMat = mpORBextractorLeft->mvImagePyramid[kpL.octave]
                                        .rowRange(scaledvL - w, scaledvL + w + 1)
                                        .colRange(scaleduL - w, scaleduL + w + 1);
            cv::Mat IL(gMat.rows, gMat.cols, gMat.type(), gMat.data, gMat.step);
#else
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave]
                             .rowRange(scaledvL - w, scaledvL + w + 1)
                             .colRange(scaleduL - w, scaleduL + w + 1);          
#endif         
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                //cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
#ifdef USE_CUDA
                cv::cuda::GpuMat gMat = mpORBextractorRight->mvImagePyramid[kpL.octave]
                        .rowRange(scaledvL - w, scaledvL + w + 1)
                        .colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                cv::Mat IR(gMat.rows, gMat.cols, gMat.type(), gMat.data, gMat.step);
#else
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave]
                        .rowRange(scaledvL - w, scaledvL + w + 1)
                        .colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
#endif
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];  // here we assume (x1,y1) = (-1, dist1)
            const float dist2 = vDists[L+bestincR];    // here we assume (x2,y2) = ( 0, dist2)
            const float dist3 = vDists[L+bestincR+1];  // here we assume (x3,y3) = ( 1, dist3)

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2)); // this is equivalent to computing of x0 = -b/2a

            if(deltaR<-1 || deltaR>1) // we are outside the parabola interpolation range 
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
    
    if(mbUseFovCentersKfGenCriterion)
    {
        mMedianDepth = ComputeSceneMedianDepth();
    }    
}

// we assume images are rectified => compute pixel overlap along y 
static double lineSegmentOverlapStereo( double ys1, double ye1, double ys2, double ye2  )
{
    double ymin1 = std::min(ys1, ye1);
    double yMax1 = std::max(ys1, ye1);
    
    double ymin2 = std::min(ys2, ye2);
    double yMax2 = std::max(ys2, ye2);

    double overlap = 0.;
    if ( (yMax2 < ymin1) || (ymin2 > yMax1) ) // lines do not intersect
    {
        overlap = 0.;
    }
    else
    {
        overlap = min(yMax1,yMax2) - max(ymin1,ymin2);
    }

    return overlap;
}

// here we assume images have been rectified 
void Frame::ComputeStereoLineMatches()
{    
    mvuRightLineStart = vector<float>(Nlines,-1);
    mvDepthLineStart  = vector<float>(Nlines,-1);
    
    mvuRightLineEnd   = vector<float>(Nlines,-1);
    mvDepthLineEnd    = vector<float>(Nlines,-1);

#if DISABLE_STATIC_LINE_TRIANGULATION
    return; // just for testing: disable static stereo triangulation  
#endif 

    //const int thDescriptorDist = (LineMatcher::TH_HIGH + LineMatcher::TH_LOW)/2;     
    const int thDescriptorDist = LineMatcher::TH_LOW_STEREO;    
    
    // Set limits for search (here we use the fact that uR = uL - fx*b/zL => disparity=fx*b/zL)
    const float minZ = mb; // baseline in meters 
    const float maxZ = std::min(mbf, Tracking::skLineStereoMaxDist); 
    const float minD = mbf/maxZ;  // minimum disparity 
    const float maxD = mbf/minZ; // maximum disparity, here maxD = fx 
    //std::cout << " stereo line matching - minZ: " << minZ << ", maxZ: " << maxZ << ", minD: " << minD << ", maxD: " << maxD << std::endl;     
    
    // For each left keyline search a match in the right image
    std::vector<cv::DMatch> vMatches;
    std::vector<bool> vValidMatches;    
    vMatches.reserve(Nlines);  
    vValidMatches.reserve(Nlines);
    
    int nLineMatches = 0;    
    LineMatcher lineMatcher(0.8);
    if( Nlines > 0 && mvKeyLinesRight.size()>0 ) 
    {
        nLineMatches = lineMatcher.SearchStereoMatchesByKnn(*this, vMatches, vValidMatches, thDescriptorDist);
        PLVS_ASSERT(vMatches.size()==vValidMatches.size(),"The two sizes must be equal");
    }
    else
    {
        return;
    }
    
    //std::cout << "matched " << nLineMatches << " lines " << std::endl; 
    
    int numStereolines = 0;    
    
    // for storing matched right lines 
    std::vector<bool> vFlagMatchedRight(mvKeyLinesRight.size(), false);
    
    // For each left keyline search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(Nlines);    
    
    for( size_t ii = 0; ii < vMatches.size(); ii++ )
    {                
        if(!vValidMatches[ii]) continue; 

        const cv::DMatch& match = vMatches[ii]; // get first match from knn search 
        
        const int& idxL = match.queryIdx;        
        const int& idxR = match.trainIdx;        
        
        // if already matched 
        if( vFlagMatchedRight[idxR]) continue;
        
        const cv::line_descriptor_c::KeyLine& kll = mvKeyLines[idxL];
        const cv::line_descriptor_c::KeyLine& klr = mvKeyLinesRight[idxR];    
        const float sigma = sqrt( mvLineLevelSigma2[kll.octave] );         
        
        // we assume the distances of the camera centers to a line are approximately equal => the lines should be matched in the same octave or close octaves
#if 0        
        if( kll.octave != klr.octave ) continue;
#else         
        if( klr.octave<(kll.octave-1) || klr.octave>(kll.octave+1) ) continue;
#endif             
        
        const float &vS = kll.startPointY;
        const float &uS = kll.startPointX;

        const float &vE = kll.endPointY;
        const float &uE = kll.endPointX;        
        
        const float dyl = fabs(kll.startPointY - kll.endPointY);
        const float dyr = fabs(klr.startPointY - klr.endPointY);

        // check line triangulation degeneracy 
        // we don't want lines that are too close to an epipolar plane (i.e. dy=0)
        const float minVerticalLineSpan = kMinVerticalLineSpan * sigma; 
        if( (dyl <= minVerticalLineSpan) || (dyr <= minVerticalLineSpan) ) continue; 
        
        const double overlap = lineSegmentOverlapStereo( kll.startPointY, kll.endPointY, klr.startPointY, klr.endPointY );    
        if( overlap <= kMinStereoLineOverlap*sigma ) continue; // we modulate the overlap threshold by sigma considering the detection uncertainty 
        
        // estimate the disparity of the line endpoints
        
        Eigen::Vector3d startL(kll.startPointX, kll.startPointY, 1.0);
        Eigen::Vector3d endL(kll.endPointX,   kll.endPointY, 1.0);
        Eigen::Vector3d ll = startL.cross(endL); 
        ll = ll / sqrt( ll(0)*ll(0) + ll(1)*ll(1) ); // now ll = [nx ny -d] with (nx^2 + ny^2) = 1   
        
        Eigen::Vector3d startR(klr.startPointX, klr.startPointY, 1.0);
        Eigen::Vector3d endR(klr.endPointX,   klr.endPointY, 1.0);
        Eigen::Vector3d lr = startR.cross(endR);     
        lr = lr / sqrt( lr(0)*lr(0) + lr(1)*lr(1) ); // now lr = [nx ny -d] with (nx^2 + ny^2) = 1       


        const float dotProductThreshold = kLineNormalsDotProdThreshold * sigma; // this a percentage over unitary modulus ( we modulate threshold by sigma)
        const float distThreshold = 2.f * sigma;  // 1.f * sigma // 1 pixel (we modulate threshold by sigma)
        
        // an alternative way to check the degeneracy 
        if( fabs( ll(0) ) < dotProductThreshold ) continue; // ll = [nx ny -d] with (nx^2 + ny^2) = 1         
        if( fabs( lr(0) ) < dotProductThreshold ) continue; // lr = [nx ny -d] with (nx^2 + ny^2) = 1 
        
        // detect if lines are equal => if yes the backprojected planes are parallel and we must discard the matched lines 
        if( Geom2DUtils::areLinesEqual(ll, lr, dotProductThreshold, distThreshold)) 
        {
            //std::cout << "detected equal lines[" << ii <<"] - ll " << ll << ", lr: " << lr <<  std::endl;
            continue;
        }

#if 0        
        if( fabs( lr(0) ) < 0.01 ) continue;
        
        startR << - (lr(2)+lr(1)*kll.startPointY )/lr(0) , kll.startPointY ,  1.0;
        endR << - (lr(2)+lr(1)*kll.endPointY   )/lr(0) , kll.endPointY ,    1.0;
        double disparity_s = kll.startPointX - startR(0);
        double disparity_e = kll.endPointX   - endR(0);     
                        
        if( disparity_s>=minD && disparity_s<=maxD && disparity_e>=minD && disparity_e<=maxD )
        {
            if(disparity_s<=0)
            {
                disparity_s=0.01;
            }
            if(disparity_e<=0)
            {
                disparity_e=0.01;
            }            

            mvuRightLineStart[idxL] = startR(0);
            mvDepthLineStart[idxL]  = mbf/disparity_s;
            mvuRightLineEnd[idxL] = endR(0);
            mvDepthLineEnd[idxL]  = mbf/disparity_e;
        }
        else
        {
            continue; 
        }
#else
     
        // < TODO: compute covariance and use chi square check ?
        
        const double disparity_s = lr.dot(startL)/lr(0);        
        const double disparity_e = lr.dot(endL)/lr(0);            
        
        if( disparity_s>=minD && disparity_s<=maxD && disparity_e>=minD && disparity_e<=maxD )
        {                     
            const double depth_s = mbf / disparity_s;           
            const double depth_e = mbf / disparity_e;               

            mvuRightLineStart[idxL] = startL(0) - disparity_s;
            mvDepthLineStart[idxL]  = depth_s;
            mvuRightLineEnd[idxL] = endL(0) - disparity_e;
            mvDepthLineEnd[idxL]  = depth_e;
        }
        else
        {
            continue;
        }
        
#endif
        
        float& dS = mvDepthLineStart[idxL];
        float& dE = mvDepthLineEnd[idxL];        
        if( dS > 0 && dE > 0) 
        {        
            // use undistorted coordinates here
            const float xS = (uS-cx)*dS*invfx;
            const float yS = (vS-cy)*dS*invfy;
            const float xE = (uE-cx)*dE*invfx;
            const float yE = (vE-cy)*dE*invfy;

            Eigen::Vector3d camRay(xS,yS,dS);
            camRay.normalize();
            
            Eigen::Vector3d lineES(xS-xE,yS-yE,dS-dE);  
            const double lineLength = lineES.norm();
            if(lineLength < Frame::skMinLineLength3D)
            {
                // force line as mono
                dS = dE = -1;  // set depths as not valid  
            }
            else
            {            
                lineES /= lineLength;
                const float cosViewAngle = fabs((float)camRay.dot(lineES));

                if(cosViewAngle>kCosViewZAngleMax)
                {                    
                    dS = dE = -1;  // set depths as not valid
                }
            }
        }   
        
        if( dS > 0 && dE > 0) 
        {        
            numStereolines++;
            vFlagMatchedRight[idxR] = true;
            vDistIdx.push_back(pair<int,int>(match.distance,idxL));            
        }
        else
        {
            mvuRightLineStart[idxL] = mvuRightLineEnd[idxL] = -1;
            mvDepthLineStart[idxL] = mvDepthLineEnd[idxL] = -1;            
        }
    }
    

#if 1    
    float median = -1;
    int numFiltered = 0;         
    if( vDistIdx.size() > 0 ) 
    {
        sort(vDistIdx.begin(),vDistIdx.end());
        median = vDistIdx[vDistIdx.size()/2].first;
        const float thDist = 1.5f*1.48f*median;    
        for(int i=vDistIdx.size()-1;i>=0;i--)
        {
            if(vDistIdx[i].first<thDist)
                break;
            else
            {
                numFiltered++;
                const int idx = vDistIdx[i].second;
                mvDepthLineStart[idx] = mvDepthLineEnd[idx] = -1;
                mvuRightLineStart[idx] = mvuRightLineEnd[idx] = -1;            
            }
        }
    }
    
    //std::cout << "num lines: "<< Nlines<< ", stereo lines %: " << 100.*numStereolines/Nlines << ", filtered: " << numFiltered << ", median dist: " << median << std::endl;    
#endif     
       
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);
    
    mMedianDepth = KeyFrame::skFovCenterDistance;   

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
        
    if(mbUseFovCentersKfGenCriterion)
    {
        mMedianDepth = ComputeSceneMedianDepth();
    }
}

static void computeLocalAverageDepth(const cv::Mat &imDepth, const int& u, const int& v, const int& deltau, const int& deltav, float& outDepth)
{
    outDepth = 0;
    
    //const float depth = imDepth.at<float>(v,u);
    //if (depth>0 && std::isfinite(depth))
    {
        float sum   = 0.0f;
        int count = 0;
        for (int du = -deltau; du <= deltau; du++)
        {
            for (int dv = -deltav; dv <= deltav; dv++)
            {
                const int otheru =  u + du;
                const int otherv =  v + dv;
                if ( (otheru >=0 ) && (otheru<imDepth.cols) && (otherv>=0) && (otherv<imDepth.rows) )
                {
                    const float& otherval = imDepth.at<float>(otherv, otheru);
                    if (std::isfinite(otherval) ) //&& fabs(val - otherval) < kDeltaDepth)
                    {
                        sum   += otherval;
                        count ++;
                    }
                }
            }
        }
        outDepth = sum / std::max(count,(int)1);
    }
}

static void computeLocalMinDepth(const cv::Mat &imDepth, const int& u, const int& v, const int& deltau, const int& deltav, float& outDepth)
{
    outDepth = 0;
    float min = std::numeric_limits<float>::max();
    
    for (int du = -deltau; du <= deltau; du++)
    {
        for (int dv = -deltav; dv <= deltav; dv++)
        {
            const int otheru =  u + du;
            const int otherv =  v + dv;
            if ( (otheru >=0 ) && (otheru<imDepth.cols) && (otherv>=0) && (otherv<imDepth.rows) )
            {
                const float& otherval = imDepth.at<float>(otherv, otheru);
                if( std::isfinite(otherval)  && (otherval > 0) )
                {
                    if(min > otherval) 
                        min = otherval;
                }
            }
        }
    }
    if(min < std::numeric_limits<float>::max())
        outDepth = min; 
 
}

static void computeLocalMinMaxDepth(const cv::Mat &imDepth, const int& u, const int& v, const int& deltau, const int& deltav, float& minDepth, float& maxDepth)
{
    minDepth = 0;
    maxDepth = 0;
    float min = std::numeric_limits<float>::max();
    float max = 0;
    
    for (int du = -deltau; du <= deltau; du++)
    {
        for (int dv = -deltav; dv <= deltav; dv++)
        {
            const int otheru =  u + du;
            const int otherv =  v + dv;
            if ( (otheru >=0 ) && (otheru<imDepth.cols) && (otherv>=0) && (otherv<imDepth.rows) )
            {
                const float& otherval = imDepth.at<float>(otherv, otheru);
                if( std::isfinite(otherval)  && (otherval > 0) )
                {
                    if(min > otherval)
                        min = otherval; 
                    if(max < otherval) 
                        max = otherval;
                }
            }
        }
    }
    
    if(min < std::numeric_limits<float>::max())
        minDepth = min;     
    if(max > 0)
        maxDepth = max; 
 
}


static void searchClosest(const cv::Mat &imDepth, const int& u, const int& v, const int& deltau, const int& deltav, const float& refDepth, float& outDepth)
{
    float minDepthDist = fabs(refDepth-outDepth);
    float closestDepth = outDepth; 
    
    for (int du = -deltau; du <= deltau; du++)
    {
        for (int dv = -deltav; dv <= deltav; dv++)
        {
            const int otheru =  u + du;
            const int otherv =  v + dv;
            if ( (otheru >=0 ) && (otheru<imDepth.cols) && (otherv>=0) && (otherv<imDepth.rows) )
            {
                const float& otherDepth = imDepth.at<float>(otherv, otheru);
                if( std::isfinite(otherDepth)  && (otherDepth > 0) ) 
                {
                    const float depthDist = fabs(refDepth-otherDepth);
                    if(minDepthDist > depthDist) 
                    {
                        closestDepth = otherDepth;
                        minDepthDist = depthDist;
                    }
                }
            }
        }
    }

    outDepth = closestDepth; 
 
}


static void searchClosestAlongLine(const cv::Mat &imDepth, const cv::Point& pt1, const cv::Point& pt2, const int delta, const float& refDepth, float& outDepth)
{
    cv::LineIterator it(imDepth, pt1, pt2, 8);
    cv::LineIterator it2 = it;

    float minDepthDist = fabs(refDepth-outDepth);
    float closestDepth = outDepth; 
    
    //cout << endl; 
    
    for(int i = 0; (i < it2.count)&& (i < delta); i++, ++it2)
    {
        float otherDepth = imDepth.at<float>(it2.pos());
        if( std::isfinite(otherDepth)  && (otherDepth > 0) ) 
        {
            //cout << "pos: " << it2.pos() <<  ", depth: " << otherDepth << endl;
            const float depthDist = fabs(refDepth-otherDepth);
            if(minDepthDist > depthDist) 
            {
                closestDepth = otherDepth;
                minDepthDist = depthDist;
            }
        }
    }
    
    outDepth = closestDepth; 
 
}

void Frame::ComputeStereoLinesFromRGBD(const cv::Mat &imDepth)
{
    //std::cout << "***********************************************" << std::endl; 
    
    mvuRightLineStart = vector<float>(Nlines,-1);
    mvDepthLineStart  = vector<float>(Nlines,-1);
    
    mvuRightLineEnd   = vector<float>(Nlines,-1);
    mvDepthLineEnd    = vector<float>(Nlines,-1);
    
#if DISABLE_STATIC_LINE_TRIANGULATION
    return; // just for testing: static stereo triangulation  
#endif 

    int numLinesWithMisalignment = 0; 
    int numStereolines = 0;

    for(int i=0; i<Nlines; i++)
    {
        const cv::line_descriptor_c::KeyLine &kl  = mvKeyLines[i];   // use distorted to get registered image depth 
        const cv::line_descriptor_c::KeyLine &klU = mvKeyLinesUn[i]; // use undistorted to unproject to 3D points 

        const float &vS = kl.startPointY;
        const float &uS = kl.startPointX;
        
        const float &vE = kl.endPointY;
        const float &uE = kl.endPointX;
        
        const float vM = 0.5 * (vS + vE);
        const float uM = 0.5 * (uS + uE);
        
        // undistorted 
        const float &vSU = klU.startPointY;
        const float &uSU = klU.startPointX;
        
        const float &vEU = klU.endPointY;
        const float &uEU = klU.endPointX;      
        
        const float vMU = 0.5 * (vSU + vEU);
        const float uMU = 0.5 * (uSU + uEU);        

#if !CHECK_RGBD_ENDPOINTS_DEPTH_CONSISTENCY   
        const float dS = imDepth.at<float>(vS,uS);
        const float dE = imDepth.at<float>(vE,uE);
#else
        float dS = 0, dE = 0; 
        float dSmax = 0, dEmax = 0;
        float dM = 0; 
        const int& delta = Frame::kDeltaSizeForSearchingDepthMinMax;
        
        //computeLocalAverageDepth(imDepth, uS, vS, delta, delta, dS);
        //computeLocalAverageDepth(imDepth, uE, vE, delta, delta, dE);
        
        //computeLocalMinDepth(imDepth, uS, vS, delta, delta, dS);
        computeLocalMinMaxDepth(imDepth, uS, vS, delta, delta, dS, dSmax); // now dS = dSmin
        
        //computeLocalMinDepth(imDepth, uE, vE, delta, delta, dE);
        computeLocalMinMaxDepth(imDepth, uE, vE, delta, delta, dE, dEmax); // now dE = dEmin
                
        /// < N.B.: ASSUMPTION: we assume it's fine to take the minimum depth at the middle point M (no foreground-background dilemma at the middle point)
        ///         ISSUE: The problem is that sometimes all the points on the line are veil points that belong to the background.
        computeLocalMinDepth(imDepth, uM, vM, delta, delta, dM);
        
#if 0
        // we privilege minima (objects in foreground)
        // check if we have to replace dEmin
        if(fabs(dM-dE) > fabs(dM-dEmax)+kDeltaZThresholdForRefiningEndPointsDepths)
        {
            //cout << "replacing depths - before: " << dS << ", " << dE << ", delta: " << fabs(dS-dE) << endl; 
            //cout << "line from: (" <<uS<<", "<<vS<<") to ("<<uE<<", "<<vE<<")"<<endl; 
            dE = dEmax; 
            //searchClosestAlongLine(imDepth, cv::Point(uE,vE), cv::Point(uS,vS), 5, dS/*ref*/, dE);
            //cout << "replacing depths - after: " << dS << ", " << dE << ", delta: " << fabs(dS-dE) << endl; 
        }
        // check if we have to replace dSmin
        if(fabs(dS-dM) > fabs(dSmax-dM)+kDeltaZThresholdForRefiningEndPointsDepths)
        {
            //cout << "replacing depths - before: " << dS << ", " << dE << ", delta: " << fabs(dS-dE) << endl; 
            //cout << "line from: (" <<uS<<", "<<vS<<") to ("<<uE<<", "<<vE<<")"<<endl; 
            dS = dSmax; 
            //searchClosestAlongLine(imDepth, cv::Point(uS,vS), cv::Point(uE,vE), 5, dE/*ref*/, dS);
            //cout << "replacing depths - after: " << dS << ", " << dE << ", delta: " << fabs(dS-dE) << endl; 
        }     
#endif 
        
        if(dS > 0 && dE > 0)
        {
            // use undistorted coordinates here
            float xS = (uSU-cx)*dS*invfx;
            float yS = (vSU-cy)*dS*invfy;
            float xE = (uEU-cx)*dE*invfx;
            float yE = (vEU-cy)*dE*invfy;       
            
            //std::cout << "line[" << i <<  "]: " << xS <<", "<< yS<<", "<< dS << "  -  " << xE << ", " << yE <<", "<< dE << std::endl;             
            //std::cout << "                    " << uSU <<", "<< vSU<<"  -  " << uEU << ", " << vEU << std::endl;             
            
            Eigen::Vector3d lineES(xS-xE,yS-yE,dS-dE);   
                
            // if we have a sound middle point then check end-points alignment with it 
            if(dM > 0)
            {
                // use undistorted coordinates here
                const float xM = (uMU-cx)*dM*invfx;
                const float yM = (vMU-cy)*dM*invfy;       
                
                Eigen::Vector3d lineMS(xS-xM,yS-yM,dS-dM);           
                
                Eigen::Vector3d lineEM(xM-xE,yM-yE,dM-dE);            
                                
                // since there is minimum line length check (when image lines are detected) we can assume lineES.norm() > 0
                float distM_SE = lineMS.cross(lineEM).norm()/lineES.norm();
                                    
                if(distM_SE > kLinePointsMaxMisalignment)
                {
                    //std::cout << "----" << std::endl;                     
                    //std::cout << "something wrong in line " << i << ", misalignment: " << distM_SE << std::endl; 
                    
                    // we have a misalignment: check which point is not ok and possibly replace it        
                    // check if we can replace E with Emax 
                    if(dEmax > 0)
                    {
                        const float scale = dEmax/dE; 
                        const float xEmax = xE * scale;
                        const float yEmax = yE * scale;
                        
                        Eigen::Vector3d lineEmaxM(xM-xEmax,yM-yEmax,dM-dEmax);
                        //lineMEmax.normalize();     
                        
                        Eigen::Vector3d lineEmaxS(xS-xEmax,yS-yEmax,dS-dEmax);   

                        // since there is minimum line length check (when lines are detected) we can assume lineES.norm() > 0
                        float distM_SEmax = lineMS.cross(lineEmaxM).norm()/lineEmaxS.norm();                        
  
                        if( (distM_SEmax < kLinePointsMaxMisalignment) && (distM_SEmax < distM_SE) )
                        {           
                            // replace E with Emax 
                            //std::cout << "replacing E with Emax " << std::endl; 
                            xE = xEmax;
                            yE = yEmax; 
                            dE = dEmax; 
                            //bPointFixed = true; 
                            
                            lineEM = lineEmaxM;
                            lineES = lineEmaxS;
                            
                            distM_SE = distM_SEmax;
                        }                        
                    }
                    // check if we can replace S with Smax
                    if(dSmax > 0)
                    {
                        const float scale = dSmax/dS; 
                        const float xSmax = xS * scale;
                        const float ySmax = yS * scale;
                        
                        Eigen::Vector3d lineMSmax(xSmax-xM,ySmax-yM,dSmax-dM);
                        //lineSmaxM.normalize();    
                        
                        Eigen::Vector3d lineESmax(xSmax-xE,ySmax-yE,dSmax-dE);                     

                        const float distM_SmaxE = lineMSmax.cross(lineEM).norm()/lineESmax.norm();     
                        if( (distM_SmaxE < kLinePointsMaxMisalignment) && (distM_SmaxE < distM_SE) )
                        {           
                            // replace S with Smax 
                            //std::cout << "replacing S with Smax " << std::endl; 
                            xS = xSmax;
                            yS = ySmax; 
                            dS = dSmax; 
                            
                            lineMS = lineMSmax;
                            lineES = lineESmax;                            
                            
                            distM_SE = distM_SmaxE;
                        }                        
                    }                                                            
                }
                
                // final check on the possible modifications 
                if(distM_SE > kLinePointsMaxMisalignment)
                {
                    //std::cout << "AGAIN something wrong in line " << i << ", misalignment: " << distM_SE << std::endl; 
                    dS = dE = -1;  // set depths as not valid   
                    numLinesWithMisalignment++;
                }
                                
            } // end if(dM > 0)

            if(lineES.norm() < Frame::skMinLineLength3D)
            {
                // force line as mono
                dS = dE = -1;  // set depths as not valid  
                //std::cout << "short line [" << i <<  "]" << std::endl; 
            }
            
            // check angle of view
            //const float deltaDepth = fabs(dS-dE);
            //if(deltaDepth > kDeltaZForCheckingViewZAngleMin)                       
            if( dS > 0 && dE > 0) 
            {
                // use undistorted coordinates here
                /*const float xS = (uSU-cx)*dS*invfx;
                const float yS = (vSU-cy)*dS*invfy;
                const float xE = (uEU-cx)*dE*invfx;
                const float yE = (vEU-cy)*dE*invfy;*/
                
                Eigen::Vector3d camRay(xS,yS,dS);
                //std::cout << "camRay[" << i <<  "]" << camRay << std::endl; 
                camRay.normalize();
                Eigen::Vector3d lineES(xS-xE,yS-yE,dS-dE);
                //std::cout << "lineES[" << i <<  "]" << lineES << std::endl;                 
                lineES.normalize();
                const float cosViewAngle = fabs((float)camRay.dot(lineES));
                
                if(cosViewAngle>kCosViewZAngleMax)
                {                    
                    dS = dE = -1;  // set depths as not valid 
                    //std::cout << "depth aligned line [" << i <<  "]" << std::endl; 
                }
            }
        } // end if(dS > 0 && dE > 0) 
#endif
        
        if( (dS>0) && std::isfinite(dS) && (dE>0) && std::isfinite(dE) )
        {
            mvDepthLineStart[i]  = dS;
            mvuRightLineStart[i] = klU.startPointX-mbf/dS;
            
            mvDepthLineEnd[i]  = dE;
            mvuRightLineEnd[i] = klU.endPointX-mbf/dE;
            
            numStereolines++;
        }
        /*else
        {
            std::cout << "non stereo line[" << i << "] dS: " << dS << ", dE: " << dE <<  std::endl; 
        }*/       
        
    } // end for(int i=0; i<Nlines; i++)
    
    //std::cout << "num lines: "<< Nlines<< ", stereo lines %: " << 100.*numStereolines/Nlines << ", misaligned lines %: " << 100.*numLinesWithMisalignment/Nlines << std::endl;
}

cv::Mat Frame::UnprojectStereo(const int& i)
{
    const float& z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

bool Frame::UnprojectStereoLine(const int& i, cv::Mat& p3DStart, cv::Mat& p3DEnd)
{
    bool res  = true; 
    const float& zS = mvDepthLineStart[i];
    const float& zE = mvDepthLineEnd[i];
    if( (zS>0) && (zE>0) )
    {
        const float uS = mvKeyLinesUn[i].startPointX;
        const float vS = mvKeyLinesUn[i].startPointY;
        const float xS = (uS-cx)*zS*invfx;
        const float yS = (vS-cy)*zS*invfy;
        cv::Mat xs3Dc = (cv::Mat_<float>(3,1) << xS, yS, zS);
        p3DStart = mRwc*xs3Dc+mOw;
        
        const float uE = mvKeyLinesUn[i].endPointX;
        const float vE = mvKeyLinesUn[i].endPointY;
        const float xE = (uE-cx)*zE*invfx;
        const float yE = (vE-cy)*zE*invfy;
        cv::Mat xe3Dc = (cv::Mat_<float>(3,1) << xE, yE, zE);
        p3DEnd = mRwc*xe3Dc+mOw;
        
        res = true; 
           
    }
    else
    {
        std::cout << "*****************************************************************" << std::endl; 
        std::cout << "Frame::UnprojectStereoLine() - WARNING unprojecting invalid line!" << std::endl; 
        std::cout << "*****************************************************************" << std::endl;
        p3DStart = cv::Mat();
        p3DEnd   = cv::Mat();
        
        res = false;
    }
    
    return res;    
}

float Frame::ComputeSceneMedianDepth(const int q)
{
    float res = KeyFrame::skFovCenterDistance;
    
    std::vector<float> vDepths;
    vDepths.reserve(N);

    for(size_t i=0; i<N; i++)
    {
        const float& d = mvDepth[i];
        if(d>0) vDepths.push_back(d);           
    }
    if(!vDepths.empty())
    {
        sort(vDepths.begin(),vDepths.end());
        res = vDepths[(vDepths.size()-1)/2];
    }    
    
    //std::cout << "median depth: " << res << std::endl; 
    
    return res; 
}

} //namespace PLVS
