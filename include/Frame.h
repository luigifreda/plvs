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

#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <memory>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
//#include "KeyFrame.h"
#include "ORBextractor.h"
#include "LineExtractor.h"
#include "Utils.h"
#include "Geom2DUtils.h"
#include "Pointers.h"

#include <opencv2/opencv.hpp>

namespace PLVS
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

#define LINE_THETA_SPAN M_PI // manifold in  [-pi/2, pi/2]
#define LINE_THETA_MIN  (-M_PI_2)
#define LINE_THETA_GRID_ROWS 36
#define LINE_D_GRID_COLS 160 // 80

//class MapPoint;
//class KeyFrame;
//class MapLine; 
//class MapObject;
//class LineExtractor; 

class Frame
{
public:
    
    static const float kDeltaTheta; 
    static const float kDeltaD;
    static const float kTgViewZAngleMin;
    static const float kCosViewZAngleMax;
    static const float kDeltaZForCheckingViewZAngleMin;
    static const float kDeltaZThresholdForRefiningEndPointsDepths;
    static const float kLinePointsMaxMisalignment; 
    static const int   kDeltaSizeForSearchingDepthMinMax; 
    static const float kMinStereoLineOverlap;    
    static const float kLineNormalsDotProdThreshold;
    static const float kMinVerticalLineSpan;
    static const int kMaxInt; 
        
public:

    static float skMinLineLength3D;  
    
public:
    
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, std::shared_ptr<LineExtractor>& lineExtractorLeft, std::shared_ptr<LineExtractor>& lineExtractorRight, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, std::shared_ptr<LineExtractor>& lineExtractor, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);
    
    // Extract LSD line segments on the image. 0 for left image and 1 for right image.
    void ExtractLSD(int flag, const cv::Mat &im);
    
    // Pre-compute Gaussian pyramid to be used for the extraction of both keypoints and keylines
    void PrecomputeGaussianPyramid(int flag, const cv::Mat &im);    

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }
    
    inline cv::Mat GetRotation(){
        return mRcw.clone();
    }
    inline cv::Mat GetTranslation(){
        return mtcw.clone();
    }
    
    inline cv::Mat GetFovCenter(){
        return mFovCw.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPointPtr& pMP, float viewingCosLimit);
    
    // Check if a MapLine is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapLinePtr& pML, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    
    bool PosLineInGrid(const cv::line_descriptor_c::KeyLine &kl, int &posX, int &posY);

    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=kMaxInt) const;
    
    std::vector<size_t> GetLineFeaturesInArea(const float &xs, const float  &ys, const float &xe, const float  &ye, 
                                              const float& dtheta = kDeltaTheta, const float& dd = kDeltaD, const int minLevel=-1, const int maxLevel=kMaxInt) const;
    std::vector<size_t> GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, 
                                              const float& dtheta = kDeltaTheta, const float& dd = kDeltaD, const int minLevel=-1, const int maxLevel=kMaxInt) const;
    
    void GetLineFeaturesInArea(const float thetaMin, const float thetaMax, const float dMin, const float dMax, const bool bCheckLevels, const int minLevel, const int maxLevel, vector<size_t>& vIndices) const;
    

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();
    // Search a match for each keyline in the left image to a keyline in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keyline is stored.
    void ComputeStereoLineMatches();
        
    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);
    // Associate a depth coordinate to a keyline if there is valid depth in the depthmap.    
    void ComputeStereoLinesFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int& i);
    
    // Backprojects a keyline (if stereo/depth info available) into 3D world coordinates.
    bool UnprojectStereoLine(const int& i, cv::Mat& p3DStart, cv::Mat& p3DEnd);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    
    // Line extractor. The right is used only in the stereo case. 
    std::shared_ptr<LineExtractor> mpLineExtractorLeft, mpLineExtractorRight; 

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;
    float mbfInv;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;
    
    /// < Key Points 
    
    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;
    
    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPointPtr> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    /// < Key Lines 
        
    // Number of KeyLines.
    int Nlines;
    
    // Vector of keylines (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::line_descriptor_c::KeyLine> mvKeyLines, mvKeyLinesRight;
    std::vector<cv::line_descriptor_c::KeyLine> mvKeyLinesUn;
    
    // LSD descriptor, each row associated to a keypoint.
    cv::Mat mLineDescriptors, mLineDescriptorsRight;
    
    // MapLines associated to keylines, NULL pointer if no association.
    std::vector<MapLinePtr> mvpMapLines;
    
    std::vector<MapObjectPtr > mvpMapObjects; 
    
    // Flag to identify outlier associations.
    std::vector<bool> mvbLineOutlier;
    std::vector<unsigned int> mvuNumLinePosOptFailures; // number of time the line is detected as outlier in pose optimization
    
    // Corresponding stereo coordinate and depth for each keyline.
    // "Monocular" keylines have a negative value.
    std::vector<float> mvuRightLineStart;
    std::vector<float> mvDepthLineStart;
    std::vector<float> mvuRightLineEnd;
    std::vector<float> mvDepthLineEnd;
    
    
    // Keylines are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfLineGridElementThetaInv;
    static float mfLineGridElementDInv;
    std::vector<std::size_t> mLineGrid[LINE_D_GRID_COLS][LINE_THETA_GRID_ROWS];
    
    /// < Other data
    
    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFramePtr mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;
    
    int mnLineScaleLevels;
    float mfLineScaleFactor;
    float mfLineLogScaleFactor;      
    vector<float> mvLineScaleFactors;
    vector<float> mvLineLevelSigma2;
    vector<float> mvLineInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;
    static float mnMaxDiag;

    static bool mbInitialComputations;
    
    static bool mbUseFovCentersKfGenCriterion;        
    
    float mMedianDepth;   
    
private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();
    void UndistortKeyLines();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();
    
    float ComputeSceneMedianDepth(const int q = 2);

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
    
    cv::Mat mFovCw; // FOV Center (along optical axis, at distance 0.5*perceptionRange from camera center)      
    
    static const std::vector<size_t> kEmptyVecSizet; 
};

}// namespace PLVS

#endif // FRAME_H
