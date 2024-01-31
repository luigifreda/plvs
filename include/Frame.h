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


#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "Thirdparty/Sophus/sophus/geometry.hpp"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include "Converter.h"
#include "Settings.h"

//#include "Config.h"
#include "LineExtractor.h"
#include "Geom2DUtils.h"
#include "Pointers.h"

#include <mutex>
#include <opencv2/core/mat.hpp>

#include "Eigen/Core"
#include "Thirdparty/Sophus/sophus/se3.hpp"

namespace PLVS2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

#define LINE_THETA_SPAN M_PI // manifold in  [-pi/2, pi/2]
#define LINE_THETA_MIN  (-M_PI_2)
#define LINE_THETA_GRID_ROWS 36
#define LINE_D_GRID_COLS 160 // 80

//class MapPoint;
//class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;
class LineExtractor; 

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
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, std::shared_ptr<LineExtractor>& lineExtractorLeft, std::shared_ptr<LineExtractor>& lineExtractorRight, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, std::shared_ptr<LineExtractor>& lineExtractorLeft, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Destructor
    // ~Frame();

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

    // Extract LSD line segments on the image. 0 for left image and 1 for right image.
    void ExtractLSD(int flag, const cv::Mat &im);
    
    // Pre-compute Gaussian pyramid to be used for the extraction of both keypoints and keylines
    void PrecomputeGaussianPyramid(int flag, const cv::Mat &im);    

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose. (Imu pose is not modified!)
    void SetPose(const Sophus::SE3<float> &Tcw);

    // Set IMU velocity
    void SetVelocity(Eigen::Vector3f Vw);

    Eigen::Vector3f GetVelocity() const;

    // Set IMU pose and velocity (implicitly changes camera pose)
    void SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb);

    Eigen::Matrix<float,3,1> GetImuPosition() const;
    Eigen::Matrix<float,3,3> GetImuRotation();
    Sophus::SE3<float> GetImuPose();

    Sophus::SE3f GetRelativePoseTrl();
    Sophus::SE3f GetRelativePoseTlr();
    Eigen::Matrix3f GetRelativePoseTlr_rotation();
    Eigen::Vector3f GetRelativePoseTlr_translation();

    void SetNewBias(const IMU::Bias &b);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPointPtr& pMP, float viewingCosLimit);
    
    // Check if a MapLine is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapLinePtr& pML, float viewingCosLimit);

    bool ProjectPointDistort(MapPointPtr pMP, cv::Point2f &kp, float &u, float &v);

    Eigen::Vector3f inRefCoordinates(Eigen::Vector3f pCw);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    
    bool PosLineInGrid(const cv::line_descriptor_c::KeyLine &kl, int &posX, int &posY);

    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=kMaxInt, const bool bRight = false) const;

    
    std::vector<size_t> GetLineFeaturesInArea(const float &xs, const float  &ys, const float &xe, const float  &ye, 
                                              const float& dtheta = kDeltaTheta, const float& dd = kDeltaD, const int minLevel=-1, const int maxLevel=kMaxInt, const bool bRight = false) const;
    std::vector<size_t> GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, 
                                              const float& dtheta = kDeltaTheta, const float& dd = kDeltaD, const int minLevel=-1, const int maxLevel=kMaxInt, const bool bRight = false) const;
    
    void GetLineFeaturesInArea(const float thetaMin, const float thetaMax, const float dMin, const float dMax, const bool bCheckLevels, const int minLevel, const int maxLevel, vector<size_t>& vIndices, const bool bRight = false) const;
    
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
    bool UnprojectStereo(const int &i, Eigen::Vector3f &x3D);

    // Backprojects a keyline (if stereo/depth info available) into 3D world coordinates.
    bool UnprojectStereoLine(const int& i, Eigen::Vector3f& p3DStart, Eigen::Vector3f& p3DEnd);

    ConstraintPoseImu* mpcpi;

    bool imuIsPreintegrated();
    void setIntegrated();

    bool isSet() const;

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline Eigen::Vector3f GetCameraCenter(){
        return mOw;
    }

    // Returns inverse of rotation
    inline Eigen::Matrix3f GetRotationInverse(){
        return mRwc;
    }

    inline Sophus::SE3<float> GetPose() const {
        //TODO: can the Frame pose be accsessed from several threads? should this be protected somehow?
        return mTcw;
    }

    inline Eigen::Matrix3f GetRwc() const {
        return mRwc;
    }

    inline Eigen::Matrix3f GetRcw() const {
        return mRcw;
    }    

    inline Eigen::Vector3f GetOw() const {
        return mOw;
    }

    inline bool HasPose() const {
        return mbHasPose;
    }

    inline bool HasVelocity() const {
        return mbHasVelocity;
    }

    Eigen::Vector3f GetFovCenter() const {
        return fovCw;
    }


private:
    //Sophus/Eigen migration
    Sophus::SE3<float> mTcw;
    Eigen::Matrix<float,3,3> mRwc;
    Eigen::Matrix<float,3,1> mOw;
    Eigen::Matrix<float,3,3> mRcw;
    Eigen::Matrix<float,3,1> mtcw;
    bool mbHasPose;

    //Rcw_ not necessary as Sophus has a method for extracting the rotation matrix: Tcw_.rotationMatrix()
    //tcw_ not necessary as Sophus has a method for extracting the translation vector: Tcw_.translation()
    //Twc_ not necessary as Sophus has a method for easily computing the inverse pose: Tcw_.inverse()

    Sophus::SE3<float> mTlr, mTrl;
    Eigen::Matrix<float,3,3> mRlr;
    Eigen::Vector3f mtlr;


    // IMU linear velocity
    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    Eigen::Vector3f fovCw; // FOV Center (along optical axis, at distance 0.5*perceptionRange from camera center)
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
    Eigen::Matrix3f mK_;
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
    std::vector<MapPoint*> mvpMapPoints;
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint. 
    // With fisheye cameras, mDescriptors contains both left and rigth descriptors, vertically concatenated.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    int mnCloseMPs;

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
    std::vector<cv::line_descriptor_c::KeyLine> mvKeyLinesUn, mvKeyLinesRightUn; // mvKeyLinesRightUn to be used only with fisheye cameras (i.e. pCamera2 != nullptr)
    
    // Line descriptor, each row associated to a keyline. 
    // With fisheye cameras, mLineDescriptors contains both left and rigth descriptors, vertically concatenated.
    cv::Mat mLineDescriptors, mLineDescriptorsRight;
    
    // MapLines associated to keylines, NULL pointer if no association.
    std::vector<MapLinePtr> mvpMapLines;
    
    std::vector<MapObjectPtr > mvpMapObjects; 
    
    // Flag to identify outlier associations.
    std::vector<bool> mvbLineOutlier;
    //int mnCloseMLs;
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

    IMU::Bias mPredBias;

    // IMU bias
    IMU::Bias mImuBias;

    // Imu calibration
    IMU::Calib mImuCalib;

    // Imu preintegration from last keyframe
    IMU::Preintegrated* mpImuPreintegrated;
    KeyFramePtr mpLastKeyFrame;

    // Pointer to previous frame
    Frame* mpPrevFrame;
    IMU::Preintegrated* mpImuPreintegratedFrame; // IMU preintegration from last frame

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

    //std::map<long unsigned int, cv::Point2f> mmProjectPoints;  // not used now
    //std::map<long unsigned int, cv::Point2f> mmMatchedInImage; // not used now
    
    typedef std::pair<cv::Point2f,cv::Point2f> LineEndPoints; 
    //std::map<long unsigned int, LineEndPoints > mmProjectLines;       // not used now
    //std::map<long unsigned int, LineEndPoints> mmMatchedLinesInImage; // not used now

    string mNameFile;

    int mnDataset;

#ifdef REGISTER_TIMES
    double mTimeORB_Ext;
    double mTimeStereoMatch;
#endif

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

    bool mbIsSet;

    bool mbImuPreintegrated;

    static const std::vector<size_t> kEmptyVecSizet; 
    std::mutex *mpMutexImu;

public:
    GeometricCamera *mpCamera=nullptr, *mpCamera2=nullptr;

    /// < Stereo fisheye keypoints information 
    //Number of KeyPoints extracted in the left and right images
    int Nleft = -1, Nright = -1;   
    // NOTE: with stereo fisheye cameras we have     
    //      Nleft = mvKeys.size();
    //      Nright = mvKeysRight.size();
    //      N = Nleft + Nright;
    // Number of Non Lapping Keypoints in mvKeys and mvKeysRight respectively (the first blocks mvKeys[0:monoLeft-1] and mvKeysRight[0:monoRight-1] are mono keypoints)
    int monoLeft, monoRight;

    //For stereo matching
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;
    
    // < Stereo fisheye keylines information 
    // Number of KeyPoints extracted in the left and right images
    int NlinesLeft = -1, NlinesRight = -1;   
    // NOTE: with stereo fisheye cameras we have     
    //      NlinesLeft = mvLineKeys.size();
    //      NlinesRight = mvLineKeysRight.size();
    //      Nlines = NlinesLeft + NlinesRight;
    // Number of Non Lapping Keylines in mvLineKeys and mvLineKeysRight respectively (the first blocks mvLineKeys[0:monoLinesLeft-1] and mvLineKeysRight[0:monoLinesRight-1] are mono keylines) 
    // TODO: not yet implemented
    int monoLinesLeft = 0, monoLinesRight = 0;

    //For stereo matching
    std::vector<int> mvLeftToRightLinesMatch, mvRightToLeftLinesMatch;
    

    //For stereo fisheye matching
    static cv::BFMatcher BFmatcher;

    //Triangulated stereo observations using as reference the left camera. These are
    //computed during ComputeStereoFishEyeMatches
    std::vector<Eigen::Vector3f> mvStereo3Dpoints;

    //Triangulated line stereo observations using as reference the left camera. These are
    //computed during ComputeStereoFishEyeLineMatches
    std::vector<Eigen::Vector3f> mvStereo3DLineStartPoints;
    std::vector<Eigen::Vector3f> mvStereo3DLineEndPoints;    

    //Grid of keypoints for the right image
    std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    //Grid of keylines for the right image
    std::vector<std::size_t> mLineGridRight[LINE_D_GRID_COLS][LINE_THETA_GRID_ROWS];

    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, 
          std::shared_ptr<LineExtractor>& lineExtractorLeft, std::shared_ptr<LineExtractor>& lineExtractorRight, 
	  ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, 
          cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, Sophus::SE3f& Tlr,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    //Stereo fisheye
    void ComputeStereoFishEyeMatches();
    void ComputeStereoFishEyeLineMatches();    

    bool isInFrustumChecks(MapPointPtr pMP, float viewingCosLimit, bool bRight = false);
    bool isInFrustumChecks(MapLinePtr pML, float viewingCosLimit, bool bRight = false);    

    Eigen::Vector3f UnprojectStereoFishEye(const int &i);
    bool UnprojectStereoLineFishEye(const int &i, Eigen::Vector3f& p3DStart, Eigen::Vector3f& p3DEnd);    

    cv::Mat imgLeft, imgRight;

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (Nleft != -1) ? Nleft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i] && !mvbOutlier[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        std::cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << std::endl;
    }

    Sophus::SE3<double> T_test;
};

}// namespace PLVS2

#endif // FRAME_H
