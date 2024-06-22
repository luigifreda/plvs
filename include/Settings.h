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

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H


// Flag to activate the measurement of time in each process (track,localmap, place recognition).
//#define REGISTER_TIMES

#include "CameraModels/GeometricCamera.h"
#include "SettingsAddsOn.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

namespace PLVS2 {

    class System;

    //TODO: change to double instead of float

    class Settings {
    public:
        /*
         * Enum for the different camera types implemented
         */
        enum CameraType {
            PinHole = 0,
            Rectified = 1,
            KannalaBrandt = 2
        };

        /*
         * Delete default constructor
         */
        Settings() = delete;

        /*
         * Constructor from file
         */
        Settings(const std::string &configFile, const int& sensor);


        /*
         * Singleton method to create the settings
         */
        static Settings* create(const std::string &configFile, const int& sensor);
        /*
        * Singleton method to get the settings
        */
        static Settings* instance(){ return instance_; }

        /*
         * Ostream operator overloading to dump settings to the terminal
         */
        friend std::ostream &operator<<(std::ostream &output, const Settings &s);

        /*
         * Getter methods
         */
        CameraType cameraType() const {return cameraType_;}
        GeometricCamera* camera1() const {return calibration1_;}
        GeometricCamera* camera2() const {return calibration2_;}
        cv::Mat camera1DistortionCoef() {return cv::Mat(vPinHoleDistorsion1_.size(),1,CV_32F,vPinHoleDistorsion1_.data());}
        cv::Mat camera2DistortionCoef() {return cv::Mat(vPinHoleDistorsion2_.size(),1,CV_32F,vPinHoleDistorsion1_.data());}
        cv::Mat camera1FisheyeDistortionCoef() {return cv::Mat(vFisheyeDistorsion1_.size(),1,CV_32F,vFisheyeDistorsion1_.data());}
        cv::Mat camera2FisheyeDistortionCoef() {return cv::Mat(vFisheyeDistorsion2_.size(),1,CV_32F,vFisheyeDistorsion2_.data());}        

        Sophus::SE3f Tlr() const {return Tlr_;}
        float bf() const {return bf_;}
        float b() const {return b_;}
        float thDepth() const {return thDepth_;}

        bool needToUndistort() const {return bNeedToUndistort_;}

        cv::Size newImSize() const {return newImSize_;}
        float imageScale() const {return imageScale_;}
        float fps() const {return fps_;}
        bool rgb() const {return bRGB_;}
        bool needToResize() const {return bNeedToResize1_;}
        bool needToRectify() const {return bNeedToRectify_;}

        float noiseGyro() const {return noiseGyro_;}
        float noiseAcc() const {return noiseAcc_;}
        float gyroWalk() const {return gyroWalk_;}
        float accWalk() const {return accWalk_;}
        float imuFrequency() const {return imuFrequency_;}
        Sophus::SE3f Tbc() const {return Tbc_;}
        bool insertKFsWhenLost() const {return insertKFsWhenLost_;}

        float depthMapFactor() const {return depthMapFactor_;}

        int nFeatures() const {return nFeatures_;}
        int nLevels() const {return nLevels_;}
        float initThFAST() const {return initThFAST_;}
        float minThFAST() const {return minThFAST_;}
        float scaleFactor() const {return scaleFactor_;}

        const SettingsLines& linesSettings() const {return lines_;}

        float keyFrameSize() const {return keyFrameSize_;}
        float keyFrameLineWidth() const {return keyFrameLineWidth_;}
        float graphLineWidth() const {return graphLineWidth_;}
        float pointSize() const {return pointSize_;}
        float lineSize() const {return lineSize_;}        
        float objectLineSize() const {return objectLineSize_;}            
        float cameraSize() const {return cameraSize_;}
        float cameraLineWidth() const {return cameraLineWidth_;}
        float viewPointX() const {return viewPointX_;}
        float viewPointY() const {return viewPointY_;}
        float viewPointZ() const {return viewPointZ_;}
        float viewPointF() const {return viewPointF_;}
        float imageViewerScale() const {return imageViewerScale_;}

        std::string atlasLoadFile() const {return sLoadFrom_;}
        std::string atlasSaveFile() const {return sSaveto_;}

        float thFarPoints() const {return thFarPoints_;}

        const cv::Mat& M1l() const {return M1l_;}
        const cv::Mat& M2l() const {return M2l_;}
        const cv::Mat& M1r() const {return M1r_;}
        const cv::Mat& M2r() const {return M2r_;}

        const cv::Mat& P1Rect() const {return P1_;}
        const cv::Mat& P2Rect() const {return P2_;}        

        const cv::Mat& R_r1_u1() const { return R_r1_u1_; }
        const Sophus::SE3f& T_r1_u1() const { return T_r1_u1_; }

        void precomputeRectificationMaps(bool bUpdateCalibration=true);

    private:
        template<typename T>
        T readParameter(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required = true){
            cv::FileNode node = fSettings[name];
            if(node.empty()){
                if(required){
                    std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                    exit(-1);
                }
                else{
                    std::cerr << name << " optional parameter does not exist..." << std::endl;
                    found = false;
                    return T();
                }

            }
            else{
                found = true;
                return (T) node;
            }
        }

        void readCamera1(cv::FileStorage& fSettings);
        void readCamera2(cv::FileStorage& fSettings);
        void readImageInfo(cv::FileStorage& fSettings);
        void readIMU(cv::FileStorage& fSettings);
        void readRGBD(cv::FileStorage& fSettings);
        
        void readORB(cv::FileStorage& fSettings);
        void readLines(cv::FileStorage& fSettings);

        void readViewer(cv::FileStorage& fSettings);
        void readLoadAndSave(cv::FileStorage& fSettings);
        void readOtherParameters(cv::FileStorage& fSettings);

        int sensor_;
        CameraType cameraType_;     //Camera type

        /*
         * Visual stuff
         */
        GeometricCamera* calibration1_=nullptr, *calibration2_=nullptr;   //Camera calibration
        GeometricCamera* originalCalib1_=nullptr, *originalCalib2_=nullptr;
        std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;
        std::vector<float> vFisheyeDistorsion1_, vFisheyeDistorsion2_;        

        cv::Size originalImSize_, newImSize_;
        float imageScale_ = 1.0f; 
        float fps_;
        bool bRGB_;

        bool bNeedToUndistort_;
        bool bNeedToRectify_;
        bool bNeedToResize1_, bNeedToResize2_;

        float linearCameraFovScale_ = 1.0;

        Sophus::SE3f Tlr_;
        float thDepth_;
        float bf_, b_;

        /*
         * Rectification stuff
         */
        cv::Mat M1l_, M2l_;
        cv::Mat M1r_, M2r_;
        cv::Mat P1_, P2_, Q_;
        cv::Mat R_r1_u1_, R_r2_u2_;   
        Sophus::SE3f T_r1_u1_;

        /*
         * Inertial stuff
         */
        float noiseGyro_, noiseAcc_;
        float gyroWalk_, accWalk_;
        float imuFrequency_;
        Sophus::SE3f Tbc_;
        bool insertKFsWhenLost_;

        /*
         * RGBD stuff
         */
        float depthMapFactor_;

        /*
         * ORB stuff
         */
        int nFeatures_;
        float scaleFactor_;
        int nLevels_;
        int initThFAST_, minThFAST_;

        /*
         * Viewer stuff
         */
        float keyFrameSize_;
        float keyFrameLineWidth_;
        float graphLineWidth_;
        float pointSize_;
        float lineSize_;
        float objectLineSize_;
        float cameraSize_;
        float cameraLineWidth_;
        float viewPointX_, viewPointY_, viewPointZ_, viewPointF_;
        float imageViewerScale_;

        /*
         * Save & load maps
         */
        std::string sLoadFrom_, sSaveto_;

        /*
         * Other stuff
         */
        float thFarPoints_;


        /*
         * Lines stuff
         */
        SettingsLines lines_; 


        static Settings* instance_; //singleton

    };
}


#endif //ORB_SLAM3_SETTINGS_H
