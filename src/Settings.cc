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

#include "Settings.h"

#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

#include "System.h"

#include "Utils.h"
#include "LineExtractor.h"
#include "Tracking.h"
#include "Optimizer.h"


#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace PLVS2 {

    Settings* Settings::instance_ = nullptr;

    template<>
    float Settings::readParameter<float>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0.0f;
            }
        }
        else if(!node.isReal()){
            std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.real();
        }
    }

    template<>
    int Settings::readParameter<int>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0;
            }
        }
        else if(!node.isInt()){
            std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.operator int();
        }
    }

    template<>
    string Settings::readParameter<string>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return string();
            }
        }
        else if(!node.isString()){
            std::cerr << name << " parameter must be a string, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.string();
        }
    }

    template<>
    cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return cv::Mat();
            }
        }
        else{
            found = true;
            return node.mat();
        }
    }

    Settings::Settings(const std::string &configFile, const int& sensor) :
    bNeedToUndistort_(false), bNeedToRectify_(false), bNeedToResize1_(false), bNeedToResize2_(false) {
        sensor_ = sensor;

        //Open settings file
        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
        if (!fSettings.isOpened()) {
            cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
            cerr << "Aborting..." << endl;

            exit(-1);
        }
        else{
            cout << "Loading settings from " << configFile << endl;
        }

        //Read first camera
        readCamera1(fSettings);
        cout << "\t-Loaded camera 1" << endl;

        //Read second camera if stereo (not rectified)
        if(sensor_ == System::STEREO || sensor_ == System::IMU_STEREO){
            readCamera2(fSettings);
            cout << "\t-Loaded camera 2" << endl;
        }

        //Read image info
        readImageInfo(fSettings);
        cout << "\t-Loaded image info" << endl;

        if(sensor_ == System::IMU_MONOCULAR || sensor_ == System::IMU_STEREO || sensor_ == System::IMU_RGBD){
            readIMU(fSettings);
            cout << "\t-Loaded IMU calibration" << endl;
        }

        if(sensor_ == System::RGBD || sensor_ == System::IMU_RGBD){
            readRGBD(fSettings);
            cout << "\t-Loaded RGB-D calibration" << endl;
        }

        readORB(fSettings);
        cout << "\t-Loaded ORB settings" << endl;
        readLines(fSettings);
        cout << "\t-Loaded Lines settings" << endl;        
        readViewer(fSettings);
        cout << "\t-Loaded viewer settings" << endl;
        readLoadAndSave(fSettings);
        cout << "\t-Loaded Atlas settings" << endl;
        readOtherParameters(fSettings);
        cout << "\t-Loaded misc parameters" << endl;

        if(bNeedToRectify_){
            precomputeRectificationMaps(true /*update calibration*/);
            cout << "\t-Computed rectification maps" << endl;
        }

        cout << "----------------------------------" << endl;
    }

    Settings* Settings::create(const std::string &configFile, const int& sensor)
    {
        if(!instance_)  instance_ = new Settings(configFile, sensor);
        return instance_;
    }

    void Settings::readCamera1(cv::FileStorage &fSettings) {
        bool found;

        //Read camera model
        string cameraModel = readParameter<string>(fSettings,"Camera.type",found);

        vector<float> vCalibration;
        if (cameraModel == "PinHole") {
            cameraType_ = PinHole;

            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera1.fx",found);
            float fy = readParameter<float>(fSettings,"Camera1.fy",found);
            float cx = readParameter<float>(fSettings,"Camera1.cx",found);
            float cy = readParameter<float>(fSettings,"Camera1.cy",found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            //Check if it is a distorted PinHole
            readParameter<float>(fSettings,"Camera1.k1",found,false);
            if(found){
                readParameter<float>(fSettings,"Camera1.k3",found,false);
                if(found){
                    vPinHoleDistorsion1_.resize(5);
                    vPinHoleDistorsion1_[4] = readParameter<float>(fSettings,"Camera1.k3",found);
                }
                else{
                    vPinHoleDistorsion1_.resize(4);
                }
                vPinHoleDistorsion1_[0] = readParameter<float>(fSettings,"Camera1.k1",found);
                vPinHoleDistorsion1_[1] = readParameter<float>(fSettings,"Camera1.k2",found);
                vPinHoleDistorsion1_[2] = readParameter<float>(fSettings,"Camera1.p1",found);
                vPinHoleDistorsion1_[3] = readParameter<float>(fSettings,"Camera1.p2",found);
            }

            //Check if we need to correct distortion from the images
            if((sensor_ == System::MONOCULAR || sensor_ == System::IMU_MONOCULAR) && vPinHoleDistorsion1_.size() != 0){
                bNeedToUndistort_ = true;
            }
        }
        else if(cameraModel == "Rectified"){
            cameraType_ = Rectified;

            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera1.fx",found);
            float fy = readParameter<float>(fSettings,"Camera1.fy",found);
            float cx = readParameter<float>(fSettings,"Camera1.cx",found);
            float cy = readParameter<float>(fSettings,"Camera1.cy",found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            //Rectified images are assumed to be ideal PinHole images (no distortion)
        }
        else if(cameraModel == "KannalaBrandt8"){
            cameraType_ = KannalaBrandt;
            bNeedToRectify_ = (bool)readParameter<int>(fSettings,"Camera.needRectification",found,false);  
            linearCameraFovScale_ = Utils::GetParam(fSettings, "Camera.fovScale", 0.7);

            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera1.fx",found);
            float fy = readParameter<float>(fSettings,"Camera1.fy",found);
            float cx = readParameter<float>(fSettings,"Camera1.cx",found);
            float cy = readParameter<float>(fSettings,"Camera1.cy",found);

            float k0 = readParameter<float>(fSettings,"Camera1.k1",found);
            float k1 = readParameter<float>(fSettings,"Camera1.k2",found);
            float k2 = readParameter<float>(fSettings,"Camera1.k3",found);
            float k3 = readParameter<float>(fSettings,"Camera1.k4",found);

            vFisheyeDistorsion1_ = {k0,k1,k2,k3}; 

            if(bNeedToRectify_){

                //Rectified images are assumed to be ideal PinHole images (no distortion)      
                vCalibration = {fx, fy, cx, cy};                    
                calibration1_ = new Pinhole(vCalibration); // the linearCameraFovScale_ is used below in the fisheye rectification process to get the adjusted P1
                originalCalib1_ = new KannalaBrandt8({fx,fy,cx,cy,k0,k1,k2,k3},linearCameraFovScale_);      

            } else {

                vCalibration = {fx,fy,cx,cy,k0,k1,k2,k3};

                calibration1_ = new KannalaBrandt8(vCalibration,linearCameraFovScale_);
                originalCalib1_ = new KannalaBrandt8(vCalibration,linearCameraFovScale_);                

                if(sensor_ == System::STEREO || sensor_ == System::IMU_STEREO){
                    int colBegin = readParameter<int>(fSettings,"Camera1.overlappingBegin",found);
                    int colEnd = readParameter<int>(fSettings,"Camera1.overlappingEnd",found);
                    vector<int> vOverlapping = {colBegin, colEnd};

                    static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea = vOverlapping;
                }                
            }  
        }
        else{
            cerr << "Error: " << cameraModel << " not known" << endl;
            exit(-1);
        }
    }

    void Settings::readCamera2(cv::FileStorage &fSettings) {
        bool found;
        vector<float> vCalibration;
        if (cameraType_ == PinHole) {
            bNeedToRectify_ = true;

            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera2.fx",found);
            float fy = readParameter<float>(fSettings,"Camera2.fy",found);
            float cx = readParameter<float>(fSettings,"Camera2.cx",found);
            float cy = readParameter<float>(fSettings,"Camera2.cy",found);


            vCalibration = {fx, fy, cx, cy};

            calibration2_ = new Pinhole(vCalibration);
            originalCalib2_ = new Pinhole(vCalibration);

            //Check if it is a distorted PinHole
            readParameter<float>(fSettings,"Camera2.k1",found,false);
            if(found){
                readParameter<float>(fSettings,"Camera2.k3",found,false);
                if(found){
                    vPinHoleDistorsion2_.resize(5);
                    vPinHoleDistorsion2_[4] = readParameter<float>(fSettings,"Camera2.k3",found);
                }
                else{
                    vPinHoleDistorsion2_.resize(4);
                }
                vPinHoleDistorsion2_[0] = readParameter<float>(fSettings,"Camera2.k1",found);
                vPinHoleDistorsion2_[1] = readParameter<float>(fSettings,"Camera2.k2",found);
                vPinHoleDistorsion2_[2] = readParameter<float>(fSettings,"Camera2.p1",found);
                vPinHoleDistorsion2_[3] = readParameter<float>(fSettings,"Camera2.p2",found);
            }
        }
        else if(cameraType_ == KannalaBrandt){
            //Read intrinsic parameters
            float fx = readParameter<float>(fSettings,"Camera2.fx",found);
            float fy = readParameter<float>(fSettings,"Camera2.fy",found);
            float cx = readParameter<float>(fSettings,"Camera2.cx",found);
            float cy = readParameter<float>(fSettings,"Camera2.cy",found);

            float k0 = readParameter<float>(fSettings,"Camera2.k1",found); // Luigi: fixed it was referencing to Camera1. Why?!
            float k1 = readParameter<float>(fSettings,"Camera2.k2",found);
            float k2 = readParameter<float>(fSettings,"Camera2.k3",found);
            float k3 = readParameter<float>(fSettings,"Camera2.k4",found);

            vFisheyeDistorsion2_ = {k0,k1,k2,k3};   

            if(bNeedToRectify_){

                //Rectified images are assumed to be ideal PinHole images (no distortion)      
                vCalibration = {fx, fy, cx, cy};  

                calibration2_ = new Pinhole(vCalibration); // the linearCameraFovScale_ is used below in the fisheye rectification process to get the adjusted P2 
                originalCalib2_ = new KannalaBrandt8({fx,fy,cx,cy,k0,k1,k2,k3},linearCameraFovScale_);      

            } else {

                vCalibration = {fx,fy,cx,cy,k0,k1,k2,k3};
                
                calibration2_ = new KannalaBrandt8(vCalibration,linearCameraFovScale_);
                originalCalib2_ = new KannalaBrandt8(vCalibration,linearCameraFovScale_);                

                int colBegin = readParameter<int>(fSettings,"Camera2.overlappingBegin",found);
                int colEnd = readParameter<int>(fSettings,"Camera2.overlappingEnd",found);
                vector<int> vOverlapping = {colBegin, colEnd};           

                static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea = vOverlapping;                  
            }

        }

        //Load stereo extrinsic calibration
        if(cameraType_ == Rectified){
            b_ = readParameter<float>(fSettings,"Stereo.b",found);
            bf_ = b_ * calibration1_->getLinearParameter(0);
        }
        else{
            cv::Mat cvTlr = readParameter<cv::Mat>(fSettings,"Stereo.T_c1_c2",found);
            Tlr_ = Converter::toSophus(cvTlr);

            //TODO: also search for Trl and invert if necessary

            b_ = Tlr_.translation().norm();
            bf_ = b_ * calibration1_->getLinearParameter(0);
            std::cout << "Stereo.b recomputed to " << b_ << std::endl; 
        }

        thDepth_ = readParameter<float>(fSettings,"Stereo.ThDepth",found);


    }

    void Settings::readImageInfo(cv::FileStorage &fSettings) {
        bool found;
        //Read original and desired image dimensions
        int originalRows = readParameter<int>(fSettings,"Camera.height",found);
        int originalCols = readParameter<int>(fSettings,"Camera.width",found);
        originalImSize_.width = originalCols;
        originalImSize_.height = originalRows;

        newImSize_ = originalImSize_;
        int newHeigh = readParameter<int>(fSettings,"Camera.newHeight",found,false);
        if(found){
            bNeedToResize1_ = true;
            newImSize_.height = newHeigh;

            if(!bNeedToRectify_){
                //Update calibration
                const float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
                imageScale_ = scaleRowFactor;
                calibration1_->setParameter(calibration1_->getParameter(1) * scaleRowFactor, 1);
                calibration1_->setParameter(calibration1_->getParameter(3) * scaleRowFactor, 3);
                calibration1_->updateLinearParameters();

                if((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified){
                    calibration2_->setParameter(calibration2_->getParameter(1) * scaleRowFactor, 1);
                    calibration2_->setParameter(calibration2_->getParameter(3) * scaleRowFactor, 3);
                    calibration2_->updateLinearParameters();
                }
            }
        }

        int newWidth = readParameter<int>(fSettings,"Camera.newWidth",found,false);
        if(found){
            bNeedToResize1_ = true;
            newImSize_.width = newWidth;

            if(!bNeedToRectify_){
                //Update calibration
                const float scaleColFactor = (float)newImSize_.width /(float) originalImSize_.width;
                MSG_ASSERT( fabs(scaleColFactor-imageScale_)<1e-6, "Image scale factors (on width and height) do not match!");
                calibration1_->setParameter(calibration1_->getParameter(0) * scaleColFactor, 0);
                calibration1_->setParameter(calibration1_->getParameter(2) * scaleColFactor, 2);
                calibration1_->updateLinearParameters();

                if((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified){
                    calibration2_->setParameter(calibration2_->getParameter(0) * scaleColFactor, 0);
                    calibration2_->setParameter(calibration2_->getParameter(2) * scaleColFactor, 2);
                    calibration2_->updateLinearParameters();

                    if(cameraType_ == KannalaBrandt){
                        static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea[0] *= scaleColFactor;
                        static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea[1] *= scaleColFactor;

                        static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea[0] *= scaleColFactor;
                        static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea[1] *= scaleColFactor;
                    }
                }
            }
        }

        fps_ = readParameter<int>(fSettings,"Camera.fps",found);
        bRGB_ = (bool) readParameter<int>(fSettings,"Camera.RGB",found);
    }

    void Settings::readIMU(cv::FileStorage &fSettings) {
        bool found;
        noiseGyro_ = readParameter<float>(fSettings,"IMU.NoiseGyro",found);
        noiseAcc_ = readParameter<float>(fSettings,"IMU.NoiseAcc",found);
        gyroWalk_ = readParameter<float>(fSettings,"IMU.GyroWalk",found);
        accWalk_ = readParameter<float>(fSettings,"IMU.AccWalk",found);
        imuFrequency_ = readParameter<float>(fSettings,"IMU.Frequency",found);

        cv::Mat cvTbc = readParameter<cv::Mat>(fSettings,"IMU.T_b_c1",found);
        Tbc_ = Converter::toSophus(cvTbc);

        readParameter<int>(fSettings,"IMU.InsertKFsWhenLost",found,false);
        if(found){
            insertKFsWhenLost_ = (bool) readParameter<int>(fSettings,"IMU.InsertKFsWhenLost",found,false);
        }
        else{
            insertKFsWhenLost_ = true;
        }
    }

    void Settings::readRGBD(cv::FileStorage& fSettings) {
        bool found;

        depthMapFactor_ = readParameter<float>(fSettings,"RGBD.DepthMapFactor",found);
        thDepth_ = readParameter<float>(fSettings,"Stereo.ThDepth",found);
        b_ = readParameter<float>(fSettings,"Stereo.b",found);
        bf_ = b_ * calibration1_->getLinearParameter(0);
    }

    void Settings::readORB(cv::FileStorage &fSettings) {
        bool found;

        nFeatures_ = readParameter<int>(fSettings,"ORBextractor.nFeatures",found);
        scaleFactor_ = readParameter<float>(fSettings,"ORBextractor.scaleFactor",found);
        nLevels_ = readParameter<int>(fSettings,"ORBextractor.nLevels",found);
        initThFAST_ = readParameter<int>(fSettings,"ORBextractor.iniThFAST",found);
        minThFAST_ = readParameter<int>(fSettings,"ORBextractor.minThFAST",found);
    }

    void Settings::readViewer(cv::FileStorage &fSettings) {
        bool found;

        keyFrameSize_ = readParameter<float>(fSettings,"Viewer.KeyFrameSize",found);
        keyFrameLineWidth_ = readParameter<float>(fSettings,"Viewer.KeyFrameLineWidth",found);
        graphLineWidth_ = readParameter<float>(fSettings,"Viewer.GraphLineWidth",found);
        
        pointSize_ = readParameter<float>(fSettings,"Viewer.PointSize",found);
        lineSize_ = Utils::GetParam(fSettings, "Viewer.LineSize", 1, false);
        objectLineSize_ = Utils::GetParam(fSettings, "Viewer.ObjectLineSize", 2, false);    

        cameraSize_ = readParameter<float>(fSettings,"Viewer.CameraSize",found);
        cameraLineWidth_ = readParameter<float>(fSettings,"Viewer.CameraLineWidth",found);
        viewPointX_ = readParameter<float>(fSettings,"Viewer.ViewpointX",found);
        viewPointY_ = readParameter<float>(fSettings,"Viewer.ViewpointY",found);
        viewPointZ_ = readParameter<float>(fSettings,"Viewer.ViewpointZ",found);
        viewPointF_ = readParameter<float>(fSettings,"Viewer.ViewpointF",found);
        
        imageViewerScale_ = readParameter<float>(fSettings,"Viewer.imageViewScale",found,false);
        if(!found)
            imageViewerScale_ = 1.0f;
    }

    void Settings::readLoadAndSave(cv::FileStorage &fSettings) {
        bool found;

        sLoadFrom_ = readParameter<string>(fSettings,"System.LoadAtlasFromFile",found,false);
        sSaveto_ = readParameter<string>(fSettings,"System.SaveAtlasToFile",found,false);
    }

    void Settings::readOtherParameters(cv::FileStorage& fSettings) {
        bool found;

        thFarPoints_ = readParameter<float>(fSettings,"System.thFarPoints",found,false);
    }

    void Settings::precomputeRectificationMaps(bool bUpdateCalibration) {

        //Precompute rectification maps, new calibrations, ...
        cv::Mat K1, K2; 
        std::vector<float> param1, param2; 
        // get the calibrations params without linear FOV scaling
        param1 = {calibration1_->getParameter(0), calibration1_->getParameter(1), calibration1_->getParameter(2), calibration1_->getParameter(3)}; // {fx1, fy1, cx1, cy1}
        param2 = {calibration2_->getParameter(0), calibration2_->getParameter(1), calibration2_->getParameter(2), calibration2_->getParameter(3)}; // {fx2, fy2, cx2, cy2}
        Pinhole cal1(param1);
        Pinhole cal2(param2);

        //K1 = calibration1_->toLinearK(); // here we get the original calibration 
        K1 = cal1.toLinearK();
        K1.convertTo(K1,CV_64F);
        //K2 = calibration2_->toLinearK();
        K2 = cal2.toLinearK();
        K2.convertTo(K2,CV_64F);

        cv::Mat cvTlr;
        cv::eigen2cv(Tlr_.inverse().matrix3x4(),cvTlr);
        cv::Mat R12 = cvTlr.rowRange(0,3).colRange(0,3);
        R12.convertTo(R12,CV_64F);
        cv::Mat t12 = cvTlr.rowRange(0,3).col(3);
        t12.convertTo(t12,CV_64F);

        if(cameraType_ == KannalaBrandt)
        {
            const float balance = 1.0; 
            const float& cameraFovScale = linearCameraFovScale_;       
            std::cout << "Unistorting and rectifying stereo fish-eye camera with fov scale: " << cameraFovScale << "" << std::endl; 
            cv::fisheye::stereoRectify(K1,camera1FisheyeDistortionCoef(),K2,camera2FisheyeDistortionCoef(),newImSize_,
                                    R12, t12,
                                    R_r1_u1_, R_r2_u2_, P1_, P2_, Q_,
                                    cv::CALIB_ZERO_DISPARITY,newImSize_,balance,cameraFovScale);        
            cv::fisheye::initUndistortRectifyMap(K1, camera1FisheyeDistortionCoef(), R_r1_u1_, P1_, newImSize_, CV_32F, M1l_, M2l_);
            cv::fisheye::initUndistortRectifyMap(K2, camera2FisheyeDistortionCoef(), R_r2_u2_, P2_, newImSize_, CV_32F, M1r_, M2r_);        

            std::cout << "============================================================" << std::endl; 
            std::cout << "Settings::precomputeRectificationMaps()" << std::endl;
            std::cout << "K1" << std::endl << K1 << std::endl;
            std::cout << "K2" << std::endl << K2 << std::endl;
            std::cout << "cameraFovScale: " << cameraFovScale << std::endl;
            std::cout << "M1l(0:3,0:3)" << std::endl << M1l_.rowRange(0,3).colRange(0,3) << std::endl;
            std::cout << "M2l(0:3,0:3)" << std::endl << M2l_.rowRange(0,3).colRange(0,3) << std::endl;
            std::cout << "M1r(0:3,0:3)" << std::endl << M1r_.rowRange(0,3).colRange(0,3) << std::endl;       
            std::cout << "M2r(0:3,0:3)" << std::endl << M2r_.rowRange(0,3).colRange(0,3) << std::endl;
            std::cout << "============================================================" << std::endl;             
        } 
        else if (cameraType_ == PinHole)
        {

            std::cout << "Unistorting and rectifying stereo pin-hole camera" << std::endl;             
            cv::stereoRectify(K1,camera1DistortionCoef(),K2,camera2DistortionCoef(),newImSize_,
                            R12, t12,
                            R_r1_u1_, R_r2_u2_, P1_, P2_, Q_,
                            cv::CALIB_ZERO_DISPARITY,-1,newImSize_);
            cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1_, P1_.rowRange(0, 3).colRange(0, 3),
                                        newImSize_, CV_32F, M1l_, M2l_);
            cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2_, P2_.rowRange(0, 3).colRange(0, 3),
                                        newImSize_, CV_32F, M1r_, M2r_);
        } 
        else 
        {
            MSG_ASSERT(false,"Unexpected camera type: " << cameraType_);
        }
        
        //Compute relative pose between camera rectified and raw
        // R_r1_u1: from points given in the unrectified first camera's coordinate system to points in the rectified first camera's coordinate system
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1_, eigenR_r1_u1);
        T_r1_u1_ = Sophus::SE3f(eigenR_r1_u1,Eigen::Vector3f::Zero());

        if(bUpdateCalibration)
        {
            //Update calibration
            calibration1_->setParameter(P1_.at<double>(0,0), 0);
            calibration1_->setParameter(P1_.at<double>(1,1), 1);
            calibration1_->setParameter(P1_.at<double>(0,2), 2);
            calibration1_->setParameter(P1_.at<double>(1,2), 3);
            calibration1_->updateLinearParameters(1.0f); // We use 1.0 instead of linearCameraFovScale_ since P1_ was computed by using the above "cameraFovScale"

            calibration2_->setParameter(P2_.at<double>(0,0), 0);
            calibration2_->setParameter(P2_.at<double>(1,1), 1);
            calibration2_->setParameter(P2_.at<double>(0,2), 2);
            calibration2_->setParameter(P2_.at<double>(1,2), 3);
            calibration2_->updateLinearParameters(1.0f); // We use 1.0 instead of linearCameraFovScale_ since P2_ was computed by using the above "cameraFovScale"        

            MSG_ASSERT(bNeedToUndistort_==false,"We should not have to undistort here!"); // in Tracking class, this guides to consider the vector of dist coefficient as zeros

            //Update bf
            bf_ = b_ * P1_.at<double>(0,0);

            //Update relative pose between camera 1 and IMU if necessary
            if(sensor_ == System::STEREO || sensor_ == System::IMU_STEREO){
                Tbc_ = Tbc_ * T_r1_u1_.inverse();
            }

            // NOTE: at this point we can safely set that the camera type is Rectified, that is, the camera models are simple pin-hole
            cameraType_ = PinHole;            
        }
    }

    void Settings::readLines(cv::FileStorage& fSettings)
    {
        // Line extractor

        lines_.bLineTrackerOn = Utils::GetParam(fSettings, "Line.on", false);      
        lines_.nNumLineFeatures = Utils::GetParam(fSettings, "Line.nfeatures", 100); // read again just for printing out
            
        lines_.lsdOptions.numOctaves = Utils::GetParam(fSettings, "Line.nLevels", LineExtractor::kNumOctavesForBdDefault); 
        
        lines_.lsdOptions.scale = Utils::GetParam(fSettings, "Line.scaleFactor", LineExtractor::kScaleFactorDefault);  
        if( Tracking::skUsePyramidPrecomputation )
        {
            lines_.lsdOptions.scale = scaleFactor_;
        }
        
        LineExtractor::skSigma0 = Utils::GetParam(fSettings, "Line.sigma", LineExtractor::skSigma0);    
            
        // LSD options  https://docs.opencv.org/3.0-beta/modules/imgproc/doc/feature_detection.html#linesegmentdetector
        LineExtractor::skUseLsdExtractor = static_cast<int> (Utils::GetParam(fSettings, "Line.LSD.on", (int)LineExtractor::skUseLsdExtractor)) != 0;    
        lines_.lsdOptions.refine = Utils::GetParam(fSettings, "Line.LSD.refine", 1);   
        lines_.lsdOptions.sigma_scale = Utils::GetParam(fSettings, "Line.LSD.sigmaScale", 0.6);
        lines_.lsdOptions.quant = Utils::GetParam(fSettings, "Line.LSD.quant", 2.0);
        lines_.lsdOptions.ang_th = Utils::GetParam(fSettings, "Line.LSD.angTh", 22.5);
        lines_.lsdOptions.log_eps = Utils::GetParam(fSettings, "Line.LSD.logEps", 1.0);
        lines_.lsdOptions.density_th = Utils::GetParam(fSettings, "Line.LSD.densityTh", 0.6);
        lines_.lsdOptions.n_bins = Utils::GetParam(fSettings, "Line.LSD.nbins", 1024);
        lines_.lsdOptions.min_length = Utils::GetParam(fSettings, "Line.minLineLength", 0.025);
        lines_.lsdOptions.lineFitErrThreshold = Utils::GetParam(fSettings, "Line.lineFitErrThreshold", lines_.lsdOptions.lineFitErrThreshold);

        // Luigi: TODO remove the static global params 
        
        Tracking::sknLineTrackWeigth = 0;
        if(lines_.bLineTrackerOn) 
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
            
    }

    ostream &operator<<(std::ostream& output, const Settings& settings){
        output << "SLAM settings: " << endl;

        output << "\t-Camera 1 original parameters (";
        const auto originalCameraType = settings.originalCalib1_->GetType();
        if(originalCameraType == GeometricCamera::CAM_PINHOLE){
            output << "Pinhole";
        }
        else{
            output << "Kannala-Brandt";
        }
        output << ")" << ": [";
        for(size_t i = 0; i < settings.originalCalib1_->size(); i++){
            output << " " << settings.originalCalib1_->getParameter(i);
        }
        output << " ]" << endl;

        if(!settings.vPinHoleDistorsion1_.empty() || !settings.vFisheyeDistorsion1_.empty()){
            output << "\t-Camera 1 original distortion parameters: [ ";
            if(originalCameraType == GeometricCamera::CAM_PINHOLE){
                for(float d : settings.vPinHoleDistorsion1_){
                    output << " " << d;
                }
            }
            else{
                for(float d : settings.vFisheyeDistorsion1_){
                    output << " " << d;
                }
            }
            output << " ]" << endl;
        }

        output << "\t-Camera 1 actual parameters (";
        const auto actualCameraType = settings.cameraType();
        if(actualCameraType == Settings::PinHole || actualCameraType==  Settings::Rectified){
            output << "Pinhole";
        }
        else{
            output << "Kannala-Brandt";
        }
        output << ")" << ": [";
        for(size_t i = 0; i < settings.calibration1_->size(); i++){
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << endl;

        if(!settings.vPinHoleDistorsion1_.empty() || !settings.vFisheyeDistorsion2_.empty()){
            output << "\t-Camera 1 actual distortion parameters: [ ";
            if(settings.needToUndistort()){ // same logic used in Tracking::newParameterLoader()
                if(actualCameraType == GeometricCamera::CAM_PINHOLE){
                    for(float d : settings.vPinHoleDistorsion1_){
                        output << " " << d;
                    }
                }
                else{
                    for(float d : settings.vFisheyeDistorsion1_){
                        output << " " << d;
                    }
                }
            }
            output << " ]" << endl;
        }
        output << " " << endl; 

        if(settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO){
            output << "\t-Camera 2 original parameters (";
            const auto originalCamera2Type = settings.originalCalib2_->GetType();
            if(originalCamera2Type == GeometricCamera::CAM_PINHOLE){
                output << "Pinhole";
            }
            else{
                output << "Kannala-Brandt";
            }
            output << ")" << ": [";
            for(size_t i = 0; i < settings.originalCalib2_->size(); i++){
                output << " " << settings.originalCalib2_->getParameter(i);
            }
            output << " ]" << endl;

            if(!settings.vPinHoleDistorsion2_.empty() || !settings.vFisheyeDistorsion2_.empty()){
                output << "\t-Camera 2 original distortion parameters: [ ";
                if(originalCamera2Type == GeometricCamera::CAM_PINHOLE){
                    for(float d : settings.vPinHoleDistorsion2_){
                        output << " " << d;
                    }
                } else {
                    for(float d : settings.vFisheyeDistorsion2_){
                        output << " " << d;
                    }
                }
                output << " ]" << endl;
            }

            output << "\t-Camera 2 actual parameters (";
            const auto actualCamera2Type = settings.cameraType();
            if(actualCamera2Type == Settings::PinHole || actualCamera2Type==  Settings::Rectified){
                output << "Pinhole";
            }
            else{
                output << "Kannala-Brandt";
            }
            output << ")" << ": [";
            for(size_t i = 0; i < settings.calibration2_->size(); i++){
                output << " " << settings.calibration2_->getParameter(i);
            }
            output << " ]" << endl;

            if(!settings.vPinHoleDistorsion2_.empty() || !settings.vFisheyeDistorsion2_.empty()){
                output << "\t-Camera 2 actual distortion parameters: [ ";
                if(settings.needToUndistort()){ // same logic used in Tracking::newParameterLoader()
                    if(actualCamera2Type == GeometricCamera::CAM_PINHOLE){
                        for(float d : settings.vPinHoleDistorsion2_){
                            output << " " << d;
                        }
                    }
                    else{
                        for(float d : settings.vFisheyeDistorsion2_){
                            output << " " << d;
                        }
                    }
                }
                output << " ]" << endl;
            }
            output << " " << endl; 
        }        

        output << "\t-Original image size: [ " << settings.originalImSize_.width << " , " << settings.originalImSize_.height << " ]" << endl;
        output << "\t-Current image size: [ " << settings.newImSize_.width << " , " << settings.newImSize_.height << " ]" << endl;

        if(settings.bNeedToRectify_){
            output << "\t-Camera 1 parameters after rectification: [ ";
            for(size_t i = 0; i < settings.calibration1_->size(); i++){
                output << " " << settings.calibration1_->getParameter(i);
            }
            output << " ]" << endl;
        }
        else if(settings.bNeedToResize1_){
            output << "\t-Camera 1 parameters after resize: [ ";
            for(size_t i = 0; i < settings.calibration1_->size(); i++){
                output << " " << settings.calibration1_->getParameter(i);
            }
            output << " ]" << endl;

            if((settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO) &&
                settings.cameraType_ == Settings::KannalaBrandt){
                output << "\t-Camera 2 parameters after resize: [ ";
                for(size_t i = 0; i < settings.calibration2_->size(); i++){
                    output << " " << settings.calibration2_->getParameter(i);
                }
                output << " ]" << endl;
            }
        }

        output << "\t-Sequence FPS: " << settings.fps_ << endl;

        //Stereo stuff
        if(settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO){
            output << "\t-Stereo baseline: " << settings.b_ << endl;
            output << "\t-Stereo depth threshold : " << settings.thDepth_ << endl;

            if(settings.cameraType_ == Settings::KannalaBrandt && !settings.bNeedToRectify_){
                auto vOverlapping1 = static_cast<KannalaBrandt8*>(settings.calibration1_)->mvLappingArea;
                auto vOverlapping2 = static_cast<KannalaBrandt8*>(settings.calibration2_)->mvLappingArea;
                output << "\t-Camera 1 overlapping area: [ " << vOverlapping1[0] << " , " << vOverlapping1[1] << " ]" << endl;
                output << "\t-Camera 2 overlapping area: [ " << vOverlapping2[0] << " , " << vOverlapping2[1] << " ]" << endl;
            }
        }

        if(settings.sensor_ == System::IMU_MONOCULAR || settings.sensor_ == System::IMU_STEREO || settings.sensor_ == System::IMU_RGBD) {
            output << "\t-Gyro noise: " << settings.noiseGyro_ << endl;
            output << "\t-Accelerometer noise: " << settings.noiseAcc_ << endl;
            output << "\t-Gyro walk: " << settings.gyroWalk_ << endl;
            output << "\t-Accelerometer walk: " << settings.accWalk_ << endl;
            output << "\t-IMU frequency: " << settings.imuFrequency_ << endl;
        }

        if(settings.sensor_ == System::RGBD || settings.sensor_ == System::IMU_RGBD){
            output << "\t-RGB-D depth map factor: " << settings.depthMapFactor_ << endl;
        }

        output << "\t-Features per image: " << settings.nFeatures_ << endl;
        output << "\t-ORB scale factor: " << settings.scaleFactor_ << endl;
        output << "\t-ORB number of scales: " << settings.nLevels_ << endl;
        output << "\t-Initial FAST threshold: " << settings.initThFAST_ << endl;
        output << "\t-Min FAST threshold: " << settings.minThFAST_ << endl;

        return output;
    }

};
