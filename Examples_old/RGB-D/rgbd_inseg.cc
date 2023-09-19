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
#include<algorithm>
#include<fstream>
#include<chrono>

#include <unistd.h>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "System.h"
#include "Tracking.h"

using namespace std;
#include "opencv2/opencv.hpp"

#include <Eigen/Dense>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <sys/timeb.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>


std::shared_ptr<PLVS2::System> pSlam;


inline double readTime()
{
    timeb timebuffer;
    ftime( &timebuffer );
    return (double) timebuffer.time + (double)timebuffer.millitm*1e-3;		
}
        

/// test class
class InSegTest
{
public:
    InSegTest()
    : frameIndex(0)
    {}
    virtual ~InSegTest(){}
    
    /// initialize test
//    bool initialize(std::string calibFilename);

    /// main loop function
    void mainLoop(std::string depthSeqencePath, std::string colorSeqencePath );
    
    /// save reconstructed map
//    void saveMap(std::string modelFilename);
    
protected:
    /// grab frame
    bool retrieveFrame(std::string& depthSeqencePath, std::string& colorSeqencePath);
    
    ///
    bool isFileExist(char const* filename);
    
protected:
    /// color image on current frame
    cv::Mat bgrImage;
    /// depth image on current frame
    cv::Mat depthImage;
    
    ///
//    InSegLib slam;
    
    /// directory name of input image seqences
    std::string dataPath;
    
    /// current frame index
    int frameIndex;
};

//
//bool InSegTest::initialize(std::string calibFilename)
//{
//    frameIndex = 0;
//    this->dataPath = dataPath;
//    
//    std::ifstream cparamIn;
//    cparamIn.open(calibFilename);
//    if (cparamIn.fail() == true){
//        return false;
//    }
//    
//    CamParams cp;
//    cparamIn >> cp.imgWidth >> cp.imgHeight;
//    cparamIn >> cp.fx >> cp.fy;
//    cparamIn >> cp.cx >> cp.cy;
//    
//    INSEG_CONFIG.setCamParams(cp);
//    
//    return true;
//}

/// grab frame
bool InSegTest::retrieveFrame(std::string& depthSeqencePath, std::string& colorSeqencePath)
{
    std::stringstream depthFilename;
    depthFilename << depthSeqencePath<< std::setfill('0') << std::setw(5) << frameIndex << ".png";
    
    std::stringstream colorFilename;
    colorFilename << colorSeqencePath << std::setfill('0') << std::setw(5) << frameIndex << ".png";
    
    bool isExist = isFileExist(depthFilename.str().c_str()) || isFileExist(colorFilename.str().c_str());
    
    if (isExist == false){
        frameIndex = 0;
        return false;
    }
    
    depthImage = cv::imread(depthFilename.str(), cv::IMREAD_UNCHANGED);
    bgrImage = cv::imread(colorFilename.str(), cv::IMREAD_UNCHANGED);
    
    frameIndex++;
    
    return true;
}

bool InSegTest::isFileExist(char const* filename)
{
    std::ifstream file_tmp(filename);
    if (!file_tmp.is_open())
    {
        return false;
    }
    file_tmp.close();
    return true;
}

//void InSegTest::saveMap(std::string modelFilename)
//{
//    slam.getMap().saveModel(modelFilename.c_str());
//}

void InSegTest::mainLoop(std::string depthSeqencePath, std::string colorSeqencePath )
{
    int mT = 30; 
    boost::posix_time::ptime start;
    int milliseconds_to_sleep = 0;
    
    while (1)
    {
        start = boost::posix_time::microsec_clock::local_time();
                
        std::cout << "FrameIndex " << frameIndex << ": " << std::endl;
        
        // grab image & update system state
        if( retrieveFrame(depthSeqencePath, colorSeqencePath) == false){
            break;
        }
        
        double tframe = readTime();
        
        // input the images to system
        //slam.processFrame(depthImage, bgrImage);
        pSlam->TrackRGBD(bgrImage,depthImage,tframe);
        
    //    static bool isFirstFrame = true;
    //    if(isFirstFrame){
    //        // initialize map
    //        slam.initializeMap();
    //        isFirstFrame = false;
    //    }
        
        // print current pose
        //std::cout << slam.getCurrentPose() << std::endl;
        
        // get current reconstructed map
        //WorldMap& map = slam.getMap();
        
#if 0        
        if(pSlam->GetTrackingState() != PLVS2::Tracking::NOT_INITIALIZED)
        {
            getchar();
        }
#else
        
        milliseconds_to_sleep = (boost::posix_time::microsec_clock::local_time() - start).total_milliseconds() - mT;
        if (milliseconds_to_sleep > 0)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(milliseconds_to_sleep));
        }
#endif
        
    }
    
    std::cout << "\n******************\n" << std::endl;
    std::cout << "press a key to end" << std::endl;
    std::cout << "\n******************\n" << std::endl;
    getchar();

    // Stop all threads
    pSlam->Shutdown();
}

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./rgbd_inseg path_to_vocabulary path_to_settings path_to_sequence" << endl;
        cerr << "argc: " << argc << endl;
        return 1;
    }
    
    cout << "running..." << endl;

    cout << "retrieve paths to images ..." << endl;
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
//    string strAssociationFilename = string(argv[4]);
//    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

//    cout << "check consistency in the number of images and depthmaps ..." << endl;
//    // Check consistency in the number of images and depthmaps
//    int nImages = vstrImageFilenamesRGB.size();
//    if(vstrImageFilenamesRGB.empty())
//    {
//        cerr << endl << "No images found in provided path." << endl;
//        return 1;
//    }
//    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
//    {
//        cerr << endl << "Different number of images for rgb and depth." << endl;
//        return 1;
//    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    pSlam.reset(new PLVS2::System(argv[1],argv[2],PLVS2::System::RGBD,true));
    
    
    std::string dataSetPath = std::string(argv[3]);
    // calibration filename (data from openCV calibration function)
    //std::string calibFilename    = "../Data/vgaOffice/seq_cparam.txt";
    // depth image sequence filename [.png]
    std::string depthSeqencePath = dataSetPath + "/seq_depth";
    // color image sequence filename [.png]
    std::string colorSeqencePath = dataSetPath + "/seq_color";
//    // output map filename [.ply]
//    std::string outputFilename   = "map.ply";
    
    // initialize
     InSegTest sys;
//    sys.initialize(calibFilename);
    
    // process main loop
    sys.mainLoop(depthSeqencePath, colorSeqencePath);
    
//    // save the final reconstructed map
//    sys.saveMap(outputFilename);
    
    return 0;
}


