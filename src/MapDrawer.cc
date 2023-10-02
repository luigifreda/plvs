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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "MapLine.h" 
#include "Utils.h"
#include "MapObject.h"
#include <pangolin/pangolin.h>
#include <mutex>

#define USE_ORIGINAL_MAP_LOCK_STYLE 1
#define DRAW_FOV_CENTERS 0


namespace PLVS
{

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mLineSize = Utils::GetParam(fSettings, "Viewer.LineSize", 1);
    mObjectLineSize = Utils::GetParam(fSettings, "Viewer.ObjectLineSize", 2);    
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mbUseAR = false;
}

void MapDrawer::DrawMapPoints()
{
#if USE_ORIGINAL_MAP_LOCK_STYLE   
    // original version 
    const vector<MapPointPtr> &vpMPs = mpMap->GetAllMapPoints();     // TOCLARIFY: why one should use "&" here ?
    const vector<MapPointPtr> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPointPtr> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
#else
    const vector<MapPointPtr> vpMPs = mpMap->GetAllMapPoints();   
    
    //set<MapPointPtr> spRefMPs;
    //mpMap->GetSetOfReferenceMapPoints(spRefMPs); // this makes the vector-to-set conversion occur while blocking the reference points in the map 
    
    const vector<MapPointPtr> vpRefMPs = mpMap->GetReferenceMapPoints();
    set<MapPointPtr> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());    // this makes the vector-to-set conversion occur in this thread   
#endif 

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        const cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    if(!mbUseAR){
        glPointSize(mPointSize);
    }else{
        glPointSize(mPointSize+1);
    }
    glBegin(GL_POINTS);
    if(!mbUseAR){
        glColor3f(1.0,0.0,0.0);
    }else{
        glColor3f(0.745,0.969,0.125);
    }
    for(set<MapPointPtr>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        const cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}


void MapDrawer::DrawMapLines()
{

#if USE_ORIGINAL_MAP_LOCK_STYLE    
    const vector<MapLinePtr> &vpMLs = mpMap->GetAllMapLines();
    const vector<MapLinePtr> &vpRefMLs = mpMap->GetReferenceMapLines();

    set<MapLinePtr> spRefMLs(vpRefMLs.begin(), vpRefMLs.end());
#else
    const vector<MapLinePtr> vpMLs = mpMap->GetAllMapLines(); 
    
    //set<MapLinePtr> spRefMLs;
    //mpMap->GetSetOfReferenceMapLines(spRefMLs); // this makes the vector-to-set conversion occur while blocking the reference lines in the map 
    
    const vector<MapLinePtr> vpRefMLs = mpMap->GetReferenceMapLines();
    set<MapLinePtr> spRefMLs(vpRefMLs.begin(), vpRefMLs.end());    // this makes the vector-to-set conversion occur in this thread  
#endif 

    if(vpMLs.empty())
        return;

    glLineWidth(mLineSize);
    glBegin(GL_LINES);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMLs.size(); i<iend;i++)
    {
        if(vpMLs[i]->isBad() || spRefMLs.count(vpMLs[i]))
            continue;

        //if(vpMLs[i]->Observations() < 3) continue;

        cv::Mat posStart, posEnd;
        vpMLs[i]->GetWorldEndPoints(posStart, posEnd);

        glVertex3f(posStart.at<float>(0),posStart.at<float>(1),posStart.at<float>(2));
        glVertex3f(posEnd.at<float>(0),posEnd.at<float>(1),posEnd.at<float>(2));
    }
    glEnd();



    if(!mbUseAR){
        glLineWidth(mLineSize);
    }else{
        glLineWidth(mLineSize+2);
    }
    glBegin(GL_LINES);
    if(!mbUseAR){
        glColor3f(1.0,0.0,0.0);
    }else{
        glColor3f(0.745,0.969,0.125);
    }


    for(set<MapLinePtr>::iterator sit=spRefMLs.begin(), send=spRefMLs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;

        //if((*sit)->Observations() < 3) continue;

        cv::Mat posStart, posEnd;
        (*sit)->GetWorldEndPoints(posStart, posEnd);

        glVertex3f(posStart.at<float>(0),posStart.at<float>(1),posStart.at<float>(2));
        glVertex3f(posEnd.at<float>(0),posEnd.at<float>(1),posEnd.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawMapObjects()
{

#if USE_ORIGINAL_MAP_LOCK_STYLE    
    const std::vector<MapObjectPtr > &vpMObjs = mpMap->GetAllMapObjects();    
    const vector<MapObjectPtr > &vpRefMObjs = mpMap->GetReferenceMapObjects();

    set<MapObjectPtr > spRefMObjs(vpRefMObjs.begin(), vpRefMObjs.end());
#else
    const std::vector<MapObjectPtr > vpMObjs = mpMap->GetAllMapObjects();    
    
    //set<MapObjectPtr > spRefMObjs;
    //mpMap->GetSetOfReferenceMapObjects(spRefMObjs);
    
    const vector<MapObjectPtr > vpRefMObjs = mpMap->GetReferenceMapObjects();    
    set<MapObjectPtr > spRefMObjs(vpRefMObjs.begin(), vpRefMObjs.end());    
#endif     
    
    if(vpMObjs.empty())
        return;
    
    
    glColor3f(0.0,1.0,0.0);
    glLineWidth(mObjectLineSize);
        
    for(size_t i=0, iend=vpMObjs.size(); i<iend;i++)
    {
        const MapObjectPtr& pObj = vpMObjs[i];         
        
        if( pObj->isBad() || spRefMObjs.count(pObj))
            continue;        

        const std::vector<cv::Point3f> corners = pObj->GetCurrent3DCorners();

        glBegin(GL_LINE_LOOP);

        for(size_t i=0, iend=corners.size(); i<iend;i++)
        {
            glVertex3f(corners[i].x,corners[i].y,corners[i].z);
        }
        glEnd();            
    }   
    
    if(!mbUseAR){
        glLineWidth(mObjectLineSize);
    }else{
        glLineWidth(mObjectLineSize+1);
    }

    if(!mbUseAR){
        glColor3f(0.0,0.0,1.0); // blue
    }else{
        glColor3f(0.745,0.969,0.125);
    }    
    
    for(set<MapObjectPtr >::iterator sit=spRefMObjs.begin(), send=spRefMObjs.end(); sit!=send; sit++)
    {
        const MapObjectPtr& pObj = (*sit);         
        
        if(pObj->isBad())
            continue;        

        const std::vector<cv::Point3f> corners = pObj->GetCurrent3DCorners();

        glBegin(GL_LINE_LOOP);

        for(size_t i=0, iend=corners.size(); i<iend;i++)
        {
            glVertex3f(corners[i].x,corners[i].y,corners[i].z);
        }
        glEnd();  

    }    
    
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFramePtr> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFramePtr pKF = vpKFs[i];
            const cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();                            
        }                
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFramePtr> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            const cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFramePtr>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFramePtr pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFramePtr> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFramePtr>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
    
    
#if DRAW_FOV_CENTERS
    glColor3f(0.0,0.0,1.0);   
    glPointSize(5);      
    glBegin(GL_POINTS);       
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        const cv::Mat fovCenter = vpKFs[i]->GetFovCenter();
        glVertex3f(fovCenter.at<float>(0),fovCenter.at<float>(1),fovCenter.at<float>(2));        
    }
    glEnd();    
#endif          
    
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace PLVS
