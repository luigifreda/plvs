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

#ifndef GL_OBJECT_LIST_H
#define GL_OBJECT_LIST_H

#include <deque>
#include <mutex>

#include "GlObject.h"


namespace PLVS2
{


class GlObjectList:
public GlObject
{
public:

    GlObjectList(){}
    ~GlObjectList(){}
 
    bool Load(const std::string& fileStorageName);
    
    GlObject::Ptr operator[](int i) { return deque_.at(i); }

    void PushFront(GlObject::Ptr object)
    { 
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);
        deque_.push_front(object); 
    } 
    
    void PushBack(GlObject::Ptr object) 
    { 
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);        
        deque_.push_back(object);	
    }  

    void Clear()
    { 
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);        
        deque_.clear(); 
    }
          
public:

    void Init()
    {
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);
        for (int i = 0, iEnd=deque_.size(); i < iEnd; i++)
        {
            deque_[i]->Init();
        }
    }

    void Draw()
    {            
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);
        for (int i = 0, iEnd=deque_.size(); i < iEnd; i++)
        {
            deque_[i]->Draw();
        }
    }      

    void Update()
    {
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);
        for (int i = 0, iEnd=deque_.size(); i < iEnd; i++)
        {
            deque_[i]->Update();
        }
    }

    void Delete()
    {
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);
        for (int i = 0, iEnd=deque_.size(); i < iEnd; i++)
        {
            deque_[i]->Delete();
        }
        deque_.clear(); 
    }
    
    void SetDisplayMode(int val)
    {
        std::unique_lock<std::recursive_timed_mutex> lck(mutex_);
        for (int i = 0, iEnd=deque_.size(); i < iEnd; i++)
        {
            deque_[i]->SetDisplayMode(val);
        }        
    }

public: 
    
    int GetSize() const { return deque_.size(); }    
    std::recursive_timed_mutex& GetMutex() { return mutex_; }   
    
protected:
	
    std::deque<GlObject::Ptr> deque_;
    std::recursive_timed_mutex mutex_;
        
};


} //namespace PLVS2

#endif
