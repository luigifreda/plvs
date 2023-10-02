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

// from http://simmesimme.github.io/tutorials/2015/09/20/signal-slot

#ifndef SIGNAL_HPP
#define SIGNAL_HPP

#include <functional>
#include <map>
#include <mutex>

// A signal object may call multiple slots with the
// same signature. You can connect functions to the signal
// which will be called when the emit() method on the
// signal object is invoked. Any argument passed to emit()
// will be passed to the given functions.
template <typename... Args>
class Signal
{
public:

    Signal() : currentFunctionId_(0) {}

    // copy creates new signal
    Signal(Signal const& other) : currentFunctionId_(0) {}

    // connects a member function to this Signal
    template <typename T>
    int connectMember(T *inst, void (T::*func)(Args...))
    {
        return connect([ = ](Args... args)
        {
            (inst->*func)(args...);
        });
    }

    // connects a const member function to this Signal
    template <typename T>
    int connectMember(T *inst, void (T::*func)(Args...) const)
    {  
        return connect([ = ](Args... args)
        {
            (inst->*func)(args...);
        });
    }

    // connects a std::function to the signal. The returned
    // value can be used to disconnect the function again
    int connect(std::function<void(Args...)> const& slot) const
    {
        std::unique_lock<std::recursive_mutex> locker(mutex_);        
        slots_.insert(std::make_pair(++currentFunctionId_, slot));
        return currentFunctionId_;
    }

    // disconnects a previously connected function
    void disconnect(int id) const
    {
        std::unique_lock<std::recursive_mutex> locker(mutex_);        
        slots_.erase(id);
    }

    // disconnects all previously connected functions
    void disconnectAll() const
    {
        std::unique_lock<std::recursive_mutex> locker(mutex_);        
        slots_.clear();
    }

    // calls all connected functions
    void emit(Args... p)
    {
        std::unique_lock<std::recursive_mutex> locker(mutex_);        
        for (auto it : slots_)
        {
            it.second(p...);
        }
    }

    // assignment creates new Signal
    Signal& operator=(Signal const& other)
    {
        if (this != &other)
        { 
            std::unique_lock<std::recursive_mutex> locker(mutex_);        
            disconnectAll();            
            
            std::unique_lock<std::recursive_mutex> lockerOther(other.mutex_);
            currentFunctionId_ = other.currentFunctionId_;
            slots_ = other.slots_; 
        }
        return *this;        
    }

private:
    
    mutable std::map<int, std::function<void(Args...) >> slots_;
    mutable int currentFunctionId_;
    mutable std::recursive_mutex mutex_;
    
};

#endif /* SIGNAL_HPP */



