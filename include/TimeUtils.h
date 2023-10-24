/*
 * This file is part of PLVS
 * Copyright (C) 2018  Luigi Freda <luigifreda at gmail dot com>
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

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <math.h>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace PLVS2
{

class TimeUtils
{
public: 
    
    // return microseconds timestamp
    static boost::uint64_t getTimestamp()
    {
        using namespace boost::posix_time;
        static ptime epoch(boost::gregorian::date(1970, 1, 1));
        ptime now = boost::posix_time::microsec_clock::local_time();
        return (now - epoch).total_microseconds();
    }

    // return microseconds timestamp from double
    static boost::uint64_t getTimestampfromSec(double t) 
    { 
        uint32_t sec  = (uint32_t)floor(t);
        uint32_t usec = (uint32_t)round((t-sec) * 1e6);  
        return sec * 1000000 + usec;
    }

};


}// namespace PLVS2

#endif 
