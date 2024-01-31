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

#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <cstring>



class Logger
{
public:

    Logger(const std::string &filename) : _filename(filename)
    {
        if (!filename.empty())
        {
            _ofile.open(filename.c_str(), std::fstream::out);
            if (!_ofile.is_open())
            {
                std::cerr << "Logger: unable to open " << filename;
            }
        }
        else
        {
            std::cerr << "Logger: filename empty";
        }
    }

    ~Logger()
    {
        if (_ofile.is_open())
        {
            _ofile.close();
        }
    }

    template <typename T>
    Logger &operator<<(const T &a)
    {
        _ofile << a;
        return *this;
    }
    
    Logger &operator<<(std::ostream& (*pf) (std::ostream&))
    {
        // intercept std::endl
        _ofile << pf;
        return *this;
    }

    /// Writes block of data s, with a size of n characters
    void Write(const char* s, std::streamsize n)
    {
        _ofile.write(s, n);
    }

    void Clear()
    {
        _ofile.clear();
    }


protected:
    std::fstream _ofile;
    std::string _filename;
};




#endif // LOGGER_H

