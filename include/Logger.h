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

//    template <typename T, unsigned int N>
//    Logger &operator<<(const T (&a)[N])
//    {
//        _ofile << a;
//        return *this;
//    }
    

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

