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

#ifndef GENERIC_UTILS_H
#define GENERIC_UTILS_H

#include <math.h>
#include <iostream>
#include <fstream>      
#include <ostream>
#include <string> 
#include <vector>
#include <algorithm>

#include <opencv2/core.hpp>


#ifndef UNUSED_VAR
#define UNUSED_VAR(x) (void)x
#endif 

namespace PLVS2
{
    namespace IoColor 
    {
        enum Code 
        {
            FG_RED      = 31,
            FG_GREEN    = 32,
            FG_YELLOW   = 33,
            FG_BLUE     = 34,
            FG_DEFAULT  = 39,
            BG_RED      = 41,
            BG_GREEN    = 42,
            BG_YELLOW   = 43,            
            BG_BLUE     = 44,
            BG_DEFAULT  = 49
        };

        class Modifier 
        {
            Code code;
        public:
            Modifier(Code pCode) : code(pCode) {}
            friend std::ostream&
            operator<<(std::ostream& os, const Modifier& mod) 
            {
                return os << "\033[" << mod.code << "m";
            }
        };
        
        inline const Modifier Default() { return Modifier(FG_DEFAULT); }          
        inline const Modifier Red() { return Modifier(FG_RED); } 
        inline const Modifier Green() { return Modifier(FG_GREEN); }  
        inline const Modifier Yellow() { return Modifier(FG_YELLOW); }
        inline const Modifier Blue() { return Modifier(FG_BLUE); }                
    }
    


#define MSG_ASSERT(condition, message)                                         \
{                                                                              \
    if(!(condition))                                                           \
    {                                                                          \
        const IoColor::Modifier red = IoColor::Red();                          \
        const IoColor::Modifier def = IoColor::Default();                      \
        std::cerr << red << "Assertion failed at " << __FILE__ << ":" << __LINE__; \
        std::cerr << " inside " << __FUNCTION__ << def << std::endl;               \
        std::cerr << "Condition: " << #condition << std::endl;                 \
        std::cerr << "Message: " << message << std::endl;                      \
        std::abort();                                                          \
    }                                                                          \
}

#define MSG_LOG(message, prefix, modifier)                               \
{                                                                        \
    const IoColor::Modifier def = IoColor::Default();                    \
    std::cerr << modifier << prefix << message << def << std::endl;      \
}

#define MSG_ERROR(message)                                               \
{                                                                        \
    MSG_LOG(message, "ERROR: ", IoColor::Red())                          \
    std::abort();                                                        \
}

#define MSG_ERROR_STREAM(args) \
do                             \
{                              \
    std::stringstream ss;      \
    ss << args;                \
    MSG_ERROR(ss.str())        \
} while (0)


#define MSG_INFO(message)                                                \
{                                                                        \
    MSG_LOG(message, "INFO: ", IoColor::Blue())                          \
}

#define MSG_INFO_STREAM(args) \
do                            \
{                             \
    std::stringstream ss;     \
    ss << args;               \
    MSG_INFO(ss.str())        \
} while (0)


#define MSG_WARN(message)                                                \
{                                                                        \
    MSG_LOG(message, "WARNING: ", IoColor::Yellow())                     \
}

#define MSG_WARN_STREAM(args) \
do                               \
{                                \
    std::stringstream ss;        \
    ss << args;                  \
    MSG_WARN(ss.str())        \
} while (0)

  

class Utils
{
public:

    template <typename T>
    static T const GetParam(cv::FileStorage& fileStorage, const std::string& nodename, T const& defaultVal, bool printOut = true)
    {
        T val;
        if (!fileStorage[nodename].empty())
        {
            val = (T const) fileStorage[nodename];
        }
        else
        {
            val = defaultVal;
        }
        if(printOut) std::cout << nodename << ": " << val << std::endl;
        return val;
    }
    
    // separate the strings in different tokens using the character delimiters specified in the input string "delimiters"
    static void TokenizeString(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters= " ")
    {
            // Skip delimiters at beginning
            std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
            // Find first "non-delimiter"
            std::string::size_type pos = str.find_first_of(delimiters, lastPos);

            while (std::string::npos != pos || std::string::npos != lastPos)
            {
                    // Found a token, add it to the vector
                    tokens.push_back(str.substr(lastPos, pos - lastPos));
                    // Skip delimiters.  Note the "not_of"
                    lastPos = str.find_first_not_of(delimiters, pos);
                    // Find next "non-delimiter"
                    pos = str.find_first_of(delimiters, lastPos);
            }
    }    
    
    static bool hasSuffix(const std::string &str, const std::string &suffix) 
    {
        std::size_t index = str.find(suffix, str.size() - suffix.size());
        return (index != std::string::npos);
    }

    static std::string getFileNameWithouExtension(const std::string& fileName)
    {
        std::size_t pos = fileName.rfind('.');
        if (pos == std::string::npos) return fileName;
        std::string resString(fileName.substr(0, pos));
        return resString;
    }

    static bool fileExist(const std::string& fileName)
    {
        std::ifstream infile(fileName.c_str());
        //std::cout << "checking file: " << fileName << " " <<  (int)infile.good() << std::endl;     
        return infile.good();
    }
    
    
    // math utils 

    // return a positive modulo 
    static int Modulus(const int& val, const int& N)
    {
        //return (val >= N) ? (val - N) : ((val < 0) ? (val + N) : val);
        return (val%N+N)%N;     
    }

    template <typename T>
    static T Pow2(const T& x)
    {
        return x*x;
    }
    
        
    static float SigmaZ(const float& z)
    {
        // sigma model from the paper "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking"
        // sigma(z) = 0.0012 + 0.0019*(z-0.4)^2
        //quad = 0.0019;
        //linear = -0.00152; // -2*0.4*0.0019
        //const = 0.001504; // 0.4^2 * 0.0019 + 0.0012
        return (0.001504f -0.00152f*z + 0.0019f*z*z);
    }
    
    static float SigmaZminOverSigmaZ(const float& z)
    {
        static const double zMin = 0.5; // [m]
        static const double sigmaZ_zMin = SigmaZ(zMin);
        // sigma model from the paper "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking"
        // sigma(z) = 0.0012 + 0.0019*(z-0.4)^2
        //quad = 0.0019;
        //linear = -0.00152; // -2*0.4*0.0019
        //const = 0.001504; // 0.4^2 * 0.0019 + 0.0012
        return sigmaZ_zMin/SigmaZ(z); // [m]
    }
    
    static double FindSigmaSquared(std::vector<double> &vdErrorSquared)
    { 
        assert(vdErrorSquared.size() > 0);
        std::sort(vdErrorSquared.begin(), vdErrorSquared.end());
        double dMedianSquared = vdErrorSquared[vdErrorSquared.size()/2];
        return (1.4826*1.4826) * dMedianSquared;
    }    

};

template <>
inline bool const Utils::GetParam<bool>(cv::FileStorage& fileStorage, const std::string& nodename, bool const& defaultVal, bool printOut)
{
    int defaultValInt = defaultVal ? 1: 0;
    return Utils::GetParam<int>(fileStorage, nodename, defaultValInt, printOut) != 0;
}    

}// namespace PLVS2

#endif // CONVERTER_H
