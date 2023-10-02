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

#ifndef GL_COLORS_H
#define GL_COLORS_H

#include <pangolin/pangolin.h>

namespace PLVS2
{


class GlColor
{
public: 
    
    GlColor(): r(0), g(0), b(0), a(1) {}
    GlColor(float rin, float gin, float bin, float ain = 1.): r(rin), g(gin), b(bin), a(ain) {}

public:     
    float r;
    float g;
    float b;
    float a;
};


namespace GlColors
{
    static const float kNone_     = 0.00;
    static const float kDark     = 0.33;
    static const float kMedium	= 0.66;
    static const float kLight	= 0.99;

    enum GLColorType {kWhite = 0, kRed, kGreen, kBlue, kMagenta, kCyan, kYellow, kOrange, kBlack, kNumColors};

    // colors definitions 
    inline const GlColor Black()	{return GlColor(kNone_,kNone_,kNone_);}
    inline const GlColor White()	{return GlColor(1.,1.,1.);}

    // grey
    inline const GlColor DarkGrey()	{return GlColor(kDark,kDark,kDark);}
    inline const GlColor LightGrey()	{return GlColor(kMedium,kMedium,kMedium);}

    // red
    inline const GlColor LightRed()	{return GlColor(kLight,kNone_,kNone_);}
    inline const GlColor Red()		{return GlColor(kMedium,kNone_,kNone_);}
    inline const GlColor DarkRed()	{return GlColor(kDark,kNone_,kNone_);}

    // green 
    inline const GlColor LightGreen()	{return GlColor(kNone_,kLight,kNone_);}
    inline const GlColor Green()	{return GlColor(kNone_,kMedium,kNone_);}
    inline const GlColor DarkGreen()	{return GlColor(kNone_,kDark,kNone_);}

    // blue
    inline const GlColor LightBlue()	{return GlColor(kNone_,kNone_,kLight);}
    inline const GlColor Blue()		{return GlColor(kNone_,kNone_,kMedium);}
    inline const GlColor DarkBlue()	{return GlColor(kNone_,kNone_,kDark);}

    // magenta
    inline const GlColor LightMagenta()	{return GlColor(kLight,kNone_,kLight);}
    inline const GlColor Magenta()	{return GlColor(kMedium,kNone_,kMedium);}
    inline const GlColor DarkMagenta()	{return GlColor(kDark,kNone_,kDark);}

    // cyan
    inline const GlColor LightCyan()	{return GlColor(kNone_,kLight,kLight);}
    inline const GlColor Cyan()		{return GlColor(kNone_,kNone_,kMedium);}
    inline const GlColor DarkCyan()	{return GlColor(kNone_,kDark,kDark);}

    // yellow
    inline const GlColor LightYellow()	{return GlColor(kLight,kLight,kNone_);}
    inline const GlColor Yellow()	{return GlColor(kMedium,kMedium,kNone_);}
    inline const GlColor DarkYellow()	{return GlColor(kDark,kDark,kNone_);}

    // orange
    inline const GlColor Orange()	{return GlColor(kLight,kMedium,kNone_);}
    inline const GlColor DarkOrange()	{return GlColor(kMedium,kDark,kNone_);}
        
    inline const GlColor GetColor(int i)
    {
        i = i % kNumColors;
        switch(i)
        {
            case kWhite: { return White(); }
            case kRed: { return Red(); }
            case kGreen: { return Green(); }
            case kBlue: { return Blue(); }
            case kMagenta: { return Magenta(); }
            case kCyan: { return Cyan(); }
            case kYellow: { return Yellow(); }
            case kOrange: { return Orange(); }
            case kBlack: { return Black(); }
            default: { return White(); }
        }
    }

        
    inline void GLSetColor3f(const GlColor& color)
    {
        glColor3f(color.r,color.g,color.b);
    }

    inline void GLSetColor4f(const GlColor& color)
    {
        glColor4f(color.r,color.g,color.b,color.a);
    }

}


} //namespace PLVS2

#endif
