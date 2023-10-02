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

#ifndef GEOM2D_UTILS_H
#define GEOM2D_UTILS_H

#include <math.h>
#include <iostream>
#include <limits>
#include <Eigen/Core>

namespace PLVS2
{

class Line2DRepresentation
{
public:

    Line2DRepresentation() : theta(0.f), nx(0.f), ny(0.f), d(0.f)
    {
    }

    float theta; // normal orientation
    float nx, ny; // unit normal components 
    float d; // signed distance from origin 
};

class Geom2DUtils
{
public:

    // get the 3D minimum distance between 2 segments
    static float distSegment2Segment(const Eigen::Vector2f& s1P0, const Eigen::Vector2f& s1P1, const Eigen::Vector2f& s2P0, const Eigen::Vector2f& s2P1)
    {
        static const float SMALL_NUM = 1e-10;

        const Eigen::Vector2f u = s1P1 - s1P0;
        const Eigen::Vector2f v = s2P1 - s2P0;
        const Eigen::Vector2f w = s1P0 - s2P0;
        const float a = u.squaredNorm(); //dot(u,u);         // always >= 0
        const float b = u.dot(v);        //dot(u,v);
        const float c = v.squaredNorm(); //dot(v,v);         // always >= 0
        const float d = u.dot(w);        //dot(u,w);
        const float e = v.dot(w);        //dot(v,w);
        const float D = a*c - b*b; // always >= 0
        float sc, sN, sD = D;  // sc = sN / sD, default sD = D >= 0
        float tc, tN, tD = D;  // tc = tN / tD, default tD = D >= 0

        // compute the line parameters of the two closest points
        if (D < SMALL_NUM)
        { // the lines are almost parallel
            sN = 0.0; // force using point P0 on segment S1
            sD = 1.0; // to prevent possible division by 0.0 later
            tN = e;
            tD = c;
        }
        else
        { // get the closest points on the infinite lines
            sN = (b * e - c * d);
            tN = (a * e - b * d);
            if (sN < 0.0)
            { // sc < 0 => the s=0 edge is visible
                sN = 0.0;
                tN = e;
                tD = c;
            }
            else if (sN > sD)
            { // sc > 1  => the s=1 edge is visible
                sN = sD;
                tN = e + b;
                tD = c;
            }
        }

        if (tN < 0.0)
        { // tc < 0 => the t=0 edge is visible
            tN = 0.0;
            // recompute sc for this edge
            if (-d < 0.0)
                sN = 0.0;
            else if (-d > a)
                sN = sD;
            else
            {
                sN = -d;
                sD = a;
            }
        }
        else if (tN > tD)
        { // tc > 1  => the t=1 edge is visible
            tN = tD;
            // recompute sc for this edge
            if ((-d + b) < 0.0)
                sN = 0;
            else if ((-d + b) > a)
                sN = sD;
            else
            {
                sN = (-d + b);
                sD = a;
            }
        }
        // finally do the division to get sc and tc
        sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
        tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

        // get the difference of the two closest points
        const Eigen::Vector2f dP = w + (sc * u) - (tc * v); // =  S1(sc) - S2(tc)

        //printf("tc = %f, sc = %f\n",sc,tc);

        /// < only valid for 2D case
        if ((tc > 0) && (tc < 1) && (sc > 0) && (sc < 1)) return 0.;

        return dP.norm(); // return the closest distance
    }
    

    static void GetLine2dRepresentationNoTheta(const float &xs, const float &ys, const float &xe, const float &ye, Line2DRepresentation& lineRepresentation)
    {
        // compute line normal
        lineRepresentation.nx = (ye - ys);
        lineRepresentation.ny = (xs - xe);
        if (lineRepresentation.nx < 0) // always get representations with nx>=0
        {
            lineRepresentation.nx *= -1.0f;
            lineRepresentation.ny *= -1.0f;
        } 
        
        const float nNormInv = 1.0f / sqrt(lineRepresentation.nx * lineRepresentation.nx + lineRepresentation.ny * lineRepresentation.ny);
        lineRepresentation.nx *= nNormInv;
        lineRepresentation.ny *= nNormInv;
        
        lineRepresentation.d = lineRepresentation.nx*xe + lineRepresentation.ny*ye;
        /// < lineRepresentation.theta = atan2(lineRepresentation.ny, lineRepresentation.nx);
    }
        
    static void GetLine2dRepresentation(const float &xs, const float &ys, const float &xe, const float &ye, Line2DRepresentation& lineRepresentation)
    {
        GetLine2dRepresentationNoTheta(xs,ys,xe,ye,lineRepresentation);
        lineRepresentation.theta = atan2(lineRepresentation.ny, lineRepresentation.nx);
        /// < N.B here theta always belongs to [-pi/2,pi/2] since nx>=0
    }    
    
    // we assume the two lines l1 and l2 are "normalized", that is they are in the form  l=[ nx, ny, -d] where nx^2+ny^2 = 1 
    static bool areLinesEqual(const Eigen::Vector3d& l1, const Eigen::Vector3d& l2, const float dotThreshold = 0.01f, const float distThreshold = 1.f)
    {       
        const float normalsDot = l1(0)*l2(0) + l1(1)*l2(1);
        float d1 = l1(2);
        const float d2 = l2(2);
        
        if( fabs(1. - fabs(normalsDot)) < dotThreshold ) // if normals are parallel or anti-parallel  
        {            
            if( normalsDot < 0) d1*=-1; // if normals are anti-parallel reverse the distance parameter of l1
            const float distDifference = fabs( d1 - d2 ); // |d1-d2| [pixel]
            if( distDifference < distThreshold  ) 
            {
                return true;
            }            
        }
        
        return false; 

    }    

};

}// namespace PLVS2

#endif 
