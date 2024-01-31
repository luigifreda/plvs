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

#ifndef NEIGHBORHOOD_H
#define NEIGHBORHOOD_H

class NeighborhoodIndices
{
};

class FourNeighborhoodIndices: public NeighborhoodIndices
{    
public: 
    static const int kSize = 4; 
    
    static const int kiStartNormal=0;  // first considered index when computing normals 
    static const int kDiNormal=1;      // increment over index when computing normals, 1 means consider all (dm,dn)
    
    // define neighborhood deltas 
    static const int dm[kSize]; // delta row
    static const int dn[kSize]; // delta column
};

class EigthNeighborhoodIndices: public NeighborhoodIndices
{    
public: 
    static const int kSize = 8;
    
    static const int kiStartNormal=0; // first considered index when computing normals
    static const int kDiNormal=1;     // increment over index when computing normals, 1 means consider all (dm,dn)
    
    // define neighborhood deltas 
    static const int dm[kSize]; // delta row
    static const int dn[kSize]; // delta column
};

class EigthNeighborhoodIndicesFast: public NeighborhoodIndices
{    
public: 
    static const int kSize = 8;
    
    static const int kiStartNormal=1; // first considered index when computing normals
    static const int kDiNormal=2;     // increment over index when computing normals, 1 means consider all (dm,dn), 2 means we only consider indexes 0,2,4,6
    
    // define neighborhood deltas 
    static const int dm[kSize]; // delta row
    static const int dn[kSize]; // delta column
};


// anticlockwise order
const int FourNeighborhoodIndices::dm[kSize]={-1, 0, 1, 0}; // delta row
const int FourNeighborhoodIndices::dn[kSize]={ 0,-1, 0, 1}; // delta column

// anticlockwise order
const int EigthNeighborhoodIndices::dm[kSize]={-1,-1,-1, 0, 1, 1, 1, 0}; // delta row
const int EigthNeighborhoodIndices::dn[kSize]={ 1, 0,-1,-1,-1, 0, 1, 1}; // delta column

// anticlockwise order
const int EigthNeighborhoodIndicesFast::dm[kSize]={-1,-1,-1, 0, 1, 1, 1, 0}; // delta row
const int EigthNeighborhoodIndicesFast::dn[kSize]={ 1, 0,-1,-1,-1, 0, 1, 1}; // delta column

#endif /* NEIGHBORHOOD_H */

