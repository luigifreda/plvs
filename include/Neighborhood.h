/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   NeighborhoodIndices.h
 * Author: luigi
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

