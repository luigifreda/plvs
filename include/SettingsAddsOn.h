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

#pragma once

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

namespace PLVS2 {

    struct LSDOptions{
        int    numOctaves = 1;
        int    refine = 1;
        double scale = 0.8;
        double sigma_scale = 0.6;
        double quant = 2.0;
        double ang_th = 22.5;
        double log_eps = 0;
        double density_th = 0.7;
        int    n_bins = 1024;
        double min_length = 0.025;
        
        double lineFitErrThreshold = 1.6; // used in EDLineDetector
    };    

    class SettingsLines {
    public:
        LSDOptions lsdOptions;

        bool bLineTrackerOn = false;    
        int nNumLineFeatures = 100; 
        
        bool bUsePyramidPrecomputation = false; // Tracking::skUsePyramidPrecomputation

        float sigma0 = 2.0; // base sigma for level 0 of the pyramid  // LineExtractor::skSigma0

        bool bUseLsdExtractor = false; // LineExtractor::skUseLsdExtractor
            
        float nLineTrackWeigth = 0.0; // Tracking::sknLineTrackWeigth = 0;
                                    //  line weight for weighting line observations w.r.t. point observations in tracking 
                                    //   {3 points} or {1 line + 1 point} are enough to zero the DOF of a body => {3 points} "=" {1 line + 1 point} = > 1 line counts as 2 points
    
        float stereoMaxDist =  20.; // [meters] // Tracking::skLineStereoMaxDist 

        float minLineLength3D = 0.01; // [meters]  // Frame::skMinLineLength3D
        
        float optMuWeightForLine3dDist = 0.5; // weight default value // Optimizer::skMuWeightForLine3dDist
        // update parameters which depend on Optimizer::skMuWeightForLine3dDist
        //Optimizer::skSigmaLineError3D = (1 + Optimizer::skMuWeightForLine3dDist)*Optimizer::kSigmaPointLineDistance;    
        //Optimizer::skInvSigma2LineError3D = 1.0/(Optimizer::skSigmaLineError3D * Optimizer::skSigmaLineError3D );     
    
    };
};


