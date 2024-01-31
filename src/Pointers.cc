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

#include "Pointers.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapObject.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace PLVS2
{
     
/// < N.B.: keep the following lines coherent with the content of 'Pointers.h'
    
class MapPoint;
DEFINE_POINTERS(MapPoint)
//DEFINE_SMART_POINTERS(MapPoint)

class MapLine;
DEFINE_POINTERS(MapLine)
//DEFINE_SMART_POINTERS(MapLine)

class MapObject;
DEFINE_POINTERS(MapObject)
//DEFINE_SMART_POINTERS(MapObject)

class KeyFrame; 
DEFINE_POINTERS(KeyFrame)
//DEFINE_SMART_POINTERS(KeyFrame)

class Frame;
DEFINE_POINTERS(Frame)
        
}        

