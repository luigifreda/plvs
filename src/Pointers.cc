
#include "Pointers.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapObject.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace PLVS
{
     
/// < N.B.: keep the following lines coherent with the content of 'Pointers.h'
    
class MapPoint;
DEFINE_SMART_POINTERS(MapPoint)

class MapLine;
DEFINE_SMART_POINTERS(MapLine)

class MapObject;
DEFINE_SMART_POINTERS(MapObject)

class KeyFrame; 
DEFINE_POINTERS(KeyFrame)
//DEFINE_SMART_POINTERS(KeyFrame)

class Frame;
DEFINE_POINTERS(Frame)
        
}        

