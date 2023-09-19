

#ifndef POINT_SURFEL_SEGMENT_H
#define POINT_SURFEL_SEGMENT_H 

#include <cstdint>

// in order to avoid possible interferences with pangolin 
#ifdef HAVE_OPENNI
#undef HAVE_OPENNI
#endif
#ifdef HAVE_OPENNI2
#undef HAVE_OPENNI2
#endif

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace pcl
{



#define PCL_ADD_UNION_POINT4D_KFID \
  union EIGEN_ALIGN16 { \
    float data[4]; \
    struct { \
      float x; \
      float y; \
      float z; \
      uint32_t kfid; \
    }; \
  };


#define PCL_ADD_POINT4D_KFID \
  PCL_ADD_UNION_POINT4D_KFID \
  PCL_ADD_EIGEN_MAPS_POINT4D

struct EIGEN_ALIGN16 _PointSurfelSegment
{
    PCL_ADD_POINT4D_KFID; // This adds the members x,y,z, kfid which can also be accessed using the point (which is float[4])
   
    PCL_ADD_NORMAL4D; // This adds the member normal[3] which can also be accessed using the point (which is float[4])

    union
    {
        struct
        {
            PCL_ADD_UNION_RGB; // 4 bytes
            float depth;      // 4 bytes 
            
//            float curvature;   // 4 bytes 
//            
//            //float confidence;
//            uint8_t confidence; //1 byte to be capped
//                       
//            uint16_t label; // 2 bytes
//            uint8_t label_confidence; // 1 byte to be capped    
            
            uint32_t label;        // 4 bytes
            uint32_t label_confidence;   // 4 bytes
            
            //float confidence;
            //uint8_t confidence; //1 byte to be capped
                       
            //int8_t label_confidence; // 1 byte to be capped (signed for convenience and allowing decrement in zero)   
        };
        float data_c[4];
    };
    PCL_ADD_EIGEN_MAPS_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



struct PointSurfelSegment : public _PointSurfelSegment
{

    inline PointSurfelSegment(const _PointSurfelSegment &p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
        kfid = p.kfid;
        //data[3] = 1.0f;
        rgba = p.rgba;
        depth = p.depth;
        //confidence = p.confidence;
        //curvature = p.curvature;
        label = p.label;
        label_confidence = p.label_confidence; 
    }

    inline PointSurfelSegment()
    {
        x = y = z = 0.0f;
        kfid = 0; 
        //data[3] = 1.0f;
        normal_x = normal_y = normal_z = data_n[3] = 0.0f;
        rgba = 0;
        depth = 0.f;
        //curvature = 0.0f;
        //confidence = 0u; 
        label = 0u;
        label_confidence = 0u; 
        
    }

    //friend std::ostream& operator<<(std::ostream& os, const PointSurfelSegment& p);
};

}

//PCL_EXPORTS std::ostream& operator<<(std::ostream& os, const pcl::PointSurfelSegment& p);

inline std::ostream& operator<<(std::ostream& s, const pcl::PointSurfelSegment & v)
{
    s << std::endl;
    s << "p: (" << v.x << ", " << v.y << ", " << v.z << "), n: (" << v.normal_x << ", " << v.normal_y << ", " << v.normal_z << ") "; 
    s << "kfid: " << v.kfid << " ";
    s << "label: " << v.label << ", label confidence: " << (int)v.label_confidence; 

    return (s);
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointSurfelSegment,
                                  (float, x, x)(float, y, y)(float, z, z) (uint32_t, kfid, kfid)
                                  (float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)
                                  (uint32_t, rgba, rgba)
                                  (float, depth, depth)
                                  (uint32_t, label, label)
                                  //(uint16_t, curvature, curvature)
                                  //(uint8_t, confidence, confidence)
                                  (uint32_t, label_confidence, label_confidence)
                                  )
        
        


#endif 