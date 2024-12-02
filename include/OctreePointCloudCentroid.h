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

#ifndef OCTREE_CENTROID_CONTAINER_H
#define OCTREE_CENTROID_CONTAINER_H 

#include "PointDefinitions.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_singlepoint.h>

#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_impl.h>




#define POW2(x) (x*x)


namespace pcl
{
namespace octree
{

template <typename T> 
int sign(T val) 
{
    //return (val > T(0)) ? 1 : ((val < T(0)) ? -1 : 0);
    return (val < T(0)) ? -1 : 1;
}

// from https://www.codeproject.com/Articles/69941/Best-Square-Root-Method-Algorithm-Function-Precisi
inline float fast_sqrt(const float m)
{
    float i = 0;
    float x1, x2;
    while ((i * i) <= m)
        i += 0.1f;
    x1 = i;
    for (int j = 0; j < 10; j++)
    {
        x2 = m;
        x2 /= x1;
        x2 += x1;
        x2 /= 2;
        x1 = x2;
    }
    return x2;
}   



template<typename PointT>
class OctreePointCloudVoxelCentroidContainerMV : public OctreeContainerBase
{
    static const unsigned int kMaxPointCounterValForMovingAverage;// = 10;
    static const unsigned int kMaxPointCounterValForNormalMovingAverage;// = 10;
    static const int kMaxLabelConfidence;// = 20; 
        
public:

    /** \brief Class initialization. */
    OctreePointCloudVoxelCentroidContainerMV()
    {
        this->reset();
        time_stamp_ = 0; 
    }

    /** \brief Empty class deconstructor. */
    virtual ~OctreePointCloudVoxelCentroidContainerMV()
    {
    }

    /** \brief deep copy function */
    virtual OctreePointCloudVoxelCentroidContainerMV *
    deepCopy() const
    {
        return (new OctreePointCloudVoxelCentroidContainerMV(*this));
    }

    /** \brief Equal comparison operator - set to false
     * \param[in] OctreePointCloudVoxelCentroidContainer to compare with
     */
    virtual bool operator==(const OctreeContainerBase&) const
    {
        return ( false);
    }

    /** \brief Add new point to voxel.
     * \param[in] new_point the new point to add  
     */    
    template <class PointType, typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type* = nullptr>
    void addPoint(const PointType& new_point, const std::uint64_t time_stamp = 0)
    {
        using namespace pcl::common;
        
        const float weight = std::min(point_counter_, kMaxPointCounterValForMovingAverage);
        const float fc = 1. / (weight + 1.0f);

        point_centroid_ = (weight*point_centroid_ + new_point)  * fc;
        
        ++point_counter_;
        
        time_stamp_ = time_stamp;
    }
    
#if 1 //COMPUTE_NORMALS    
    template <class PointType, typename std::enable_if<
    pcl::traits::has_field<PointType, pcl::fields::normal_x>::value && !pcl::traits::has_field<PointType, pcl::fields::label>::value 
                                                      >::type* = nullptr> 
    void addPoint(const PointType& new_point, const std::uint64_t time_stamp = 0)
    {
        updateXYZRGBAndNormal(new_point, time_stamp);
    }

    template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::normal_x>::value>::type* = nullptr> 
    void updateXYZRGBAndNormal(const PointType& new_point, const std::uint64_t time_stamp = 0)
    {
        //std::cout << "addPoint with normals " << std::endl; 
        using namespace pcl::common;
        
        const float weight = std::min(point_counter_, kMaxPointCounterValForMovingAverage);
        const float fc = 1. / (weight + 1.0f);
  
        point_centroid_.x = (weight*point_centroid_.x + new_point.x)  * fc;
        point_centroid_.y = (weight*point_centroid_.y + new_point.y)  * fc;
        point_centroid_.z = (weight*point_centroid_.z + new_point.z)  * fc;
        point_centroid_.r = (weight*point_centroid_.r + new_point.r)  * fc;
        point_centroid_.g = (weight*point_centroid_.g + new_point.g)  * fc;
        point_centroid_.b = (weight*point_centroid_.b + new_point.b)  * fc; 
        
           
        const float weightNormal = std::min(point_counter_, kMaxPointCounterValForNormalMovingAverage);                                                            
        const float fcNormal = 1.0f / (weightNormal + 1.0f);

#if 0          
        // align to new normal
        const float scalarProdNormals = (point_centroid_.normal_x*new_point.normal_x) + (point_centroid_.normal_y*new_point.normal_y) + (point_centroid_.normal_z*new_point.normal_z);
        weightNormal *= sign(scalarProdNormals); 
#endif
        
        point_centroid_.normal_x = (weightNormal*point_centroid_.normal_x + new_point.normal_x)  * fcNormal;
        point_centroid_.normal_y = (weightNormal*point_centroid_.normal_y + new_point.normal_y)  * fcNormal;
        point_centroid_.normal_z = (weightNormal*point_centroid_.normal_z + new_point.normal_z)  * fcNormal; 
        const float norminv = 1.0f / fast_sqrt( POW2(point_centroid_.normal_x) + POW2(point_centroid_.normal_y)+ POW2(point_centroid_.normal_z) );
        point_centroid_.normal_x*=norminv;
        point_centroid_.normal_y*=norminv;
        point_centroid_.normal_z*=norminv;
                
        ++point_counter_;
        
        time_stamp_ = time_stamp;
    }
    
    
#if 1 // COMPUTE_SEGMENTS
    
    template <class PointType, typename std::enable_if< 
    pcl::traits::has_all_fields<PointType, boost::mpl::vector<pcl::fields::normal_x, pcl::fields::label> >::value
                                                      >::type* = nullptr> 
    void addPoint(const PointType& new_point, const std::uint64_t time_stamp = 0)
    {
        point_centroid_.kfid = new_point.kfid;
        updateLabel(new_point);  // TODO: add more efficient insertion without updateLabel() when no segmentation is required 
        updateXYZRGBAndNormal(new_point, time_stamp);
    }
    
    static float SigmaZ(float z)
    {
        // sigma model from the paper "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking"
        // sigma(z) = 0.0012 + 0.0019*(z-0.4)^2
        //quad = 0.0019;
        //linear = -0.00152; // -2*0.4*0.0019
        //const = 0.001504; // 0.4^2 * 0.0019 + 0.0012
        return (0.001504 -0.00152*z + 0.0019*z*z);
    }
    
    static float SigmaZminOverSigmaZ(float z)
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
    
    static int SigmaZminOverSigmaZApprox(const float& z)
    {
        if(z < 0.5f)
        {
            return 4;
        }
        else
        {
            if(z < 1.f)
            {
              return 2;
            }
            else
            {
                if(z < 2.f)
                {
                    return 1; 
                }
            }
        }
        return 0; 
    }



    
    template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::label>::value>::type* = nullptr> 
    void updateLabel(const PointType& new_point)
    {
        if(new_point.label == 0)
            return; /// < EXIT POINT
                
        if(point_centroid_.label==0)
        {
            point_centroid_.label = new_point.label;
            point_centroid_.depth = new_point.depth;
        }
        else
        {
            //int weigth = lrint(10*SigmaZminOverSigmaZ(new_point.depth)); 
            const int weigth = SigmaZminOverSigmaZApprox(new_point.depth);
                        
            if(point_centroid_.label == new_point.label)
            {
                point_centroid_.label_confidence = std::min( int(point_centroid_.label_confidence)+weigth, kMaxLabelConfidence); // clamping 
            }
            else
            {
                point_centroid_.label_confidence = std::max( int(point_centroid_.label_confidence)-weigth, 0);
                if(point_centroid_.label_confidence==0)
                {
                    point_centroid_.label = new_point.label;
                }
            }
            
        }
    }
    
    template <class PointType, typename std::enable_if<pcl::traits::has_field<PointType, pcl::fields::label>::value>::type* = nullptr> 
    void reinsertPoint(const PointType& new_point, const std::uint64_t time_stamp = 0)
    {    
        point_centroid_.kfid = new_point.kfid;
        point_centroid_.label = new_point.label;        
        point_centroid_.label_confidence = new_point.label_confidence;
        updateXYZRGBAndNormal(new_point, time_stamp);        
    }    
    
#endif    
    
    
#endif // end if COMPUTE_NORMALS
    
    template <class PointType, typename std::enable_if<!pcl::traits::has_field<PointType, pcl::fields::label>::value>::type* = nullptr> 
    void reinsertPoint(const PointType& new_point, const std::uint64_t time_stamp = 0)
    {    
        this->addPoint(new_point, time_stamp);
    }      
            
    /** \brief Calculate centroid of voxel.
     * \param[out] centroid_arg the resultant centroid of the voxel 
     */
    PointT getCentroid() const
    {
        return point_centroid_;
    }
    
    PointT& getCentroid()
    {
        return point_centroid_;
    }
    
    unsigned int getCounter() const 
    {
        return point_counter_;
    }
    unsigned int& getCounter() 
    {
        return point_counter_;
    }    
        
    std::uint64_t getTimestamp() const 
    {
        return time_stamp_;
    }

    /** \brief Reset leaf container. */
    virtual void reset()
    {
        using namespace pcl::common;

        point_counter_ = 0;
        point_centroid_ *= 0.0f;
        // do not reset time here!
    }

private:
    unsigned int point_counter_;
    PointT point_centroid_;
    std::uint64_t time_stamp_;
};


template<typename PointT, typename LeafContainerT = OctreePointCloudVoxelCentroidContainerMV<PointT>,//OctreeCentroidContainer, //OctreePointCloudVoxelCentroidContainerMV<PointT>,
typename BranchContainerT = OctreeContainerEmpty,
typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> >
class OctreePointCloudCentroid : public OctreePointCloud<PointT, LeafContainerT,
BranchContainerT, OctreeT>
{
public:
    // public typedefs for single/double buffering
    typedef OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT,
    OctreeBase<LeafContainerT, BranchContainerT> > SingleBuffer;

    typedef OctreeBranchNode<BranchContainerT> BranchNode;
    typedef OctreeLeafNode<LeafContainerT> LeafNode;

    typedef BranchContainerT BranchContainer;
    typedef LeafContainerT LeafContainer;

    /** \brief Constructor.
     *  \param resolution_arg: octree resolution at lowest octree level
     * */
    OctreePointCloudCentroid(const double resolution_arg) :
    OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> (resolution_arg)
    {
    }

    /** \brief Empty class constructor. */
    virtual ~OctreePointCloudCentroid()
    {
    }

    void addPointIdx(const int point_idx_arg, const std::uint64_t& time_stamp);
    void addPointsFromInputCloud();
    
    void reinserPointIdx(const int point_idx_arg, const std::uint64_t& time_stamp);    
    void reinsertPointsFromInputCloud();
    
    
    LeafContainerT* findLeafAtPointPublic (const PointT& point_arg) const { return this->findLeafAtPoint(point_arg); }
    

    int boxSearch(const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt, pcl::PointCloud<PointT>& cloud) const;

    void boxSearchRecursive(const Eigen::Vector3f &min_pt,
                            const Eigen::Vector3f &max_pt, const BranchNode* node,
                            const OctreeKey& key,
                            unsigned int tree_depth,
                            pcl::PointCloud<PointT>& cloud) const;


//    void boxSearchRecursive(const Eigen::Vector3f &min_pt,
//                            const Eigen::Vector3f &max_pt,
//                            const BranchNode* node,
//                            const OctreeKey& key,
//                            unsigned int tree_depth,
//                            pcl::PointCloud<PointT>& cloud,
//                            unsigned int search_level) const;

};

template class OctreePointCloudCentroid<pcl::PointXYZRGBA>;
template class OctreePointCloudCentroid<pcl::PointXYZRGBNormal>;
template class OctreePointCloudCentroid<pcl::PointSurfelSegment>;

}//namespace octree
}//namespace pcl




//class OctreeCentroidContainer : public pcl::octree::OctreeContainerBase
//{
//    static const unsigned int kMaxPointCounterValForMovingAverage = 10;
//
//public:
//
//    /** \brief Class initialization. */
//    OctreeCentroidContainer()
//    {
//        this->reset();
//    }
//
//    /** \brief Empty class deconstructor. */
//    virtual ~OctreeCentroidContainer()
//    {
//    }
//
//    /** \brief deep copy function */
//    virtual OctreeCentroidContainer *
//    deepCopy() const
//    {
//        return (new OctreeCentroidContainer(*this));
//    }
//
//    /** \brief Add new point to voxel. 
//     * \param[in] new_point the new point to add 
//     */
//    void addPoint(const pcl::PointXYZRGBA& new_point, std::uint64_t time_stamp = 0)
//    {
//        const float weight = std::min(point_counter_, kMaxPointCounterValForMovingAverage);
//        const float fc = 1. / (weight + 1.0f);
//
//        x_sum_ = (weight * x_sum_ + new_point.x) * fc;
//        y_sum_ = (weight * y_sum_ + new_point.y) * fc;
//        z_sum_ = (weight * z_sum_ + new_point.z) * fc;
//
//        r_sum_ = (weight * r_sum_ + new_point.r) * fc;
//        g_sum_ = (weight * g_sum_ + new_point.g) * fc;
//        b_sum_ = (weight * b_sum_ + new_point.b) * fc;
//
//        ++point_counter_;
//        time_stamp_ = time_stamp;
//    }
//
//    /** \brief Calculate centroid of voxel. 
//     * \param[out] centroid_arg the resultant centroid of the voxel 
//     */
//    //    void getCentroid(pcl::PointXYZRGBA& centroid_arg) const
//    //    {
//    //        if (point_counter_)
//    //        {
//    //            float fc = static_cast<float> (point_counter_);
//    //            centroid_arg.x = x_sum_ / fc;
//    //            centroid_arg.y = y_sum_ / fc;
//    //            centroid_arg.z = z_sum_ / fc;
//    //
//    //            centroid_arg.r = r_sum_ / fc;
//    //            centroid_arg.g = g_sum_ / fc;
//    //            centroid_arg.b = b_sum_ / fc;
//    //        }
//    //    }
//
//    void getCentroid(pcl::PointXYZRGBA& centroid) const
//    {
//        centroid.x = x_sum_;
//        centroid.y = y_sum_;
//        centroid.z = z_sum_;
//
//        centroid.r = r_sum_;
//        centroid.g = g_sum_;
//        centroid.b = b_sum_;
//    }
//
//    int getCounter()
//    {
//        return point_counter_;
//    }
//
//    /** \brief Calculate centroid of voxel. 
//     * \param[out] centroid_arg the resultant centroid of the voxel 
//     */
//    std::uint64_t getTimeStamp() const
//    {
//        return time_stamp_;
//    }
//
//    /** \brief Reset leaf container. */
//    virtual void reset()
//    {
//        point_counter_ = 0;
//
//        r_sum_ = 0.0;
//        g_sum_ = 0.0;
//        b_sum_ = 0.0;
//
//        x_sum_ = y_sum_ = z_sum_ = 0.0;
//
//        time_stamp_ = 0;
//
//    }
//
//private:
//    unsigned int point_counter_;
//
//    unsigned int r_sum_;
//    unsigned int g_sum_;
//    unsigned int b_sum_;
//
//    float x_sum_;
//    float y_sum_;
//    float z_sum_;
//
//    std::uint64_t time_stamp_;
//};

#endif 