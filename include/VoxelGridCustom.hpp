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

#ifndef VOXEL_GRID_CUSTOM_HPP
#define VOXEL_GRID_CUSTOM_HPP


#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

#ifndef pcl_isfinite
#include <cmath> // For std::isfinite
#define pcl_isfinite(x) std::isfinite(x)
#endif


inline void pcl::VoxelGridCustom<pcl::PointSurfelSegment>::applyFilter (PointCloud &output)
{
  // Has the input dataset been set already?
  if (!input_)
  {
    PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height       = 1;                    // downsampling breaks the organized structure
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D<PointT> (input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
  {
    PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    output = *input_;
    return;
  }

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  // Set up the division multiplier
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

  int centroid_size = 4;
  if (downsample_all_data_)
    centroid_size = boost::mpl::size<FieldList>::value;

  // ---[ RGB special case
  std::vector<pcl::PCLPointField> fields;
  int rgba_index = -1;
  rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
  if (rgba_index == -1)
    rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
  if (rgba_index >= 0)
  {
    rgba_index = fields[rgba_index].offset;
    centroid_size += 3;
  }
  int kfid_index = -1;
  kfid_index = pcl::getFieldIndex (*input_, "kfid", fields);  

  std::vector<cloud_point_index_idx> index_vector;
  index_vector.reserve (indices_->size ());

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
    if (distance_idx == -1)
      PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName ().c_str (), distance_idx);

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!pcl_isfinite (input_->points[*it].x) || 
            !pcl_isfinite (input_->points[*it].y) || 
            !pcl_isfinite (input_->points[*it].z))
          continue;

      // Get the distance value
      const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input_->points[*it]);
      float distance_value = 0;
      memcpy (&distance_value, pt_data + fields[distance_idx].offset, sizeof (float));

      if (filter_limit_negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
          continue;
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
          continue;
      }
      
      int ijk0 = static_cast<int> (floor (input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
      int ijk1 = static_cast<int> (floor (input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
      int ijk2 = static_cast<int> (floor (input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.push_back (cloud_point_index_idx (static_cast<unsigned int> (idx), *it));
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
    {
      if (!input_->is_dense)
        // Check if the point is invalid
        if (!pcl_isfinite (input_->points[*it].x) || 
            !pcl_isfinite (input_->points[*it].y) || 
            !pcl_isfinite (input_->points[*it].z))
          continue;

      int ijk0 = static_cast<int> (floor (input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
      int ijk1 = static_cast<int> (floor (input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
      int ijk2 = static_cast<int> (floor (input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.push_back (cloud_point_index_idx (static_cast<unsigned int> (idx), *it));
    }
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  unsigned int total = 0;
  unsigned int index = 0;
  // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
  // index_vector belonging to the voxel which corresponds to the i-th output point,
  // and of the first point not belonging to.
  std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
  // Worst case size
  first_and_last_indices_vector.reserve (index_vector.size ());
  while (index < index_vector.size ()) 
  {
    unsigned int i = index + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
      ++i;
    if (i - index >= min_points_per_voxel_)
    {
      ++total;
      first_and_last_indices_vector.push_back (std::pair<unsigned int, unsigned int> (index, i));
    }
    index = i;
  }

  // Fourth pass: compute centroids, insert them into their final position
  output.points.resize (total);
  if (save_leaf_layout_)
  {
    try
    { 
      // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
      uint32_t new_layout_size = div_b_[0]*div_b_[1]*div_b_[2];
      //This is the number of elements that need to be re-initialized to -1
      uint32_t reinit_size = std::min (static_cast<unsigned int> (new_layout_size), static_cast<unsigned int> (leaf_layout_.size()));
      for (uint32_t i = 0; i < reinit_size; i++)
      {
        leaf_layout_[i] = -1;
      }        
      leaf_layout_.resize (new_layout_size, -1);           
    }
    catch (std::bad_alloc&)
    {
      throw PCLException("VoxelGridCustom bin size is too low; impossible to allocate memory for layout", 
        "voxel_grid.hpp", "applyFilter");	
    }
    catch (std::length_error&)
    {
      throw PCLException("VoxelGridCustom bin size is too low; impossible to allocate memory for layout", 
        "voxel_grid.hpp", "applyFilter");	
    }
  }
  
  index = 0;
  Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
  Eigen::VectorXf temporary = Eigen::VectorXf::Zero (centroid_size);

  for (unsigned int cp = 0; cp < first_and_last_indices_vector.size (); ++cp)
  {
    // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
    unsigned int first_index = first_and_last_indices_vector[cp].first;
    unsigned int last_index = first_and_last_indices_vector[cp].second;
    
    uint32_t kfid = input_->points[index_vector[first_index].cloud_point_index].kfid; // custom part: do not average kfids (keep the kfid of the first point in the voxel)
    // TODO: better solution is to keep the most voted kfid 
    
    if (!downsample_all_data_) 
    {
      centroid[0] = input_->points[index_vector[first_index].cloud_point_index].x;
      centroid[1] = input_->points[index_vector[first_index].cloud_point_index].y;
      centroid[2] = input_->points[index_vector[first_index].cloud_point_index].z;
    }
    else 
    {
      // ---[ RGB special case
      if (rgba_index >= 0)
      {
        // Fill r/g/b data, assuming that the order is BGRA
        pcl::RGB rgb;
        memcpy (&rgb, reinterpret_cast<const char*> (&input_->points[index_vector[first_index].cloud_point_index]) + rgba_index, sizeof (RGB));
        centroid[centroid_size-3] = rgb.r;
        centroid[centroid_size-2] = rgb.g;
        centroid[centroid_size-1] = rgb.b;
      }
      pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <PointT> (input_->points[index_vector[first_index].cloud_point_index], centroid));
    }

    for (unsigned int i = first_index + 1; i < last_index; ++i) 
    {
      if (!downsample_all_data_) 
      {
        centroid[0] += input_->points[index_vector[i].cloud_point_index].x;
        centroid[1] += input_->points[index_vector[i].cloud_point_index].y;
        centroid[2] += input_->points[index_vector[i].cloud_point_index].z;
      }
      else 
      {
        // ---[ RGB special case
        if (rgba_index >= 0)
        {
          // Fill r/g/b data, assuming that the order is BGRA
          pcl::RGB rgb;
          memcpy (&rgb, reinterpret_cast<const char*> (&input_->points[index_vector[i].cloud_point_index]) + rgba_index, sizeof (RGB));
          temporary[centroid_size-3] = rgb.r;
          temporary[centroid_size-2] = rgb.g;
          temporary[centroid_size-1] = rgb.b;
        }
        pcl::for_each_type <FieldList> (NdCopyPointEigenFunctor <PointT> (input_->points[index_vector[i].cloud_point_index], temporary));
        centroid += temporary;
      }
    }

    // index is centroid final position in resulting PointCloud
    if (save_leaf_layout_)
      leaf_layout_[index_vector[first_index].idx] = index;

    centroid /= static_cast<float> (last_index - first_index);

    // store centroid
    // Do we need to process all the fields?
    if (!downsample_all_data_) 
    {
      output.points[index].x = centroid[0];
      output.points[index].y = centroid[1];
      output.points[index].z = centroid[2];
    }
    else 
    {
      pcl::for_each_type<FieldList> (pcl::NdCopyEigenPointFunctor <PointT> (centroid, output.points[index]));
      // ---[ RGB special case
      if (rgba_index >= 0) 
      {
        // pack r/g/b into rgb
        float r = centroid[centroid_size-3], g = centroid[centroid_size-2], b = centroid[centroid_size-1];
        int rgb = (static_cast<int> (r) << 16) | (static_cast<int> (g) << 8) | static_cast<int> (b);
        memcpy (reinterpret_cast<char*> (&output.points[index]) + rgba_index, &rgb, sizeof (float));
      }
    }
    
    output.points[index].kfid = kfid; // custom part: do not average kfids (keep the kfid of the first point in the voxel)
    // TODO: better solution is to keep the most voted kfid 
    
    ++index;
  }
  output.width = static_cast<uint32_t> (output.points.size ());
}

#endif 