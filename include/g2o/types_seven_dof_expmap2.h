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

#ifdef USE_G2O_NEW
#include "Thirdparty/g2o_new/install/include/g2o/core/eigen_types.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sba/types_sba.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sim3/types_seven_dof_expmap.h"
#else
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#endif 

namespace g2o {
  
  /**
 * \brief Edge between an SE3 Vertex and Sim3 Vertex 
 */
  class EdgeSE3Sim3 : public BaseBinaryEdge<7, Sim3, VertexSE3Expmap, VertexSim3Expmap>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3Sim3();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
      const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

      Sim3 C(_measurement);
      const SE3Quat& v1SE3Estimate = v1->estimate();
      Sim3 v1Sim3Estimate = Sim3(v1SE3Estimate.rotation(), v1SE3Estimate.translation(), 1.);
      Sim3 error_=C*v1Sim3Estimate*v2->estimate().inverse();
      _error = error_.log();
    }

//    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
//    {
//      VertexSE3Expmap* v1 = static_cast<VertexSE3Expmap*>(_vertices[0]);
//      VertexSim3Expmap* v2 = static_cast<VertexSim3Expmap*>(_vertices[1]);
//      if (from.count(v1) > 0)
//        v2->setEstimate(measurement()*v1->estimate());
//      else
//        v1->setEstimate(measurement().inverse()*v2->estimate());
//    }
  };

  
    /**
 * \brief Edge between a Sim3 Vertex and SE3 Vertex 
 */
  class EdgeSim3SE3 : public BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSE3Expmap>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3SE3();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
      const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);      

      Sim3 C(_measurement);
      const SE3Quat& v2SE3Estimate = v2->estimate();
      Sim3 v2Sim3Estimate = Sim3(v2SE3Estimate.rotation(), v2SE3Estimate.translation(), 1.);
      Sim3 error_=C*v1->estimate()*v2Sim3Estimate.inverse();
      _error = error_.log();
    }

//    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
//    {
//      VertexSE3Expmap* v1 = static_cast<VertexSE3Expmap*>(_vertices[0]);
//      VertexSim3Expmap* v2 = static_cast<VertexSim3Expmap*>(_vertices[1]);
//      if (from.count(v1) > 0)
//        v2->setEstimate(measurement()*v1->estimate());
//      else
//        v1->setEstimate(measurement().inverse()*v2->estimate());
//    }
  };

} // end namespace


