// This file is part of PLVS
// Copyright (C) 2018-present Luigi Freda <luigifreda at gmail dot com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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


