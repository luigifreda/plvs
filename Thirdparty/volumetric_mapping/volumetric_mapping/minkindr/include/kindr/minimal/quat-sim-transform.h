#ifndef KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_
#define KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_

#include "kindr/minimal/quat-transformation.h"

namespace kindr {
namespace minimal {

// Scale & rotate then translate = scale then transform.
// In particular, the transformation matrix is:
//
//   R*s t     R t   s*I 0
//   0   1  =  0 1 * 0   1
template <typename Scalar>
class QuatSimTransformTemplate {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef QuatTransformationTemplate<Scalar> Transform;
  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vectors;

  // Creates identity similarity transform.
  QuatSimTransformTemplate();

  QuatSimTransformTemplate(const Transform& T_A_B, const Scalar scale_A_B);

  Vectors operator*(const Vectors& rhs) const;

  QuatSimTransformTemplate<Scalar> inverse() const;

  Eigen::Matrix<Scalar, 4, 4> getTransformationMatrix() const;
  Transform getTransform() const { return T_A_B_; }
  Scalar getScale() const { return scale_A_B_; }

private:
  Transform T_A_B_;
  Scalar scale_A_B_;

  template <typename FriendScalar>
  friend std::ostream & operator<<(
      std::ostream &, const QuatSimTransformTemplate<FriendScalar>&);
};

typedef QuatSimTransformTemplate<double> QuatSimTransform;

template<typename Scalar>
std::ostream & operator<<(std::ostream & out,
                          const QuatSimTransformTemplate<Scalar>& sim_3);

}  // namespace minimal
}  // namespace kindr

#include "kindr/minimal/implementation/quat-sim-transform-inl.h"

#endif  // KINDR_MINIMAL_QUAT_SIM_TRANSFORM_H_
