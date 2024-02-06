/**
 * @file FootRollConstraint.h
 * @author xuejialong
 */

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include "humanoid_interface/reference_manager/SwitchedModelReferenceManager.h"



namespace ocs2 {
namespace humanoid {

/**
 * Defines a linear constraint on an end-effector position (xee) and linear velocity (vee).
 * g(xee, vee) = Ax * xee + Av * vee + b
 * - For defining constraint of type g(xee), set Av to matrix_t(0, 0)
 * - For defining constraint of type g(vee), set Ax to matrix_t(0, 0)
 */
class FootRollConstraint final : public StateInputConstraint {
 public:

  /**
   * Constructor
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] numConstraints: The number of constraints {1, 2, 3}
   */
  FootRollConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex,
                     CentroidalModelInfo info);

  ~FootRollConstraint() override = default;
  FootRollConstraint* clone() const override { return new FootRollConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  FootRollConstraint(const FootRollConstraint& rhs);

    const SwitchedModelReferenceManager* referenceManagerPtr_;
    const size_t contactPointIndex_;
    const CentroidalModelInfo info_;
};

}  // namespace humanoid
}  // namespace ocs2
