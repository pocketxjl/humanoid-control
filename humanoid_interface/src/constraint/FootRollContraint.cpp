//
// Created by pocket on 24-2-4.
//
#include "humanoid_interface/constraint/FootRollConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace humanoid {
    FootRollConstraint::FootRollConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex,CentroidalModelInfo info)
            : StateInputConstraint(ConstraintOrder::Linear),
              referenceManagerPtr_(&referenceManager),
              contactPointIndex_(contactPointIndex),
              info_(std::move(info)) {}

    FootRollConstraint::FootRollConstraint(const FootRollConstraint& rhs)
            : StateInputConstraint(rhs),
                referenceManagerPtr_(rhs.referenceManagerPtr_),
                contactPointIndex_(rhs.contactPointIndex_),
                info_(rhs.info_) {}

    bool FootRollConstraint::isActive(scalar_t time) const {
//        return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
        return true;
    }


    vector_t FootRollConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                   const PreComputation& preComp) const {
        //获取最后一个关节的速度
        long index = 5 + (contactPointIndex_ / 2) * 6;
        return centroidal_model::getJointVelocities(input, info_).block<-1, 1>(index, 0, 1, 1);
    }


    VectorFunctionLinearApproximation FootRollConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                          const vector_t& input,
                                                                                          const PreComputation& preComp) const {
        VectorFunctionLinearApproximation approx;
        approx.f = getValue(time, state, input, preComp);
        approx.dfdx = matrix_t::Zero(1, state.size());
        approx.dfdu = matrix_t::Zero(1, input.size());
        approx.dfdu.middleCols<1>(17 + (contactPointIndex_ / 2) * 6).diagonal() = vector_t::Ones(1);
        return approx;
    }
}
}
