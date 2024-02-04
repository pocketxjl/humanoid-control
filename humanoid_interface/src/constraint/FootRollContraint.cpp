//
// Created by pocket on 24-2-4.
//
#include "humanoid_interface/constraint/FootRollConstraint.h"

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <utility>
#include <random>

namespace ocs2 {
namespace humanoid {
    FootRollConstraint::FootRollConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                             double Gain)
            : StateInputConstraint(ConstraintOrder::Linear),
              endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
                Gain_(Gain)
   {
        if (endEffectorKinematicsPtr_->getIds().size() != 1) {
            throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
        }
    }

    FootRollConstraint::FootRollConstraint(const FootRollConstraint& rhs)
            : StateInputConstraint(rhs),
                Gain_(rhs.Gain_),
              endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()){}


    vector_t FootRollConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                   const PreComputation& preComp) const {
        //get the orientation of end effector
        //创建一个四元数，方向向前
        Eigen::Quaternion<scalar_t> q;
        q.w() = 1;
        q.x() = 0;
        q.y() = 0;
        q.z() = 0;
        auto referenceQuaternion = {q};

        vector_t f = endEffectorKinematicsPtr_->getOrientationError(state, referenceQuaternion).front();
        Eigen::Matrix<scalar_t, 3, 3> I = Eigen::Matrix<scalar_t, 3, 3>::Identity();
        I(2, 2) = 0;

        return Gain_ * I * f;
    }


    VectorFunctionLinearApproximation FootRollConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                          const vector_t& input,
                                                                                          const PreComputation& preComp) const {
        //get the orientation of end effector
        //创建一个四元数，方向向前
        Eigen::Quaternion<scalar_t> q;
        q.w() = 1;
        q.x() = 0;
        q.y() = 0;
        q.z() = 0;
        auto referenceQuaternion = {q};

        VectorFunctionLinearApproximation linearApproximation =
                VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
        auto orientationApprox = endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, referenceQuaternion).front();

        //创建一个3x3的单位矩阵
        Eigen::Matrix<scalar_t, 3, 3> I = Eigen::Matrix<scalar_t, 3, 3>::Identity();
        I(2, 2) = 0;

        linearApproximation.f.noalias() = Gain_ * I * orientationApprox.f;
        linearApproximation.dfdx.noalias() = Gain_ * I * orientationApprox.dfdx;

        return linearApproximation;
    }
}
}