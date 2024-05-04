

#include "legged_interface/constraint/FootPositionConstraint.h"
#include <cmath>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FootPositionConstraint::FootPositionConstraint(const SwitchedModelReferenceManager& referenceManager, const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool FootPositionConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FootPositionConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  // vector_t f = vector_t::Zero(4);
  // vector_t pos = vector_t::Zero(2);
  // vector_t constraint_value_world = vector_t::Zero(3);
  // constraint_value_world << 0.4, 0.03, 0;
  // vector_t currentPose = state.segment<6>(6);
  // scalar_t z = currentPose(3);
  // scalar_t c = cos(z);
  // scalar_t s = sin(z);
  // Eigen::Matrix<scalar_t, 3, 3> rotation;
  // rotation.setZero();
  // rotation << c, -s, 0, s, c, 0, 0, 0, 1;
  // constraint_value_world = rotation.transpose() * constraint_value_world;
  // pos = endEffectorKinematicsPtr_->getPosition(state).front().segment<3>(0)-currentPose.segment<3>(0);
  // f.segment<2>(0) = pos.segment<2>(0) + constraint_value_world.segment<2>(0);
  // f.segment<2>(2) = -pos.segment<2>(0) + constraint_value_world.segment<2>(0);

  vector_t f = vector_t::Zero(4);
  vector_t pos = vector_t::Zero(2);
  vector_t constraint_value_local = vector_t::Zero(3);
  constraint_value_local << 0.4, 0.03, 0;
  vector_t currentPose = state.segment<6>(6);
  scalar_t z = currentPose(3);
  scalar_t c = cos(z);
  scalar_t s = sin(z);
  Eigen::Matrix<scalar_t, 3, 3> rotation;
  rotation.setZero();
  rotation << c, -s, 0, s, c, 0, 0, 0, 1;
  rotation = rotation.transpose();
  pos = rotation * (endEffectorKinematicsPtr_->getPosition(state).front().segment<3>(0)-currentPose.segment<3>(0));
  f.segment<2>(0) = pos.segment<2>(0) + constraint_value_local.segment<2>(0);
  f.segment<2>(2) = -pos.segment<2>(0) + constraint_value_local.segment<2>(0);
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation FootPositionConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                                 const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
  vector_t currentPose = state.segment<6>(6);
  scalar_t z = state(9);
  scalar_t c = cos(z);
  scalar_t s = sin(z);
  Eigen::Matrix<scalar_t, 3, 3> rotation;
  rotation.setZero();
  rotation << c, -s, 0, s, c, 0, 0, 0, 1;
  rotation = rotation.transpose();
  auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  positionApprox.dfdx = rotation * positionApprox.dfdx;
  linearApproximation.f = getValue(time, state, input, preComp);
  linearApproximation.dfdx.block<2,27>(0,0) = positionApprox.dfdx.block<2,27>(0,0);
  linearApproximation.dfdx.block<2,27>(2,0) = -positionApprox.dfdx.block<2,27>(0,0);
  linearApproximation.dfdu = matrix_t::Zero(4, input.size());
  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
