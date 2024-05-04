

#include "legged_interface/constraint/ArmPositionConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ArmPositionConstraint::ArmPositionConstraint(const SwitchedModelReferenceManager& referenceManager, const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ArmPositionConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ArmPositionConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  vector_t f = vector_t::Zero(3);
  f<<0.0, 0.0, -0.5;
  vector_t pos_vector = vector_t::Zero(3);
  vector_t currentPose = state.segment<6>(6);
  // Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  // df = getRotationMatrixFromZyxEulerAngles(zyx)*(endEffectorKinematicsPtr_->getPosition(state).front()-currentPose.head(3));
  pos_vector = (endEffectorKinematicsPtr_->getPosition(state).front()-currentPose.head(3));
  pos_vector(2) += currentPose(2);
  f.noalias() += pos_vector;
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ArmPositionConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                                 const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  linearApproximation.f = getValue(time, state, input, preComp);
  linearApproximation.dfdx.noalias() = positionApprox.dfdx;
  linearApproximation.dfdu = matrix_t::Zero(3, input.size());
  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
