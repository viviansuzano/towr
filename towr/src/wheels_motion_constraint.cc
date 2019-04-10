/*
 * wheels_motion_constraint.cc
 *
 *  Created on: Apr 5, 2019
 *      Author: vivian
 */
#include <iostream>
#include <towr/constraints/wheels_motion_constraint.h>
#include <towr/variables/variable_names.h>
#include <towr/variables/cartesian_dimensions.h>
#include <towr/variables/spline_holder.h>

namespace towr {


WheelsMotionConstraint::WheelsMotionConstraint (const KinematicModel::Ptr& model,
												double T, double dt, const EE& ee,
                                                const SplineHolderDrive& spline_holder)
    :TimeDiscretizationConstraint(T, dt, "wheels_motion-" + std::to_string(ee))
{
  base_linear_  = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);
  ee_      		= ee;

  ee_wheels_motion_ = spline_holder.ee_wheels_motion_.at(ee_);

  max_leg_extension_ = 0.45;
  min_leg_extension_ = 0.35;

  nominal_pos_hip_B_ = model->GetHipPositionInBaseFrame().at(ee);

  SetRows(GetNumberOfNodes());   // one constraint per node
}

void
WheelsMotionConstraint::UpdateConstraintAtInstance (double t, int k,
                                                    VectorXd& g) const
{
  Vector3d base_W  = base_linear_->GetPoint(t).p();
  Vector3d pos_ee_W = ee_wheels_motion_->GetPoint(t).p();
  EulerConverter::MatrixSXd w_R_b = base_angular_.GetRotationMatrixBaseToWorld(t);
  Vector3d pos_hip_W = w_R_b*(nominal_pos_hip_B_) + base_W;

  g(k) = (pos_ee_W - pos_hip_W).norm();
}

void
WheelsMotionConstraint::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  bounds.at(k) = ifopt::Bounds(min_leg_extension_,  max_leg_extension_);
}

WheelsMotionConstraint::Vector3d
WheelsMotionConstraint::NormDerivative (const Vector3d& v) const
{
  double q = pow(v.x(),2) + pow(v.y(),2) + pow(v.z(),2);

  // TODO: protection against division by 0
  return Vector3d(v.x()/sqrt(q), v.y()/sqrt(q), v.z()/sqrt(q));
}

int
WheelsMotionConstraint::GetCol(int node, int dim) const
{
  int col = node*k6D+dim;
  int n_opt_variables = ee_wheels_motion_->GetNodeVariablesCount();

  if (col >= n_opt_variables)
    col = (node-1)*k6D+dim;

  return col;
}

void
WheelsMotionConstraint::UpdateJacobianAtInstance (double t, int k,
                                                  std::string var_set,
                                                  Jacobian& jac) const
{
  Vector3d base_W  = base_linear_->GetPoint(t).p();
  Vector3d pos_ee_W = ee_wheels_motion_->GetPoint(t).p();
  EulerConverter::MatrixSXd w_R_b = base_angular_.GetRotationMatrixBaseToWorld(t);
  Vector3d pos_hip_W = w_R_b*(nominal_pos_hip_B_) + base_W;

  Vector3d g_deriv = NormDerivative(pos_ee_W - pos_hip_W);

  if (var_set == id::base_lin_nodes) {
    for (auto dim: {X, Y, Z}) {
	  int col = GetCol(k, dim);
	  jac.coeffRef(k, col) = -g_deriv(dim);
    }
  }

  if (var_set == id::base_ang_nodes) {
	double sum = 0.0;
	for (auto dim: {X, Y, Z}) {
	  int col = GetCol(k, dim);
	  jac.coeffRef(k, col) = -g_deriv.transpose() * base_angular_.DerivOfRotVecMult(t,nominal_pos_hip_B_, false).col(col);
	}
  }

  if (var_set == id::EEWheelsMotionNodes(ee_)) {
	for (auto dim: {X, Y, Z}) {
	  int col = GetCol(k, dim);
	  jac.coeffRef(k, col) = g_deriv(dim);
	}
  }
}

} /* namespace towr */




