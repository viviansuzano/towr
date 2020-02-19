/*
 * wheels_motion_constraint.cc
 *
 *  Created on: Jan 27, 2020
 *      Author: vivian
 */

#include <iostream>
#include <towr/constraints/wheels_motion_constraint.h>
#include <towr/variables/variable_names.h>

namespace towr {

WheelsMotionConstraint::WheelsMotionConstraint (const KinematicModel::Ptr& model,
                                                  double T, double dt, double wheel_radius,
                                                  const EE& ee, const HeightMap::Ptr& terrain,
                                                  const SplineHolder& spline_holder)
    :TimeDiscretizationConstraint(T, dt, "wheels-motion-" + std::to_string(ee))
{
  base_linear_  = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);
  ee_motion_    = spline_holder.ee_motion_.at(ee);

  wh_center_pos_B_ = model->GetWheelsCenterPositionInBase().at(ee);
  ee_ = ee;

  terrain_ = terrain;

  wheel_radius_ = wheel_radius;

  SetRows(GetNumberOfNodes()*k3D);
}

WheelsMotionConstraint::WheelsMotionConstraint (const KinematicModel::Ptr& model,
                                                  double T, double dt, double wheel_radius,
                                                  const EE& ee, const HeightMap::Ptr& terrain,
                                                  const SplineHolderDrive& spline_holder)
    :TimeDiscretizationConstraint(T, dt, "wheels-motion-" + std::to_string(ee))
{
  base_linear_  = spline_holder.base_linear_;
  base_angular_ = EulerConverter(spline_holder.base_angular_);
  ee_motion_    = spline_holder.ee_wheels_motion_.at(ee);

  wh_center_pos_B_ = model->GetWheelsCenterPositionInBase().at(ee);
  ee_ = ee;

  terrain_ = terrain;

  wheel_radius_ = wheel_radius;

  SetRows(GetNumberOfNodes()*k3D);
}

int
WheelsMotionConstraint::GetRow (int node, int dim) const
{
  return node*k3D + dim;
}

void
WheelsMotionConstraint::UpdateConstraintAtInstance (double t, int k, VectorXd& g) const
{
  Vector3d base_W  = base_linear_->GetPoint(t).p();
  Vector3d pos_ee_W = ee_motion_->GetPoint(t).p();
  EulerConverter::MatrixSXd w_R_b = base_angular_.GetRotationMatrixBaseToWorld(t);

  Vector3d n  = terrain_->GetNormalizedBasis(HeightMap::Normal, pos_ee_W.x(), pos_ee_W.y());

  Vector3d wh_center_pos_W = base_W + w_R_b*wh_center_pos_B_;

  Vector3d pos_ee_W_calc = wh_center_pos_W - n*wheel_radius_;

  g.middleRows(GetRow(k, X), k3D) = pos_ee_W - pos_ee_W_calc;
}

void
WheelsMotionConstraint::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
//  for (int dim=0; dim<k3D; ++dim) {
//    bounds.at(GetRow(k,dim)) = Bounds(0.005, 0.005);
//  }

//  bounds.at(GetRow(k,X)) = Bounds(-0.005, 0.005);
//  bounds.at(GetRow(k,Y)) = Bounds(-0.005, 0.005);
//  bounds.at(GetRow(k,Z)) = Bounds(-0.005, 0.005);

  bounds.at(GetRow(k,X)) = ifopt::BoundZero;
  bounds.at(GetRow(k,Y)) = ifopt::BoundZero;
  bounds.at(GetRow(k,Z)) = ifopt::BoundZero;
}

Eigen::Matrix3d
WheelsMotionConstraint::GetJacobianTerrainBasis(HeightMap::Direction basis, double x, double y) const
{
  Eigen::Matrix3d jac = Eigen::Matrix3d::Ones() * 1e-10;

  jac.col(X_) += terrain_->GetDerivativeOfNormalizedBasisWrt(basis, X_, x, y);
  jac.col(Y_) += terrain_->GetDerivativeOfNormalizedBasisWrt(basis, Y_, x, y);

  return jac;
}

void
WheelsMotionConstraint::UpdateJacobianAtInstance (double t, int k,
                                                   std::string var_set,
                                                   Jacobian& jac) const
{
  EulerConverter::MatrixSXd w_R_b = base_angular_.GetRotationMatrixBaseToWorld(t);
  int row_start = GetRow(k,X);

  if (var_set == id::base_lin_nodes) {
    jac.middleRows(row_start, k3D) = -base_linear_->GetJacobianWrtNodes(t, kPos);
  }

  if (var_set == id::base_ang_nodes) {
    jac.middleRows(row_start, k3D) = -base_angular_.DerivOfRotVecMult(t, wh_center_pos_B_, false);
  }

  if (var_set == id::EEWheelsMotionNodes(ee_)) {
	Vector3d p = ee_motion_->GetPoint(t).p();
	Jacobian jac_n = (Eigen::Matrix3d::Identity() + wheel_radius_*GetJacobianTerrainBasis(HeightMap::Normal, p.x(), p.y())).sparseView();
	jac.middleRows(row_start, k3D) = jac_n*ee_motion_->GetJacobianWrtNodes(t,kPos);
  }

}

} /* namespace towr */


