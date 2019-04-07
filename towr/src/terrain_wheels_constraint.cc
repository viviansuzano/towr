/*
 * terrain_wheels_constraint.cc
 *
 *  Created on: Mar 29, 2019
 *      Author: vivian
 */

#include <iostream>
#include <towr/constraints/terrain_wheels_constraint.h>
#include <towr/variables/variable_names.h>

namespace towr {

TerrainWheelsConstraint::TerrainWheelsConstraint (const HeightMap::Ptr& terrain,
        										  double T, double dt, const EE& ee,
												  const SplineHolderDrive& spline_holder)
	:TimeDiscretizationConstraint(T, dt, "terrain-wheels-" + std::to_string(ee))
{
  ee_wheels_motion_ = spline_holder.ee_wheels_motion_.at(ee);
  terrain_ = terrain;
  ee_ = ee;

  SetRows(GetNumberOfNodes());   // one constraint per node
}

void
TerrainWheelsConstraint::UpdateConstraintAtInstance (double t, int k, VectorXd& g) const
{
  Vector3d pos_ee_W = ee_wheels_motion_->GetPoint(t).p();

  g(k) = pos_ee_W.z() - terrain_->GetHeight(pos_ee_W.x(), pos_ee_W.y());
}

void
TerrainWheelsConstraint::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  bounds.at(k) = ifopt::BoundZero;
}

void
TerrainWheelsConstraint::UpdateJacobianAtInstance(double t, int k, std::string var_set, Jacobian& jac) const
{
  if (var_set == id::EEWheelsMotionNodes(ee_)) {
    jac.middleRows(k,1) = ee_wheels_motion_->GetJacobianWrtNodes(t,kPos).row(Z);

    int n_opt_variables = ee_wheels_motion_->GetNodeVariablesCount();
    Vector3d pos_ee_W = ee_wheels_motion_->GetPoint(t).p();

    for (auto dim : {X_,Y_}) {
      int col = k*k6D+dim;
      if (col >= n_opt_variables) col = (k-1)*k6D+dim;
	  jac.coeffRef(k,col) = -terrain_->GetDerivativeOfHeightWrt(dim,pos_ee_W.x(),pos_ee_W.y());
    }
  }
}

} /* namespace towr */



