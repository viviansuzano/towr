/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/constraints/force_constraint.h>

#include <towr/variables/variable_names.h>

#include <iostream>
#include <cmath>

namespace towr {


ForceConstraint::ForceConstraint (const HeightMap::Ptr& terrain,
        						  double force_limit,
								  EE ee, double dt,
								  const SplineHolder& spline_holder)
    :ifopt::ConstraintSet(kSpecifyLater, "force-" + id::EEForceNodes(ee))
{
  terrain_ = terrain;
  fn_max_  = force_limit;
  mu_      = terrain->GetFrictionCoeff();
  ee_      = ee;

  spline_ee_motion_ = spline_holder.ee_motion_.at(ee_);
  spline_ee_force_  = spline_holder.ee_force_.at(ee_);

  n_polys_ = spline_ee_force_->GetPolynomialCount();
  T_ = spline_ee_force_->GetPolyDurations();

  dt_ = dt;

  n_constraints_per_node_ = 1 + 2*k2D; // positive normal force + 4 friction pyramid constraints
}

void
ForceConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_force_  = x->GetComponent<NodesVariablesPhaseBased>(id::EEForceNodes(ee_));

  // discretized time constraint on stance polynomials
  std::cout << "ee" << ee_ << "_force constrained times: ";
  dts_.push_back(0.0);
  std::cout << 0.0 << " ";
  for (int id=0; id<n_polys_; ++id)
  {
	  if (!ee_force_->IsInConstantPhase(id)) {
		  double t_local = 0.0;
		  double T = T_.at(id);
		  for (int i=0; i<(floor(T/dt_)+1); ++i) {
			  double t_global = spline_ee_force_->GetGlobalTime(id, t_local);
			  if (t_global > dts_.back()) {
				  dts_.push_back(t_global);
				  std::cout << t_global << " ";
			  }
			  t_local += dt_;
		  }
		  double t_end = spline_ee_force_->GetGlobalTime(id, T);
		  if (t_end > dts_.back()) {
			  dts_.push_back(t_end);
			  std::cout << t_end << " ";
		  }
	  }
  }
  std::cout << std::endl;

  int num_const = dts_.size();
  int constraint_count = num_const*n_constraints_per_node_;
  SetRows(constraint_count);
}

Eigen::VectorXd
ForceConstraint::GetValues () const
{
  VectorXd g(GetRows());

  int row=0;
  for (double t : dts_) {
    Vector3d p = spline_ee_motion_->GetPoint(t).p();
    Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, p.x(), p.y());
    Vector3d f = spline_ee_force_->GetPoint(t).p();

    // unilateral force
    g(row++) = f.transpose() * n; // >0 (unilateral forces)

    // frictional pyramid
    Vector3d t1 = terrain_->GetNormalizedBasis(HeightMap::Tangent1, p.x(), p.y());
    g(row++) = f.transpose() * (t1 - mu_*n); // t1 < mu*n
    g(row++) = f.transpose() * (t1 + mu_*n); // t1 > -mu*n

    Vector3d t2 = terrain_->GetNormalizedBasis(HeightMap::Tangent2, p.x(), p.y());
    g(row++) = f.transpose() * (t2 - mu_*n); // t2 < mu*n
    g(row++) = f.transpose() * (t2 + mu_*n); // t2 > -mu*n
  }

  return g;
}

ForceConstraint::VecBound
ForceConstraint::GetBounds () const
{
  VecBound bounds;

  for (double t : dts_) {
    bounds.push_back(ifopt::Bounds(0.0, fn_max_)); // unilateral forces
    bounds.push_back(ifopt::BoundSmallerZero); // f_t1 <  mu*n
    bounds.push_back(ifopt::BoundGreaterZero); // f_t1 > -mu*n
    bounds.push_back(ifopt::BoundSmallerZero); // f_t2 <  mu*n
    bounds.push_back(ifopt::BoundGreaterZero); // f_t2 > -mu*n
  }

  return bounds;
}

Eigen::Matrix3d
ForceConstraint::GetJacobianTerrainBasis(HeightMap::Direction basis, double x, double y) const
{
  Eigen::Matrix3d jac = Eigen::Matrix3d::Ones() * 1e-10;

  jac.col(X_) += terrain_->GetDerivativeOfNormalizedBasisWrt(basis, X_, x, y);
  jac.col(Y_) += terrain_->GetDerivativeOfNormalizedBasisWrt(basis, Y_, x, y);

  return jac;
}

void
ForceConstraint::FillJacobianBlock (std::string var_set,
                                    Jacobian& jac) const
{
  if (var_set == ee_force_->GetName()) {
    int row = 0;
    for (double t : dts_) {
      // unilateral force
      Vector3d p = spline_ee_motion_->GetPoint(t).p();
      Vector3d n  = terrain_->GetNormalizedBasis(HeightMap::Normal,   p.x(), p.y());
      Vector3d t1 = terrain_->GetNormalizedBasis(HeightMap::Tangent1, p.x(), p.y());
      Vector3d t2 = terrain_->GetNormalizedBasis(HeightMap::Tangent2, p.x(), p.y());

	  Jacobian jac_n  = n.transpose().sparseView(1.0,-1.0);
	  Jacobian jac_t1 = t1.transpose().sparseView(1.0,-1.0);
	  Jacobian jac_t2 = t2.transpose().sparseView(1.0,-1.0);

	  Jacobian nodes = spline_ee_force_->GetJacobianWrtNodes(t, kPos);

	  // unilateral force
	  jac.row(row++) = jac_n * nodes; // (n.transpose() * nodes).sparseView();

   	  // friction force tangent 1 derivative
	  jac.row(row++) = (jac_t1-mu_*jac_n) * nodes;
	  jac.row(row++) = (jac_t1+mu_*jac_n) * nodes;

	  // friction force tangent 2 derivative
	  jac.row(row++) = (jac_t2-mu_*jac_n) * nodes;
	  jac.row(row++) = (jac_t2+mu_*jac_n) * nodes;

    }
  }

  if (var_set == id::EEMotionNodes(ee_)) {
    int row = 0;
    for (double t : dts_) {    	// unilateral force
    	Vector3d f = spline_ee_force_->GetPoint(t).p();
    	Vector3d p = spline_ee_motion_->GetPoint(t).p();

    	Jacobian jac_n  = (f.transpose()*GetJacobianTerrainBasis(HeightMap::Normal, p.x(), p.y())).sparseView(1.0,-1.0);
    	Jacobian jac_t1 = (f.transpose()*GetJacobianTerrainBasis(HeightMap::Tangent1, p.x(), p.y())).sparseView(1.0,-1.0);
    	Jacobian jac_t2 = (f.transpose()*GetJacobianTerrainBasis(HeightMap::Tangent2, p.x(), p.y())).sparseView(1.0,-1.0);

    	Jacobian nodes = spline_ee_motion_->GetJacobianWrtNodes(t, kPos);

    	// unilateral force
    	jac.row(row++) = jac_n * nodes;

    	// friction force tangent 1 derivative
    	jac.row(row++) = (jac_t1-mu_*jac_n) * nodes;
    	jac.row(row++) = (jac_t1+mu_*jac_n) * nodes;

    	// friction force tangent 2 derivative
    	jac.row(row++) = (jac_t2-mu_*jac_n) * nodes;
    	jac.row(row++) = (jac_t2+mu_*jac_n) * nodes;
    }
  }

}

} /* namespace towr */
