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

#include <towr/constraints/terrain_constraint.h>

#include <iostream>
#include <iomanip>
#include <cmath>

namespace towr {


TerrainConstraint::TerrainConstraint (const HeightMap::Ptr& terrain,
									  const NodeSpline::Ptr& spline,
                                      std::string ee_motion, double dt)
    :ConstraintSet(kSpecifyLater, "terrain-" + ee_motion)
{
  ee_motion_id_ = ee_motion;
  terrain_ = terrain;

  spline_ = spline;

  n_polys_ = spline->GetPolynomialCount();
  T_ = spline->GetPolyDurations();


  dt_ = dt;
}

void
TerrainConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(ee_motion_id_);

  swing_node_ids_ = ee_motion_->GetIndicesOfNonConstantNodes();
//  std::cout << "swing nodes: ";
//  for (int id : swing_node_ids_) {
//	  std::cout << id << " ";
//  }
//  std::cout << std::endl;

  // discretized time constraint on stance polynomials
  std::cout << "ee" << ee_motion_id_[10] << "_terrain constrained times: ";
  dts_.push_back(0.0);
  std::cout << 0.0 << " ";
  for (int id=0; id<n_polys_; ++id)
  {
	  if (ee_motion_->IsInConstantPhase(id)) {
		  double t_local = 0.0;
		  double T = T_.at(id);
		  for (int i=0; i<(floor(T/dt_)+1); ++i) {
			  double t_global = spline_->GetGlobalTime(id, t_local);
			  if (t_global > dts_.back()) {
				  dts_.push_back(t_global);
				  std::cout << t_global << " ";
			  }
			  t_local += dt_;
		  }
		  double t_end = spline_->GetGlobalTime(id, T);
		  if (t_end > dts_.back()) {
			  dts_.push_back(t_end);
			  std::cout << t_end << " ";
		  }
	  }
  }
  std::cout << std::endl;

  int constraint_count = swing_node_ids_.size() + dts_.size();
  std::cout << "number of constraints: " << constraint_count << std::endl;
  SetRows(constraint_count);
}

Eigen::VectorXd
TerrainConstraint::GetValues () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();

  int row = 0;
  for (int id : swing_node_ids_) {
    Vector3d p = nodes.at(id).p();
    g(row++) = p.z() - terrain_->GetHeight(p.x(), p.y());
  }

  for (double t : dts_) {
    Vector3d p = spline_->GetPoint(t).p();
    g(row++) = p.z() - terrain_->GetHeight(p.x(), p.y());
  }

  return g;
}

TerrainConstraint::VecBound
TerrainConstraint::GetBounds () const
{
  VecBound bounds(GetRows());
  double max_distance_above_terrain = 1e20; // [m]

  int row = 0;

  // constraint for swing nodes
  for (int id : swing_node_ids_) {
	bounds.at(row) = ifopt::Bounds(0.0, max_distance_above_terrain);
	row++;
  }

  for (double t : dts_) {
    bounds.at(row) = ifopt::Bounds(0.0, 0.0);
	row++;
  }

  return bounds;
}

TerrainConstraint::Jacobian
TerrainConstraint::GetJacobianTerrainHeight(double x, double y) const
{
  Jacobian jac = Eigen::Vector3d::Ones().transpose().sparseView();

  jac.coeffRef(0, X_) = -terrain_->GetDerivativeOfHeightWrt(X_, x, y);
  jac.coeffRef(0, Y_) = -terrain_->GetDerivativeOfHeightWrt(Y_, x, y);

  return jac;
}

void
TerrainConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {
    auto nodes = ee_motion_->GetNodes();
    int row = 0;

    for (int id : swing_node_ids_) {
      int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, Z));
      jac.coeffRef(row, idx) = 1.0;

      Vector3d p = nodes.at(id).p();
      for (auto dim : {X,Y}) {
        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, dim));
        jac.coeffRef(row, idx) = -terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x(), p.y());
      }
      row++;
    }

    for (double t : dts_) {
    	Vector3d pos = spline_->GetPoint(t).p();
    	jac.row(row) = GetJacobianTerrainHeight(pos.x(), pos.y()) * spline_->GetJacobianWrtNodes(t, kPos);
    	row++;
    }

  }
}

} /* namespace towr */
