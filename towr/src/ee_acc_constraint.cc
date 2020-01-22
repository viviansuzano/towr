/*
 * ee_acc_constraint.cc
 *
 *  Created on: Jan 16, 2020
 *      Author: vivian
 */

#include <towr/constraints/ee_acc_constraint.h>

#include <towr/variables/variable_names.h>

#include <iostream>

namespace towr {

EEAccConstraint::EEAccConstraint (const NodeSpline::Ptr& spline,
                                  std::string node_variable_name)
    :ConstraintSet(kSpecifyLater, "acc-" + node_variable_name)
{
  spline_ = spline;
  ee_motion_id_ = node_variable_name;

  n_dim_ = 1; //spline->GetPoint(0.0).p().rows();
  n_polys_ = spline->GetPolynomialCount();
  T_ = spline->GetPolyDurations();

}

void
EEAccConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(ee_motion_id_);

  // select stance nodes
  auto nodes = ee_motion_->GetNodes();
  for (int id=0; id<nodes.size(); ++id) {
    if (ee_motion_->AdjacentPolyIdsConstant(id)) {
    	stance_node_ids_.push_back(id);
    	stance_poly_ids_.push_back(id-1);
    	stance_poly_ids_.push_back(id);
    }
  }

//  std::cout << "ee" << ee_motion_id_[10] << " ACC stance nodes: ";
//  for (int id : stance_node_ids_)
//	  std::cout << id << " ";
//  std::cout << std::endl;
//
//  std::cout << "ee" << ee_motion_id_[10] << " ACC stance polys: ";
//  for (int id : stance_poly_ids_)
//	  std::cout << id << " ";
//  std::cout << std::endl;

  int num_constraints = n_dim_*stance_node_ids_.size();
  std::cout << "ee" << ee_motion_id_[10] << " ACC num con: " << num_constraints << std::endl;
  SetRows(num_constraints);
}

Eigen::VectorXd
EEAccConstraint::GetValues () const
{
  VectorXd g(GetRows());

  int row = 0;
  for (int i=0; i<stance_poly_ids_.size(); ++i) {
//	std::cout << row << " ";
    int p_prev = stance_poly_ids_.at(i); // id of previous polynomial
    VectorXd acc_prev = spline_->GetPoint(p_prev, T_.at(p_prev)).a();

    int p_next = stance_poly_ids_.at(i+1);
    VectorXd acc_next = spline_->GetPoint(p_next, 0.0).a();

//    std::cout << "(" << p_prev << ", " << p_next << ") ";
    g(row) = acc_prev(0) - acc_next(0);
//    g.segment(j*n_dim_, n_dim_) = acc_prev - acc_next;

    i++;
    row += n_dim_;
  }

  return g;
}

void
EEAccConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == ee_motion_id_) {
	int row = 0;
	for (int i=0; i<stance_poly_ids_.size(); ++i) {
	  int p_prev = stance_poly_ids_.at(i); // id of previous polynomial
	  Jacobian acc_prev = spline_->GetJacobianWrtNodes(p_prev, T_.at(p_prev), kAcc).row(0);

      int p_next = stance_poly_ids_.at(i+1);
      Jacobian acc_next = spline_->GetJacobianWrtNodes(p_next, 0.0, kAcc).row(0);

//	  jac.middleRows(row*n_dim_, n_dim_) = acc_prev - acc_next;
      jac.row(row) = acc_prev - acc_next;

	  i++;
	  row += n_dim_;
	}
  }
}

EEAccConstraint::VecBound
EEAccConstraint::GetBounds () const
{
  return VecBound(GetRows(), ifopt::BoundZero);
}

} /* namespace towr */



