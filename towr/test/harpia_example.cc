/*
 * harpia_example.cc
 *
 *  Created on: Jan 27, 2020
 *      Author: vivian
 */

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <towr/nlp_formulation_drive.h>
#include <ifopt/ipopt_solver.h>

using namespace towr;

void SetTowrParameters(NlpFormulationDrive *formulation)
{
  // set the initial position of the robot
  formulation->initial_base_.lin.at(kPos) << 0.0, 0.0, 0.04445;

  // define the desired goal state of the robot
  formulation->final_base_.lin.at(kPos) << 2.0, 0.0, 0.04445;

  formulation->params_drive_.total_time_ = 2.0;
  formulation->params_drive_.n_ee_ = 4;
}

int main()
{
  // NLP formulation
  NlpFormulationDrive formulation;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0); //(0.0);

  Eigen::Vector3d r_wh = Eigen::Vector3d(0.0, 0.0, -formulation.params_drive_.wheels_radius_);

  // Kinematic limits and dynamic parameters of the Harpia robot
  formulation.model_ = RobotModel(RobotModel::Harpia);
  int n_ee = formulation.model_.kinematic_model_->GetNumberOfEndeffectors();
  auto wh_center_B = formulation.model_.kinematic_model_->GetWheelsCenterPositionInBase();
  formulation.initial_ee_W_.resize(n_ee);
  for (int ee = 0; ee < n_ee; ++ee) {
	  formulation.initial_ee_W_.at(ee) = (wh_center_B.at(ee) + r_wh) + formulation.initial_base_.lin.at(kPos);
	  Eigen::Vector3d ee_pos = formulation.initial_ee_W_.at(ee);
	  formulation.initial_ee_W_.at(ee)(Z) = formulation.terrain_->GetHeight(ee_pos.x(), ee_pos.y());
  }

  // Load TOWR parameters
  SetTowrParameters(&formulation);

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolderDrive solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("linear_solver", "ma97");
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 60.0);
  solver->SetOption("print_level", 5);

//  solver->SetOption("max_iter", 0);
//  solver->SetOption("derivative_test", "first-order");
//  solver->SetOption("print_level", 4);
//  solver->SetOption("derivative_test_tol", 1e-3);
  //solver_->SetOption("derivative_test_perturbation", 1e-4);
  //solver_->SetOption("derivative_test_print_all", "yes");

  solver->Solve(nlp);

  nlp.PrintCurrent();
//  formulation.PrintSolution(solution, 0.1);
}



