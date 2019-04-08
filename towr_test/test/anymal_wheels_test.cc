/*
 * anymal_wheels_test.cc
 *
 *  Created on: Apr 2, 2019
 *      Author: vivian
 */

#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include "yaml_tools/yaml_tools.hpp"

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/initialization/gait_generator.h>
#include <towr/nlp_formulation_drive.h>
#include <ifopt/ipopt_solver.h>

#include "towr_ros/helpers_towr.h"

using namespace towr;

/**
* @brief Sets the parameters required to formulate the TOWR problem.
*/
bool SetTowrParameters(NlpFormulationDrive *formulation, const std::string& filename, const std::string terrain)
{
  yaml_tools::YamlNode basenode = yaml_tools::YamlNode::FromFile(filename);
  Eigen::Vector3d initial_position_, final_position_;

  if (basenode.IsNull())
	throw std::runtime_error("CONFIGURATION LOADING FAILED");

  initial_position_.x() = basenode[terrain]["initial_pose"]["x"].as<double>();
  initial_position_.y() = basenode[terrain]["initial_pose"]["y"].as<double>();
  initial_position_.z() = basenode[terrain]["initial_pose"]["z"].as<double>();

  final_position_.x() = basenode[terrain]["final_pose"]["x"].as<double>();
  final_position_.y() = basenode[terrain]["final_pose"]["y"].as<double>();
  final_position_.z() = basenode[terrain]["final_pose"]["z"].as<double>();

	// set the initial position of the robot
  formulation->initial_base_.lin.at(kPos) = initial_position_;

  // define the desired goal state of the robot
  formulation->final_base_.lin.at(kPos) = final_position_;

  // Number of end-effectors
  int n_ee = 4;
  formulation->params_drive_.n_ee_ = n_ee;

  // initial feet position
  auto nominal_stance_B = formulation->model_.kinematic_model_->GetNominalStanceInBase();
//  formulation->initial_ee_W_.resize(n_ee);
//  for (int ee = 0; ee < n_ee; ++ee) {
//	  formulation->initial_ee_W_.at(ee) = nominal_stance_B.at(ee) + formulation->initial_base_.lin.at(kPos);
//	  Eigen::Vector3d ee_pos = formulation->initial_ee_W_.at(ee);
//	  formulation->initial_ee_W_.at(ee)(Z) = formulation->terrain_->GetHeight(ee_pos.x(), ee_pos.y());
//  }
  formulation->initial_ee_W_ = nominal_stance_B;
  std::for_each(formulation->initial_ee_W_.begin(), formulation->initial_ee_W_.end(),
				[&](Eigen::Vector3d& p){ p.z() = 0.0; } // feet at 0 height
  );

  // Time duration of the motion.
  double total_duration = basenode[terrain]["total_time"].as<double>();
  formulation->params_drive_.total_time_ = total_duration;

  return basenode["run_derivative_test"].as<bool>();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "anymal_wheels_to");
  ros::NodeHandle nh;

  // NLP formulation
  NlpFormulationDrive formulation;

  // terrain
  HeightMap::TerrainID terrain_id = HeightMap::StepID;
  formulation.terrain_ = HeightMap::MakeTerrain(terrain_id);

  // Robot Model
  formulation.model_ = RobotModel(RobotModel::AnymalWheels);

  // Load parameters from file
  const std::string config_file = ros::package::getPath("towr_test") + "/config/parameters.yaml";
  const std::string terrain_name = terrain_names.find(terrain_id)->second;
  bool run_derivative_test = SetTowrParameters(&formulation, config_file, terrain_name);

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
  solver->SetOption("linear_solver", "ma97"); // ma57, ma77, ma97
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 180.0); // 3 min
  solver->SetOption("print_level", 5);

  // derivative test
  if (run_derivative_test) {
	solver->SetOption("max_cpu_time", 20.0);
    //solver->SetOption("max_iter", 0);
    solver->SetOption("derivative_test", "first-order");
    solver->SetOption("print_level", 4);
    solver->SetOption("derivative_test_perturbation", 1e-4);
    solver->SetOption("derivative_test_tol", 1e-3);
  }

  solver->Solve(nlp);

  // Print NLP information and some nodes solution
  nlp.PrintCurrent();
//  formulation.PrintSolution(solution, 0.1);

  // save entire trajectory (to send to the controller)
  std::string bag_file = ros::package::getPath("towr_test") + "/bags/anymal_wheels_traj.bag";
  SaveDrivingTrajectoryInRosbag (solution, bag_file);

  // save trajectory as collection of states (for plots)
  bag_file = ros::package::getPath("towr_test") + "/bags/anymal_wheels_states.bag";
  SaveDrivingTrajectoryStatesInRosbag (solution, bag_file);

  // Create a new bag with geometry messages to plot in MATLAB
  ExtractGeometryMessagesFromTrajectoryBag(bag_file);

  return 0;
}







