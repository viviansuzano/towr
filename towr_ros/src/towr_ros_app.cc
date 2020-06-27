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

#include <towr/initialization/gait_generator.h>
#include <towr_ros/towr_ros_interface.h>

#include "yaml_tools/yaml_tools.hpp"

namespace towr {

/**
 * @brief An example application of using TOWR together with ROS.
 *
 * Build your own application with your own formulation using the building
 * blocks provided in TOWR and following the example below.
 */
class TowrRosApp : public TowrRosInterface {
public:

  using VecTimes = std::vector<double>;

  TowrRosApp (std::string config_file)
  {
	config_file_ = config_file;

	initial_position_ = Eigen::Vector3d::Zero();

	goal_geom_.lin.p_.setZero();
	goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''

  }

  void PrintPhaseDurations(std::vector<VecTimes> phase_durations) const
  {
    std::cout << "Initial Phase Durations: \n";
    for (int ee = 0; ee < phase_durations.size(); ++ee) {
      std::cout << "EE " << ee << " : ";
      for (int i = 0; i < phase_durations.at(ee).size(); i++)
        std::cout << phase_durations.at(ee).at(i) << " ";
      std::cout << std::endl;
    }

  }

  Eigen::Matrix3d GetRotationMatrixBaseToWorld (const Eigen::Vector3d& xyz)
  {
    double x = xyz(X);
    double y = xyz(Y);
    double z = xyz(Z);

    Eigen::Matrix3d M;
    //  http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf (Euler ZYX)
    M << cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
         cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x),
               -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y);

    return M;
  }

  /**
   * @brief Sets the feet to nominal position on flat ground and base above.
   */
  void SetTowrInitialStateFromFile()
  {
	formulation_.initial_base_.lin.at(kPos) = initial_position_;

    // initial feet position
    int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
    auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();
    formulation_.initial_ee_W_.resize(n_ee);
    for (int ee = 0; ee < n_ee; ++ee) {
  	  formulation_.initial_ee_W_.at(ee) = nominal_stance_B.at(ee) + formulation_.initial_base_.lin.at(kPos);
  	  Eigen::Vector3d ee_pos = formulation_.initial_ee_W_.at(ee);
  	  formulation_.initial_ee_W_.at(ee)(Z) = formulation_.terrain_->GetHeight(ee_pos.x(), ee_pos.y());
    }
  }

  /**
   * @brief Sets the parameters required to formulate the TOWR problem.
   */
  Parameters GetTowrParameters(int n_ee, const TowrCommandMsg& msg) const override
  {
    Parameters params;

    yaml_tools::YamlNode basenode = yaml_tools::YamlNode::fromFile(config_file_);
    if (basenode.isNull())
    	throw std::runtime_error("CONFIGURATION LOADING FAILED");

    std::string terrain = basenode["terrain"].as<std::string>();
    auto towr_terrain_id = terrain_ids.find(terrain)->second;

    // Instead of manually defining the initial durations for each foot and
    // step, for convenience we use a GaitGenerator with some predefined gaits
    // for a variety of robots (walk, trot, pace, ...).
    auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
    auto id_gait   = static_cast<GaitGenerator::Combos>(msg.gait);
    gait_gen_->SetCombo(id_gait);
    for (int ee=0; ee<n_ee; ++ee) {
      params.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(msg.total_duration, ee));
      params.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }

    PrintPhaseDurations(params.ee_phase_durations_);

    // Here you can also add other constraints or change parameters
    // params.constraints_.push_back(Parameters::BaseRom);

    // increases optimization time, but sometimes helps find a solution for
    // more difficult terrain.
    if (msg.optimize_phase_durations)
      params.OptimizePhaseDurations();

    bool constrain_final_z_base = basenode["constrain_final_z_base"].as<bool>();
    if (constrain_final_z_base)
    {
    	params.bounds_final_lin_pos_ = {X, Y, Z};
    }

    params.is_pure_driving_motion_ = false;
    if (msg.gait == towr::GaitGenerator::DRIVE) {
	  std::cout << "PARAM is_pure_driving_motion_ TRUE!!!" << std::endl;
      params.is_pure_driving_motion_ = true;
    }

    bool use_non_holonomic_constraint = basenode[terrain]["use_non_holonomic_constraint"].as<bool>();
    if (use_non_holonomic_constraint)
  	  params.SetNonHolonomicConstraint();

    double max_wheels_acc_z = basenode[terrain]["max_wheels_acc_z"].as<double>();
    params.max_wheels_acc_.at(Z) = max_wheels_acc_z;

    bool constrain_base_acc = basenode[terrain]["constrain_base_acc"].as<bool>();
    if (constrain_base_acc)
  	  params.SetBaseAccLimitsContraint();

    return params;
  }

  towr_ros::TowrCommand BuildTowrCommandMsg () override
  {
    towr_ros::TowrCommand msg;

    yaml_tools::YamlNode basenode = yaml_tools::YamlNode::fromFile(config_file_);

    if (basenode.isNull())
    	throw std::runtime_error("CONFIGURATION LOADING FAILED");

    std::string terrain = basenode["terrain"].as<std::string>();
    auto towr_terrain_id = terrain_ids.find(terrain)->second;

    // initial position now comes from the file
    initial_position_.x() = basenode[terrain]["initial_pose"]["x"].as<double>();
    initial_position_.y() = basenode[terrain]["initial_pose"]["y"].as<double>();
    initial_position_.z() = basenode[terrain]["initial_pose"]["z"].as<double>();

    bool init_state_from_file = basenode["init_state_from_file"].as<bool>();
    if (init_state_from_file) {
      formulation_.terrain_ = HeightMap::MakeTerrain(towr_terrain_id);
      SetTowrInitialStateFromFile();
    }

    Eigen::Vector3d final_position_;
    final_position_.x() = basenode[terrain]["final_pose"]["x"].as<double>();
    final_position_.y() = basenode[terrain]["final_pose"]["y"].as<double>();
    final_position_.z() = basenode[terrain]["final_pose"]["z"].as<double>();
    goal_geom_.lin.p_ = final_position_;  // x, y, z

    double final_yaw_angle = basenode[terrain]["final_yaw_angle"].as<double>();

    //goal_geom_.ang.p_ = Vector3d(0.0, 0.0, 0.48*M_PI);  // roll, pitch, yaw
    goal_geom_.ang.p_ = Vector3d(0.0, 0.0, final_yaw_angle);  // roll, pitch, yaw
    goal_geom_.lin.p_.x() += formulation_.initial_base_.lin.at(kPos).x();

    bool plot_trajectory = basenode["plot_trajectory"].as<bool>();
    bool play_initialization = basenode["play_initialization"].as<bool>();
    bool optimize_phase_durations = basenode["optimize_phase_durations"].as<bool>();

    float total_duration = basenode[terrain]["total_time"].as<float>();

    bool shift_initialization = basenode[terrain]["shift_initialization"].as<bool>();
    if (shift_initialization)
    {
	  Eigen::Vector3d pos_phase (0.15, 0.0, 0.0);
	  formulation_.initial_ee_W_.at(LF) += pos_phase;
	  formulation_.initial_ee_W_.at(LH) += pos_phase;
	  formulation_.initial_ee_W_.at(RF) -= pos_phase;
	  formulation_.initial_ee_W_.at(RH) -= pos_phase;
    }

    int gait_combo = basenode[terrain]["gait"].as<int>();

    msg.goal_lin                 = xpp::Convert::ToRos(goal_geom_.lin);
    msg.goal_ang                 = xpp::Convert::ToRos(goal_geom_.ang);
    msg.total_duration           = total_duration;
    msg.replay_trajectory        = true;
    msg.play_initialization      = play_initialization;
    msg.replay_speed             = 0.6; //1.0;
    msg.optimize                 = true;
    msg.terrain                  = (int) towr_terrain_id;
    msg.gait                     = gait_combo; //towr::GaitGenerator::C0;
    msg.robot                    = towr::RobotModel::AnymalWheels;
    msg.optimize_phase_durations = optimize_phase_durations;
    msg.plot_trajectory          = plot_trajectory;

    return msg;

  }

  /**
   * @brief Sets the paramters for IPOPT.
   */
  void SetIpoptParameters(const TowrCommandMsg& msg) override
  {
	yaml_tools::YamlNode basenode = yaml_tools::YamlNode::fromFile(config_file_);

	if (basenode.isNull())
	throw std::runtime_error("CONFIGURATION LOADING FAILED");

	bool run_derivative_test = basenode["run_derivative_test"].as<bool>();
	solver_->SetOption("linear_solver", "ma57"); // ma27, ma57, ma77, ma86, ma97
	solver_->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
	solver_->SetOption("max_cpu_time", 10.0); // 3 min
	solver_->SetOption("print_level", 5);

	if (msg.play_initialization)
	  solver_->SetOption("max_iter", 0);
	else
	  solver_->SetOption("max_iter", 3000);

    // derivative test
    if (run_derivative_test) {
      solver_->SetOption("max_iter", 0);
      solver_->SetOption("derivative_test", "first-order");
      solver_->SetOption("print_level", 4);
      solver_->SetOption("derivative_test_tol", 1e-3);
      //solver_->SetOption("derivative_test_perturbation", 1e-4);
      //solver_->SetOption("derivative_test_print_all", "yes");
    }

  }

protected:
  std::string config_file_;
  xpp::State3dEuler goal_geom_;
  Eigen::Vector3d initial_position_;

};

} // namespace towr


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "my_towr_ros_app");

  std::string config_file = ros::package::getPath("towr_ros") + "/config/parameters.yaml";
  towr::TowrRosApp towr_app (config_file);
  ros::spin();

  return 1;
}
