/*
 * towr_drive_app.cc
 *
 *  Created on: Jun 11, 2019
 *      Author: vivian
 */

#include <towr_drive/towr_drive_ros_interface.h>

#include <towr/initialization/gait_generator.h>

#include <xpp_states/convert.h>

#include <std_msgs/Float64.h>

#include "yaml_tools/yaml_tools.hpp"

namespace towr {

class TowrDriveRosApp : public TowrDriveRosInterface {
public:

  TowrDriveRosApp (std::string config_file)
  {
	config_file_ = config_file;

	initial_position_ = Eigen::Vector3d::Zero();

	goal_geom_.lin.p_.setZero();
	goal_geom_.ang.p_ << 0.0, 0.0, 0.0; // roll, pitch, yaw angle applied Z->Y'->X''
  }

  static Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond& q)
  {
	double roll, pitch, yaw;

  	// roll (x-axis rotation)
  	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  	roll = atan2(sinr_cosp, cosr_cosp);

  	// pitch (y-axis rotation)
  	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  	if (fabs(sinp) >= 1)
  		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  	else
  		pitch = asin(sinp);

  	// yaw (z-axis rotation)
  	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  	yaw = atan2(siny_cosp, cosy_cosp);

  	return Eigen::Vector3d(roll, pitch, yaw);
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

  void SetTowrInitialStateFromFile()
  {
	formulation_.initial_base_.lin.at(kPos) = initial_position_;
//	formulation_.initial_base_.ang.at(kPos) = Eigen::Vector3d(0.0, 0.0, 0.3491);

	Eigen::Vector3d r_wh = Eigen::Vector3d(0.0, 0.0, -formulation_.params_drive_.wheels_radius_);

    // initial feet position
    int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
    auto wh_center_B = formulation_.model_.kinematic_model_->GetWheelsCenterPositionInBase();
    formulation_.initial_ee_W_.resize(n_ee);
    for (int ee = 0; ee < n_ee; ++ee) {
  	  formulation_.initial_ee_W_.at(ee) = GetRotationMatrixBaseToWorld(formulation_.initial_base_.ang.at(kPos))
  										  * (wh_center_B.at(ee) + r_wh)
  										  + formulation_.initial_base_.lin.at(kPos);
  	  Eigen::Vector3d ee_pos = formulation_.initial_ee_W_.at(ee);
  	  formulation_.initial_ee_W_.at(ee)(Z) = formulation_.terrain_->GetHeight(ee_pos.x(), ee_pos.y());
    }
  }


  ParametersDrive GetTowrDriveParameters(int n_ee, const TowrCommandMsg& msg) const override
  {
    ParametersDrive params_drive;

    yaml_tools::YamlNode basenode = yaml_tools::YamlNode::fromFile(config_file_);

    if (basenode.isNull())
  	throw std::runtime_error("CONFIGURATION LOADING FAILED");

    auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
    std::string terrain = terrain_names.find(terrain_id)->second;

    // Number of end-effectors
    params_drive.n_ee_ = n_ee;
    params_drive.wheels_radius_ = 0.0762;  // 3 inches = 76.2mm

    // Time duration of the motion
    double total_duration = basenode[terrain]["total_time"].as<double>();
    params_drive.total_time_ = total_duration;

    params_drive.force_limit_in_x_direction_ = basenode[terrain]["traction_limit"].as<double>();

    return params_drive;
  }

  towr_ros::TowrCommand BuildTowrCommandMsg () override
  {
    towr_ros::TowrCommand msg;

    yaml_tools::YamlNode basenode = yaml_tools::YamlNode::fromFile(config_file_);

    if (basenode.isNull())
    	throw std::runtime_error("CONFIGURATION LOADING FAILED");

    std::string terrain = basenode["terrain"].as<std::string>();
    auto towr_terrain_id = terrain_ids.find(terrain)->second;

    // initial position now comes from the controller
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
    goal_geom_.lin.p_ = final_position_;

    bool plot_trajectory = basenode["plot_trajectory"].as<bool>();
    bool play_initialization = basenode["play_initialization"].as<bool>();

    float total_duration = basenode[terrain]["total_time"].as<float>();

    msg.goal_lin                 = xpp::Convert::ToRos(goal_geom_.lin);
    msg.goal_ang                 = xpp::Convert::ToRos(goal_geom_.ang);
    msg.total_duration           = total_duration;
    msg.replay_trajectory        = true;
    msg.play_initialization      = play_initialization;
    msg.replay_speed             = 0.6; //1.0;
    msg.optimize                 = true;
    msg.terrain                  = (int) towr_terrain_id;
    msg.gait                     = towr::GaitGenerator::C0;   // it's not used for driving motions!!
    msg.robot                    = towr::RobotModel::Harpia;
    msg.optimize_phase_durations = false;
    msg.plot_trajectory          = plot_trajectory;

    return msg;

  }

  /**
   * @brief Sets the parameters for IPOPT.
   */
  void SetIpoptParameters(const TowrCommandMsg& msg) override
  {
	yaml_tools::YamlNode basenode = yaml_tools::YamlNode::fromFile(config_file_);

	if (basenode.isNull())
	throw std::runtime_error("CONFIGURATION LOADING FAILED");

	bool run_derivative_test = basenode["run_derivative_test"].as<bool>();
    solver_->SetOption("linear_solver", "ma97"); // ma27, ma57, ma77, ma86, ma97
	solver_->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
	solver_->SetOption("max_cpu_time", 60.0); // 1 min
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
  ros::init(argc, argv, "towr_drive_app");

  std::string config_file = ros::package::getPath("towr_drive") + "/config/parameters.yaml";
  towr::TowrDriveRosApp towr_drive_app (config_file);

//  towr_drive_app.OptimizeMotion();

  ros::spin();

  return 1;
}



