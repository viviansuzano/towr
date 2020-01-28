/*
 * wheels_motion_constraint.h
 *
 *  Created on: Jan 27, 2020
 *      Author: vivian
 */

#ifndef TOWR_CONSTRAINTS_WHEELS_MOTION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_WHEELS_MOTION_CONSTRAINT_H_

#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/spline_holder_drive.h>
#include <towr/variables/euler_converter.h>
#include <towr/terrain/height_map.h>

#include <towr/models/kinematic_model.h>

#include "time_discretization_constraint.h"

namespace towr {

/** @brief Constrains an endeffector to lie in a box around the nominal stance.
  *
  * These constraints are necessary to avoid configurations
  * that are outside the kinematic reach of the robot. The constraint
  * is defined by Cartesian estimates of the reachability of each endeffector.
  *
  * This constraint calculates the position of of the contact expressed in the
  * current CoM frame and constrains it to lie in a box around the nominal/
  * natural contact position for that leg.
  *
  * @ingroup Constraints
  */
class WheelsMotionConstraint : public TimeDiscretizationConstraint {
public:
  using EE = uint;
  using Vector3d = Eigen::Vector3d;

  /**
   * @brief Constructs a constraint instance.
   * @param robot_model   The kinematic restrictions of the robot.
   * @param T   The total duration of the optimization.
   * @param dt  the discretization intervall at which to enforce constraints.
   * @param ee            The endeffector for which to constrain the range.
   * @param spline_holder Pointer to the current variables.
   */
  WheelsMotionConstraint(const KinematicModel::Ptr& robot_model,
                          double T, double dt, double wheel_radius,
                          const EE& ee, const HeightMap::Ptr& terrain,
                          const SplineHolder& spline_holder);

  // Allows this constraint to receive a SplineHolderDrive as parameter (for driving motions)
  WheelsMotionConstraint(const KinematicModel::Ptr& robot_model,
                          double T, double dt, double wheel_radius,
                          const EE& ee, const HeightMap::Ptr& terrain,
                          const SplineHolderDrive& spline_holder);
  virtual ~WheelsMotionConstraint() = default;

private:
  NodeSpline::Ptr base_linear_;     ///< the linear position of the base.
  EulerConverter base_angular_; ///< the orientation of the base.
  NodeSpline::Ptr ee_motion_;       ///< the linear position of the endeffectors.

  Eigen::Vector3d wh_center_pos_B_;
  HeightMap::Ptr terrain_;
  EE ee_;

  double wheel_radius_;

  // see TimeDiscretizationConstraint for documentation
  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  void UpdateJacobianAtInstance(double t, int k, std::string, Jacobian&) const override;

  int GetRow(int node, int dimension) const;
  Eigen::Matrix3d GetJacobianTerrainBasis(HeightMap::Direction basis, double x, double y) const;
};

} /* namespace towr */



#endif /* TOWR_CONSTRAINTS_WHEELS_MOTION_CONSTRAINT_H_ */
