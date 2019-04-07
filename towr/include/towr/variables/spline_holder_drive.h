/*
 * spline_holder_drive.h
 *
 *  Created on: Mar 28, 2019
 *      Author: vivian
 */

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_DRIVE_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_DRIVE_H_

#include "node_spline.h"
#include "spline_holder.h"
#include "nodes_variables.h"

namespace towr {

/**
 * SplineHolder for driving motions only. The nodes representing the wheels motion and forces are no longer phase based.
 * Instead, they are optimized in fixed intervals similar to the base variables.
 */
struct SplineHolderDrive
{
  /**
   * @brief Fully construct all splines.
   * @param base_lin  				 The nodes describing the base linear motion.
   * @param base_ang  				 The nodes describing the base angular motion.
   * @param base_poly_durations 	 The durations of each base polynomial.
   * @param ee_wheels_motion 		 The nodes describing the wheels motions.
   * @param ee_wheels_force 		 The nodes describing the wheels forces.
   * @param ee_wheels_poly_durations The durations of each wheel's polynomial.
   */
  SplineHolderDrive (NodesVariables::Ptr base_lin,
                	 NodesVariables::Ptr base_ang,
					 const std::vector<double>& base_poly_durations,
					 std::vector<NodesVariables::Ptr> ee_wheels_motion,
					 std::vector<NodesVariables::Ptr> ee_wheels_force,
					 const std::vector<double>& ee_wheels_poly_durations);

  SplineHolderDrive () = default;

  NodeSpline::Ptr base_linear_;
  NodeSpline::Ptr base_angular_;
  std::vector<NodeSpline::Ptr> ee_wheels_motion_;
  std::vector<NodeSpline::Ptr> ee_wheels_force_;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_SPLINE_HOLDER_DRIVE_H_ */
