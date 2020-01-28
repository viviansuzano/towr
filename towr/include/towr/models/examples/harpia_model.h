/*
 * harpia_model.h
 *
 *  Created on: Jan 25, 2020
 *      Author: vivian
 */

#ifndef TOWR_MODELS_EXAMPLES_HARPIA_MODEL_H_
#define TOWR_MODELS_EXAMPLES_HARPIA_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the wheeled Harpia robot.
 */
class HarpiaKinematicModel : public KinematicModel {
public:
  HarpiaKinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b =  0.0849;
    const double y_nominal_b =  0.2975;
    const double z_nominal_b =  0.03175; //-0.0074;

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.0, 0.0, 0.0;
  }

};

/**
 * @brief The Dynamics of the quadruped robot ANYmal with wheels.
 */
class HarpiaDynamicModel : public SingleRigidBodyDynamics {
public:
  HarpiaDynamicModel()
  : SingleRigidBodyDynamics(27.24, 1.39698239, 0.56281087, 1.82100537,
		  	  	  	  	    -0.00160128, -0.01746027, -0.00814363, 4) {}
};

} // namespace towr




#endif /* TOWR_MODELS_EXAMPLES_HARPIA_MODEL_H_ */
