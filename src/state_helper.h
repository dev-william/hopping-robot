#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"

using namespace drake;

class StateHelper {
public:
	StateHelper(multibody::MultibodyPlant<double>& plantIn, bool is3dIn) : plant(plantIn), is3d(is3dIn) {
		elbowJoint = &plant.GetMutableJointByName<multibody::RevoluteJoint>("elbow");
		springJoint = &plant.GetMutableJointByName<multibody::PrismaticJoint>("spring");
		if(is3d) {
			//shoulderJoint = &plant.GetMutableJointByName<multibody::RevoluteJoint>("shoulder");
		}
		floatingJoint = &plant.GetMutableJointByName<multibody::Joint>("floating");

		elbowActuator = &plant.GetJointActuatorByName(elbowJoint->name());
		springActuator = &plant.GetJointActuatorByName(springJoint->name());

		/*std::vector<multibody::JointActuatorIndex> actuatorIndices = plant.GetJointActuatorIndices(modelIndex);
		plant.GetActuatorNames();
		for(auto actuatorIndex : actuatorIndices) {		//Not sure why there's not a way to get the actuator from the joint
			multibody::JointActuator<double>* actuator = &plant.get_mutable_joint_actuator(actuatorIndex);
			if(&actuator->joint() == elbowJoint)
				elbowActuator = actuator;
			if(&actuator->joint() == springJoint)
				springActuator = actuator;
		}*/
	}
	multibody::MultibodyPlant<double>& plant;
	bool is3d;
	multibody::RevoluteJoint<double>* shoulderJoint = nullptr;
	multibody::RevoluteJoint<double>* elbowJoint = nullptr;
	const multibody::JointActuator<double>* elbowActuator = nullptr;
	multibody::PrismaticJoint<double>* springJoint = nullptr;
	const multibody::JointActuator<double>* springActuator = nullptr;
	multibody::Joint<double>* floatingJoint = nullptr;
};