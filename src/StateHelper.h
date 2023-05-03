#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"

using namespace drake;

class StateHelper {
public:
	StateHelper(multibody::MultibodyPlant<double>& plantIn) : plant(plantIn) {
		elbowJoint = &plant.GetMutableJointByName<multibody::RevoluteJoint>("elbow");
		springJoint = &plant.GetMutableJointByName<multibody::PrismaticJoint>("spring");

		try {
			floatingJoint = &plant.GetMutableJointByName<multibody::Joint>("floating");
		}
		catch (std::logic_error e) {}		//Leave at nullptr if joint doesn't exist

		try {
			pinJoint = &plant.GetMutableJointByName<multibody::Joint>("pin");
		}
		catch (std::logic_error e) {}

		elbowActuator = &plant.GetJointActuatorByName(elbowJoint->name());
		springActuator = &plant.GetJointActuatorByName(springJoint->name());

		if(floatingJoint) {
			is3d = (floatingJoint->num_positions() != 3);		//In 2d, planar joint has 3 pos
		}
		else {
			is3d = (pinJoint->num_positions() != 1);		//In 2d, revolute joint has 1 pos
		}
		posDim = (is3d ? 3 : 2);
		
		if(is3d) {
			//shoulderJoint = &plant.GetMutableJointByName<multibody::RevoluteJoint>("shoulder");
		}

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
	int posDim;
	multibody::RevoluteJoint<double>* shoulderJoint = nullptr;
	multibody::RevoluteJoint<double>* elbowJoint = nullptr;
	const multibody::JointActuator<double>* elbowActuator = nullptr;
	multibody::PrismaticJoint<double>* springJoint = nullptr;
	const multibody::JointActuator<double>* springActuator = nullptr;
	multibody::Joint<double>* floatingJoint = nullptr;
	multibody::Joint<double>* pinJoint = nullptr;

	int floatingPzStateIndex() const {
		return floatingJoint->position_start() + posDim - 1;		//Cannot be called on pinned system
	}
	int floatingVelStateIndex() const {
		return plant.num_positions() + floatingJoint->velocity_start();
	}
	int floatingVzStateIndex() const {
		return plant.num_positions() + floatingJoint->velocity_start() + posDim - 1;
	}
	int springVelStateIndex() const {
		return plant.num_positions() + springJoint->velocity_start();
	}
	int elbowVelStateIndex() const {
		return plant.num_positions() + elbowJoint->velocity_start();
	}
	int pinVelStateIndex() const {
		return plant.num_positions() + pinJoint->velocity_start();
	}

	int stateSize() const {
		return plant.num_multibody_states();
	}
	Eigen::VectorXd getDefaultState() const {
		return Eigen::VectorXd::Zero(stateSize());		//Todo: support quaternion in 3d case
	}

	const multibody::PrismaticSpring<double>* findSpring() const {
		const multibody::PrismaticSpring<double>* output = nullptr;
		for(int i = 0; i < plant.num_force_elements(); ++i) {
			const multibody::PrismaticSpring<double>* current = nullptr;
			try {
				current = &plant.GetForceElement<multibody::PrismaticSpring>(multibody::ForceElementIndex(i));
			}
			catch (std::exception e) {}

			if(current && &current->joint() == springJoint) {
				output = current;
				break;
			}
		}
		return output;
	}
};