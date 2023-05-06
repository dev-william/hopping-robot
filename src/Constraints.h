#pragma once

#include "drake/solvers/constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <iostream>

//Drake provides a super easy generic constraint interface for Python, where you just pass a function,
//but apparently they don't want to do that for C++, so you have to go make custom constraint classes.
//There is technically a FunctionEvaluator, which could go into a EvaluatorConstraint,
//but it's slightly cursed and you still have to define a class anyways.

//PositionConstraint is a good example of a multibody constraint
//Only providing the autodiff definitions since that's all SNOPT uses. Double hacked on so that the constraint infeasibility check works when SNOPT fails.
//Todo: test autodiff gradients with finite difference

typedef Eigen::Matrix<AutoDiffd<Eigen::Dynamic>, Eigen::Dynamic, Eigen::Dynamic> AutoDiffMatXd;

//Constrain a component of the world position of the origin of a frame on a MultibodyPlant, plus an optional var, to equal some other decision variable
//Effective expression: frame_world_pos[dim] + left_var = right_var
//Decision variables must be the state position vector first and then the other decision variables in the order they appear in the above expression
//Select component xyz with dim (0-2)
//Todo: consider replacing autodiff with Jacobian gradient. Do perf testing. PositionConstraint has ex of this exact gradient computation
//Also for perf, number of calculations is triple what is needed - two dimensions are discarded
class FramePosConstraint : public drake::solvers::Constraint {
public:
	FramePosConstraint(const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plantIn,
		drake::systems::Context<drake::AutoDiffXd>* plantContextIn,
		const drake::multibody::Frame<drake::AutoDiffXd>* frameIn, int dimIn, bool extraLhsIn)
		: Constraint(1, plantIn->num_positions() + 1 + ((int) extraLhsIn), drake::Vector1d{0.0}, drake::Vector1d{0.0}),
		plant(plantIn), plantContext(plantContextIn), frame(frameIn), dim(dimIn), extraLhs(extraLhsIn) {}

protected:
	void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x, drake::AutoDiffVecXd* y) const override {
		plant->SetPositions(plantContext, x.head(plant->num_positions()));
		drake::AutoDiffVecXd origin = drake::AutoDiffVecXd::Zero(3);		//Has nothing in the derivatives vector
		drake::AutoDiffVecXd worldPos(3);
		plant->CalcPointsPositions(*plantContext, *frame, origin, plant->world_frame(), &worldPos);
		*y = worldPos(Eigen::seqN(dim, 1)) - x.tail(1);
		if(extraLhs)
			*y += x(Eigen::seqN(plant->num_positions(), 1));
	}

	void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const override {
		AutoDiffVecXd yAd = AutoDiffVecXd::Zero(y->size());
		AutoDiffVecXd xAd(x);
		DoEval(xAd, &yAd);
		*y = math::ExtractValue(yAd);
	}

	void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>&, drake::VectorX<drake::symbolic::Expression>*) const override {
		throw std::logic_error("FramePosConstraint does not support Eval with symbolic, only autodiff");
	}

	const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plant;
	drake::systems::Context<drake::AutoDiffXd>* plantContext;
	const drake::multibody::Frame<drake::AutoDiffXd>* frame;
	int dim;
	bool extraLhs;
};

//Sets the world velocity (x and z) of the base of the floating robot based on the full state of the pinned robot
//Decision vars must be the full pinned state first and then x and z base velocities
class TakeoffBaseVelConstraint : public drake::solvers::Constraint {
public:
	TakeoffBaseVelConstraint(const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plantIn, drake::systems::Context<drake::AutoDiffXd>* plantContextIn)
			: Constraint(2, plantIn->num_multibody_states() + 2, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()), plant(plantIn), plantContext(plantContextIn) {
		baseFrame = &plant->GetFrameByName("base_link");
	}

protected:
	void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x, drake::AutoDiffVecXd* y) const override {
		plant->SetPositions(plantContext, x.head(plant->num_positions()));		//Jacobian is only a function of positions

		//This big autodiff matrix is awkward and probably slow, but Drake doesn't seem to have a method to get translational velocity directly
		//There is EvalBodySpatialVelocityInWorld(), but not 100% sure on how to get the correct translational world velocity from that
		AutoDiffMatXd J(3, plant->num_velocities());
		plant->CalcJacobianTranslationalVelocity(*plantContext, drake::multibody::JacobianWrtVariable::kV, *baseFrame, drake::AutoDiffVecXd::Zero(3), plant->world_frame(), plant->world_frame(), &J);
		drake::AutoDiffVecXd worldVel = J * x(Eigen::seqN(plant->num_positions(), plant->num_velocities()));
		(*y)[0] = worldVel[0] - x(Eigen::last - 1);
		(*y)[1] = worldVel[2] - x(Eigen::last);
	}

	void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const override {
		AutoDiffVecXd yAd = AutoDiffVecXd::Zero(y->size());
		AutoDiffVecXd xAd(x);
		DoEval(xAd, &yAd);
		*y = math::ExtractValue(yAd);
	}

	void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>&, drake::VectorX<drake::symbolic::Expression>*) const override {
		throw std::logic_error("TakeoffBaseVelConstraint does not support Eval with symbolic, only autodiff");
	}

	const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plant;
	drake::systems::Context<drake::AutoDiffXd>* plantContext;
	const drake::multibody::Frame<drake::AutoDiffXd>* baseFrame;
};

//Maps floating robot velocities to pinned robot velocities, taking into account impact impulse
//Decision vars: full floating robot state, velocities of pinned robot
//Source: Mathematical Modeling of a Robot Collision with its Environment, Zheng 1985, eqn. 14
class ImpactConstraint : public drake::solvers::Constraint {
public:
	ImpactConstraint(const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plantIn, drake::systems::Context<drake::AutoDiffXd>* plantContextIn,
			const StateHelper& helpFloatingIn, const StateHelper& helpPinnedIn)
			: Constraint(3, plantIn->num_multibody_states() + 3, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
			plant(plantIn), plantContext(plantContextIn), helpFloating(helpFloatingIn), helpPinned(helpPinnedIn) {
		footFrame = &plant->GetFrameByName("foot");
	}

protected:
	void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x, drake::AutoDiffVecXd* y) const override {
		plant->SetPositions(plantContext, x.head(plant->num_positions()));

		AutoDiffMatXd J3d(3, plant->num_positions());
		plant->CalcJacobianPositionVector(*plantContext, *footFrame, drake::AutoDiffVecXd::Zero(3), plant->world_frame(), plant->world_frame(), &J3d);
		AutoDiffMatXd J(2, 5);
		Eigen::Vector2i xzSelect{0, 2};
		J = J3d(xzSelect, Eigen::all);		//Leave out y

		AutoDiffVecXd qdot(plant->num_positions());			//Same as velocity in 2d case, but keeping to make extension to 3d easier
		plant->MapVelocityToQDot(*plantContext, x(Eigen::seqN(plant->num_positions(), plant->num_velocities())), &qdot);
		AutoDiffVecXd footVel = J * qdot;		//According to the docs, this should be equivalent to using CalcJacobianTranslationalVelocity()

		AutoDiffMatXd M(5, 5);
		plant->CalcMassMatrix(*plantContext, &M);
		AutoDiffMatXd Minv = M.inverse();

		drake::AutoDiffVecXd deltaQdot = Minv * J.transpose() * (J * Minv * J.transpose()).inverse() * -footVel;

		//With this qdot, the floating robot's foot has zero velocity, and so its velocities can be directly mapped to the pinned robot
		drake::AutoDiffVecXd updatedQdot = qdot + deltaQdot;
		drake::AutoDiffVecXd updatedVel(plant->num_velocities());
		plant->MapQDotToVelocity(*plantContext, updatedQdot, &updatedVel);

		const AutoDiffVecXd& pinnedVel = x.tail(3);
		(*y)[0] = updatedVel[helpFloating.springJoint->velocity_start()] + pinnedVel[helpPinned.springJoint->velocity_start()];		//Add instead of subtract because they are negatives of each other
		(*y)[1] = updatedVel[helpFloating.elbowJoint->velocity_start()] + pinnedVel[helpPinned.elbowJoint->velocity_start()];
		(*y)[2] = updatedVel[helpFloating.elbowJoint->velocity_start()] - updatedVel[helpFloating.floatingJoint->velocity_start() + 2] - pinnedVel[helpPinned.pinJoint->velocity_start()];

		/*std::cout << "Floating pos: " << x.head(plant->num_positions()) << "\n";
		std::cout << "Floating initial vel: " << x(Eigen::seqN(plant->num_positions(), plant->num_velocities())) << "\n";
		std::cout << "Floating initial qdot: " << qdot << "\n";
		std::cout << "Initial foot vel: " << footVel << "\n";
		std::cout << "Updated vel: " << updatedVel << "\n";

		//Confirm foot vel is 0
		AutoDiffMatXd Jv(3, plant->num_velocities());
		plant->CalcJacobianTranslationalVelocity(*plantContext, drake::multibody::JacobianWrtVariable::kV, *footFrame, drake::AutoDiffVecXd::Zero(3), plant->world_frame(), plant->world_frame(), &Jv);
		drake::AutoDiffVecXd worldVel = Jv * updatedVel;
		std::cout << "Final foot vel: " << worldVel << "\n\n";*/
	}

	void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const override {
		AutoDiffVecXd yAd = AutoDiffVecXd::Zero(y->size());
		AutoDiffVecXd xAd(x);
		DoEval(xAd, &yAd);
		*y = math::ExtractValue(yAd);
	}

	void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>&, drake::VectorX<drake::symbolic::Expression>*) const override {
		throw std::logic_error("TakeoffBaseVelConstraint does not support Eval with symbolic, only autodiff");
	}
	const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plant;
	drake::systems::Context<drake::AutoDiffXd>* plantContext;
	const drake::multibody::Frame<drake::AutoDiffXd>* footFrame;
	const StateHelper& helpFloating;
	const StateHelper& helpPinned;
};