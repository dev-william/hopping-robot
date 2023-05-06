#include "Optimize.h"

//#include "drake/planning/trajectory_optimization/direct_transcription.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/math/autodiff.h"

#include "StateHelper.h"
#include "RobotSystem.h"
#include "Constraints.h"
#include <iostream>
#include <chrono>

using namespace std;
using namespace drake;
using namespace planning::trajectory_optimization;

Eigen::VectorXd makeState(Eigen::VectorXd vec, double z, const StateHelper& help) {
	Eigen::VectorXd state = help.getDefaultState();
	state(Eigen::seqN(help.floatingJoint->position_start(), vec.size())) = replaceZ(vec, z);
	return state;
}

FootstepGuesser::FootstepGuesser(Eigen::VectorXd initialPosIn, Eigen::VectorXd finalPosIn, int numContacts) : initialPos(initialPosIn), finalPos(finalPosIn) {
	springPeriod = 2*M_PI / std::sqrt(springConstant / mass);		//Gravity probably changes this, at least for a half cycle
	ballisticTime = 2.0*fallTime(apexHeight - robotHeight, g);
	bounceTime = ballisticTime + springPeriod / 2.0;
	contactSpeed = std::abs(g * ballisticTime / 2.0);

	double initialZ = initialPos.tail(1)[0];
	double finalZ = finalPos.tail(1)[0];
	
	firstBallisticTime = fallTime(initialZ - robotHeight, g);
	lastBallisticTime = fallTime(finalZ - robotHeight, g);
	//additionalBounces = std::max(0, (int) std::lround((totalTime - firstBallisticTime - lastBallisticTime - springPeriod / 2.0) / bounceTime));

	Eigen::VectorXd deltaPosPerBounce = replaceZ(finalPos - initialPos, 0.0) / (double) (numContacts - 1);		//Unused if divided by zero
	for(int i = 0; i < numContacts; ++i) {
		if(i == 0) {
			contacts.push_back(replaceZ(initialPos, robotHeight));
			contactTimes.push_back(firstBallisticTime);
		}
		else {
			Eigen::VectorXd ballisticEnd = contacts[0] + (i+1) * deltaPosPerBounce;
			contacts.push_back(ballisticEnd);
			contactTimes.push_back(contactTimes.back() + ballisticTime + springPeriod / 2.0);
		}
	}
}

Traj FootstepGuesser::makeFullGuess(const StateHelper& help) {
	Traj output;
	auto baseSpringTraj = makeSpringTraj(contactSpeed, springPeriod / 2.0, robotHeight, 10, help);

	output.x.ConcatenateInTime(makeBallisticTraj(initialPos, contacts[0], contactTimes[0], 10, g, help));

	for(int i = 0; i < contacts.size(); ++i) {
		auto springTraj = (baseSpringTraj + makeState(contacts[i], 0.0, help));
		springTraj.shiftRight(output.x.end_time());
		output.x.ConcatenateInTime(springTraj);

		Eigen::VectorXd ballisticStart = contacts[i];
		bool isFinal = (i + 1) == contacts.size();
		Eigen::VectorXd ballisticEnd = isFinal ? finalPos : contacts[i+1];
		auto ballisticTraj = makeBallisticTraj(ballisticStart, ballisticEnd, isFinal ? lastBallisticTime : ballisticTime, 10, g, help);
		ballisticTraj.shiftRight(output.x.end_time());
		output.x.ConcatenateInTime(ballisticTraj);
	}

	/*if(enforceTotalTime) {
		double timeScaleFactor = totalTime / output.x.end_time();		//Assuming start_time is 0
		output.x.ScaleTime(timeScaleFactor);
		std::cout << "Scaled initial guess time by " << timeScaleFactor << "\n";
	}*/
	output.u = makeZeroInput(output.x, help);

	return output;
}

Traj FootstepGuesser::makeContactGuess(int contactIndex, const StateHelper& help) {
	Traj output;
	output.x = makeSpringTraj(contactSpeed, springPeriod / 2.0, robotHeight, 10, help);
	output.u = makeZeroInput(output.x, help);
	return output;
}

Traj FootstepGuesser::makeFlightGuess(int flightIndex, const StateHelper& help) {
	Traj output;
	bool isInitial = (flightIndex == 0);
	bool isFinal = (flightIndex == contacts.size());
	Eigen::VectorXd ballisticStart = isInitial ? initialPos : contacts[flightIndex - 1];
	Eigen::VectorXd ballisticEnd = isFinal ? finalPos : contacts[flightIndex];
	double currentTime = ballisticTime;
	if(isInitial)
		currentTime = firstBallisticTime;
	else if(isFinal)
		currentTime = lastBallisticTime;
	output.x = makeBallisticTraj(ballisticStart, ballisticEnd, currentTime, 10, g, help);
	output.u = makeZeroInput(output.x, help);
	return output;
}

trajectories::PiecewisePolynomial<double> FootstepGuesser::makeZeroInput(trajectories::PiecewisePolynomial<double> xTraj, const StateHelper& help) {
	Eigen::VectorXd uConstant = Eigen::VectorXd::Zero(help.plant.num_total_inputs());
	return trajectories::PiecewisePolynomial<double>::ZeroOrderHold({xTraj.start_time(), xTraj.end_time()}, {uConstant, uConstant});
}


drake::trajectories::PiecewisePolynomial<double> makeBallisticTraj(Eigen::VectorXd initialPos, Eigen::VectorXd finalPos, double trajTime, int sampleCount, double g, const StateHelper& help) {
	int posIndex = help.floatingJoint->position_start();
	int velIndex = help.floatingVelStateIndex();
	
	std::vector<double> times;
	for(int i = 0; i < sampleCount + 1; ++i) {
		times.push_back(i * trajTime / (double) sampleCount);
	}

	double initialZ = initialPos.tail(1)[0];
	double finalZ = finalPos.tail(1)[0];
	double initialVz = (0.5 * g * trajTime*trajTime - (finalZ - initialZ)) / -trajTime;
	int horzDims = initialPos.size() - 1;
	Eigen::VectorXd vHorz = (finalPos.head(horzDims) - initialPos.head(horzDims)) / trajTime;
	std::vector<Eigen::MatrixXd> samples, sampleDots;
	for(double sampleTime : times) {
		double pz = initialZ + sampleTime * initialVz + 0.5 * g * sampleTime*sampleTime;		//It occurred to me after writing this that I could just do one quadratic polynomial
		double vz = initialVz + g*sampleTime;
		Eigen::VectorXd sample = help.getDefaultState();
		Eigen::VectorXd sampleDot = Eigen::VectorXd::Zero(help.stateSize());
		for(int i = 0; i < initialPos.size(); ++i) {
			if(i == initialPos.size() - 1) {
				sample[posIndex + i] = pz;
				sample[velIndex + i] = vz;
				sampleDot[posIndex + i] = vz;
				sampleDot[velIndex + i] = g;
			}
			else {
				sample[posIndex + i] = initialPos[i] + vHorz[i] * sampleTime;
				sample[velIndex + i] = vHorz[i];
				sampleDot[posIndex + i] = vHorz[i];
				sampleDot[velIndex + i] = 0.0;
			}
		}
		samples.push_back(sample);
		sampleDots.push_back(sampleDot);
	}

	return drake::trajectories::PiecewisePolynomial<double>::CubicHermite(times, samples, sampleDots);
}

drake::trajectories::PiecewisePolynomial<double> makeSpringTraj(double initialSpeed, double trajTime, double robotHeight, int sampleCount, const StateHelper& help) {
	int index_pz_spring = help.springJoint->position_start();
	int index_vz_spring = help.springVelStateIndex();
	
	std::vector<double> times;
	for(int i = 0; i < sampleCount + 1; ++i) {
		times.push_back(i * trajTime / (double) sampleCount);
	}

	double stretchFactor = M_PI / trajTime;
	double amplitude = initialSpeed / stretchFactor;
	std::vector<Eigen::MatrixXd> samples, sampleDots;
	for(double sampleTime : times) {
		double pz = amplitude * -std::sin(stretchFactor * sampleTime);
		double vz = amplitude * stretchFactor * -std::cos(stretchFactor * sampleTime);
		double vzDot = amplitude * stretchFactor*stretchFactor * std::sin(stretchFactor * sampleTime);

		Eigen::VectorXd sample = help.getDefaultState();
		Eigen::VectorXd sampleDot = Eigen::VectorXd::Zero(help.stateSize());
		if(help.floatingJoint) {
			sample[help.floatingPzStateIndex()] = robotHeight + pz;
			sample[help.floatingVzStateIndex()] = vz;
			sampleDot[help.floatingPzStateIndex()] = vz;
			sampleDot[help.floatingVzStateIndex()] = vzDot;
		}

		sample[index_pz_spring] = -pz;
		sample[index_vz_spring] = vz;
		sampleDot[index_pz_spring] = vz;
		sampleDot[index_vz_spring] = vzDot;

		if(help.pinJoint) {
			sample[index_pz_spring] *= -1.0;
			sample[index_vz_spring] *= -1.0;
			sampleDot[index_pz_spring] *= -1.0;
			sampleDot[index_vz_spring] *= -1.0;
		}

		samples.push_back(sample);
		sampleDots.push_back(sampleDot);
	}

	return drake::trajectories::PiecewisePolynomial<double>::CubicHermite(times, samples, sampleDots);
}

trajectories::PiecewisePolynomial<double> optimizeOverScene() {
	double timestep = 0.0;        //If 0.0, then system is continuous
	bool use3d = false;

	RobotSystem sys(timestep, use3d, true, false);

	//sys.plant->set_contact_model(multibody::ContactModel::kPoint);
	//Setting penetration allowance made things worse
	//sys.plant->set_penetration_allowance(0.005);		//Default value .001
	sys.plantFinalize();

	systems::InputPortIndex exportedInputIndex = sys.exportInput();		//Give dircol access to plant input

	sys.finalize();


	double horizon = 0.36;
	int numTimeSamples = 80;
	double stepDuration = horizon / numTimeSamples;
	//DirectTranscription dirTran(&plant, plantContext, numTimeSamples, plant.get_actuation_input_port(modelIndex).get_index());	//Plant or diagram? sceneGraph needed for collisions
	DirectCollocation dirCol(sys.diagram.get(), *sys.diagramContext, numTimeSamples, stepDuration*0.9, stepDuration*1.1, exportedInputIndex, true);

	solvers::MathematicalProgram& mp = dirCol.prog();
	dirCol.AddEqualTimeIntervalsConstraints();

	//std::cout << "Opt state size " << dirCol.final_state().rows() << "\n";

	StateHelper help(*sys.plant);

	mp.AddConstraint(dirCol.initial_state()[help.elbowJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.initial_state()[help.springJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.initial_state()(Eigen::seqN(help.floatingJoint->position_start(), help.floatingJoint->num_positions())) == Vector3<double>(0.0, 0.45, 0.0));
	mp.AddConstraint(dirCol.initial_state()(Eigen::seqN(sys.plant->num_positions(), sys.plant->num_velocities())) == Eigen::VectorXd::Zero(sys.plant->num_velocities()));

	mp.AddConstraint(dirCol.final_state()[help.elbowJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.final_state()(Eigen::seqN(help.floatingJoint->position_start(), help.floatingJoint->num_positions())) == Vector3<double>(0.0, 0.45, 0.0));
	for(int i = 0; i < sys.plant->num_velocities(); ++i) {
		//if(i == help.floatingJoint->velocity_start() + 1)
			//continue;
		mp.AddConstraint(dirCol.final_state()[i + sys.plant->num_positions()] == 0.0);
		//mp.AddConstraint(dirCol.final_state()(Eigen::seqN(plant.num_positions(), plant.num_velocities())) == Eigen::VectorXd::Zero(plant.num_velocities()));
	}

	auto u = dirCol.input();
	dirCol.AddRunningCost(10.0 * u[0] * u[0]);
	dirCol.AddRunningCost(10.0 * u[1] * u[1]);

	double maxElbowTorque = 2.0;
	double maxSpringForce = 5.0;
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] <= maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] >= -maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] <= maxSpringForce);
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] >= -maxSpringForce);

	FootstepGuesser guesser(Eigen::VectorXd{{0.0, 0.45}}, Eigen::VectorXd{{0.0, 0.45}}, horizon);
	Traj initial = guesser.makeFullGuess(help);
	dirCol.SetInitialTrajectory(initial.u, initial.x);


	solvers::SnoptSolver solver;
	solvers::SolverOptions options = mp.solver_options();
	options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);		//Doesn't seem to do anything
	options.SetOption(solver.id(), "Iterations limit", 1000000);		//Increase iterations limit a lot
	//options.SetOption(solver.id(), "Major iterations limit", 500);
	options.SetOption(solver.id(), "Major optimality tolerance", 1e-4);
	mp.SetSolverOptions(options);

	auto start = chrono::steady_clock::now();
	solvers::MathematicalProgramResult res = solver.Solve(mp);
	auto end = chrono::steady_clock::now();
	std::cout << "Solve time: " << chrono::duration<double>(end - start).count() << " s\n\n";

	std::cout << "Success: " << res.is_success() << "\n";
	std::cout << "Solver: " << res.get_solver_id().name() << "\n";
	solvers::SnoptSolverDetails snoptDetails = res.get_solver_details<solvers::SnoptSolver>();
	std::cout << "SNOPT info code: " << snoptDetails.info << "\n";

	//if(res.is_success())
	return dirCol.ReconstructStateTrajectory(res);
}



HybridOptimization::HybridOptimization() {
	double timestep = 0.0;
	bool use3d = false;

	sysFloating = std::make_unique<RobotSystem>(timestep, use3d, true, false, false);
	sysFloating->plantFinalize();
	inputFloating = sysFloating->exportInput();
	sysFloating->finalize();

	sysPinned = std::make_unique<RobotSystem>(timestep, use3d, true, true, false);
	sysPinned->plantFinalize();
	inputPinned = sysPinned->exportInput();
	sysPinned->finalize();

	helpFloating = std::make_unique<StateHelper>(*sysFloating->plant);
	helpPinned = std::make_unique<StateHelper>(*sysPinned->plant);

	diagramFloatingAd = systems::System<double>::ToAutoDiffXd(*sysFloating->diagram);		//Converting to symbolic is also possible but probably not a good idea
	plantFloatingAd = &diagramFloatingAd->GetDowncastSubsystemByName<multibody::MultibodyPlant>(sysFloating->plant->get_name());
	diagramContextFloatingAd = diagramFloatingAd->CreateDefaultContext();
	plantContextFloatingAd = &diagramFloatingAd->GetMutableSubsystemContext(*plantFloatingAd, diagramContextFloatingAd.get());
	footFloatingAd = &plantFloatingAd->GetFrameByName("foot");

	diagramPinnedAd = systems::System<double>::ToAutoDiffXd(*sysPinned->diagram);
	plantPinnedAd = &diagramPinnedAd->GetDowncastSubsystemByName<multibody::MultibodyPlant>(sysPinned->plant->get_name());
	diagramContextPinnedAd = diagramPinnedAd->CreateDefaultContext();
	plantContextPinnedAd = &diagramPinnedAd->GetMutableSubsystemContext(*plantPinnedAd, diagramContextPinnedAd.get());
	basePinnedAd = &plantPinnedAd->GetFrameByName("base_link");
}

void HybridOptimization::createProblem(Eigen::VectorXd initialPos, Eigen::VectorXd finalPos, int numContacts, bool allowVariableTime) {
	FootstepGuesser guesser(initialPos, finalPos, numContacts);
	double timeVariation = allowVariableTime ? 0.1 : 0.0;

	for(int i = 0; i < guesser.contacts.size(); ++i) {
		Traj flightGuess = guesser.makeFlightGuess(i, *helpFloating);
		shared_ptr<DirectCollocation> dirColFloating = setupFloating(flightGuess, timeVariation, false);
		dirColsFloating.push_back(dirColFloating);

		if(i == 0) {
			constrainStateToPos(dirColFloating->initial_state(), initialPos);
		}
		else {
			linkAtContactEnd2d(*dirColsPinned.back(), *dirColFloating);
		}

		Traj pinnedGuess = guesser.makeContactGuess(i, *helpPinned);
		shared_ptr<DirectCollocation> dirColPinned = setupPinned(pinnedGuess, timeVariation);
		dirColsPinned.push_back(dirColPinned);
		Eigen::VectorXd pinPosGuess = flightGuess.x.value(flightGuess.x.end_time())(Eigen::seqN(helpFloating->floatingJoint->position_start(), 1), 0);
		createPinnedPosVars(dirColPinned.get(), pinPosGuess);

		linkAtContactStart2d(*dirColFloating, *dirColPinned);
	}

	Traj finalGuess = guesser.makeFlightGuess(guesser.contacts.size(), *helpFloating);
	shared_ptr<DirectCollocation> dirColFinal = setupFloating(finalGuess, timeVariation, true);
	dirColsFloating.push_back(dirColFinal);

	constrainStateToPos(dirColFinal->final_state(), finalPos);
	linkAtContactEnd2d(*dirColsPinned.back(), *dirColFinal);
}

void HybridOptimization::solve() {
	solvers::SnoptSolver solver;
	solvers::SolverOptions options = mp.solver_options();
	options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);		//Doesn't seem to do anything
	options.SetOption(solver.id(), "Iterations limit", 1000000);		//Increase iterations limit a lot
	//options.SetOption(solver.id(), "Major iterations limit", 500);
	options.SetOption(solver.id(), "Major optimality tolerance", 1e-4);
	mp.SetSolverOptions(options);

	auto start = chrono::steady_clock::now();
	res = solver.Solve(mp);
	auto end = chrono::steady_clock::now();
	std::cout << "Solve time: " << chrono::duration<double>(end - start).count() << " s\n\n";

	std::cout << "Success: " << res.is_success() << "\n";
	std::cout << "Solver: " << res.get_solver_id().name() << "\n";
	solvers::SnoptSolverDetails snoptDetails = res.get_solver_details<solvers::SnoptSolver>();
	std::cout << "SNOPT info code: " << snoptDetails.info << "\n";
	std::cout << "Optimal cost: " << res.get_optimal_cost() << "\n";
	if(!res.is_success()) {
		auto infeasibleConstraints = res.GetInfeasibleConstraints(mp);
		std::cout << "Num infeasible constraints: " << infeasibleConstraints.size() << "\n";
		for(auto& element : infeasibleConstraints) {
			std::cout << element << "\n";
		}
	}
}

Traj HybridOptimization::convertPinnedToFloating(const Traj& pinnedTraj, double px_wfoot) {
	Traj output;

	const std::vector<double>& breaks = pinnedTraj.x.get_segment_times();		//Includes start and end times
	std::vector<MatrixX<double>> updatedSamples;
	for(double t : breaks) {
		Eigen::VectorXd orig = pinnedTraj.x.value(t);
		Eigen::VectorXd updated = helpFloating->getDefaultState();

		updated[helpFloating->elbowJoint->position_start()] = -orig[helpPinned->elbowJoint->position_start()];
		updated[helpFloating->springJoint->position_start()] = -orig[helpPinned->springJoint->position_start()];
		updated[helpFloating->floatingJoint->position_start() + 2] = -orig[helpPinned->elbowJoint->position_start()] - orig[helpPinned->pinJoint->position_start()];

		sysPinned->plantContext->SetContinuousState(orig);
		math::RigidTransformd basePose = sysPinned->plant->EvalBodyPoseInWorld(*sysPinned->plantContext, sysPinned->plant->GetBodyByName("base_link"));
		updated[helpFloating->floatingJoint->position_start()] = basePose.translation()[0] + px_wfoot;
		updated[helpFloating->floatingJoint->position_start() + 1] = basePose.translation()[2];		//Todo: assumes 2d

		//Ignoring velocity for now, since this is mainly for visualization
		updatedSamples.push_back(updated);
	}
	output.x = trajectories::PiecewisePolynomial<double>::FirstOrderHold(breaks, updatedSamples);		//Not bothering with cubic for now

	//Negate inputs. There should definitely be an simpler way to do this
	const std::vector<double>& inputBreaks = pinnedTraj.u.get_segment_times();
	std::vector<MatrixX<double>> updatedInputs;
	for(double t : inputBreaks) {
		Eigen::VectorXd orig = pinnedTraj.u.value(t);
		Eigen::VectorXd updated = Eigen::VectorXd::Zero(helpFloating->plant.num_actuated_dofs());

		updated[helpFloating->elbowActuator->input_start()] = -orig[helpPinned->elbowActuator->input_start()];
		updated[helpFloating->springActuator->input_start()] = -orig[helpPinned->springActuator->input_start()];

		updatedInputs.push_back(updated);
	}
	output.u = trajectories::PiecewisePolynomial<double>::FirstOrderHold(inputBreaks, updatedInputs);

	return output;
}

Traj HybridOptimization::reconstructFullTraj() {
	Traj output;
	for(int i = 0; i < dirColsFloating.size(); ++i) {
		auto xTraj = dirColsFloating[i]->ReconstructStateTrajectory(res);
		if(i != 0)
			xTraj.shiftRight(output.x.end_time());
		output.x.ConcatenateInTime(xTraj);
		auto uTraj = dirColsFloating[i]->ReconstructInputTrajectory(res);
		if(i != 0)
			uTraj.shiftRight(output.u.end_time());
		output.u.ConcatenateInTime(uTraj);

		if(i < dirColsPinned.size()) {
			xTraj = dirColsPinned[i]->ReconstructStateTrajectory(res);
			xTraj.shiftRight(output.x.end_time());
			uTraj = dirColsPinned[i]->ReconstructInputTrajectory(res);
			uTraj.shiftRight(output.u.end_time());

			Traj pinned;
			pinned.x = xTraj;
			pinned.u = uTraj;
			Traj converted = convertPinnedToFloating(pinned, res.GetSolution(pinnedPosVars[dirColsPinned[i].get()][0]));
			output.x.ConcatenateInTime(converted.x);
			output.u.ConcatenateInTime(converted.u);
		}
	}
	return output;
}

std::shared_ptr<DirectCollocation> HybridOptimization::setupFloating(const Traj& guess, double timeVariationPercent, bool isFinal) {
	double horizon = guess.x.end_time();
	int numTimeSamples = 20;
	double stepDuration = horizon / numTimeSamples;
	double stepDurationMin = stepDuration * (1.0 - timeVariationPercent);
	double stepDurationMax = stepDuration * (1.0 + timeVariationPercent);

	shared_ptr<DirectCollocation> dirCol = std::make_shared<DirectCollocation>(sysFloating->diagram.get(), *sysFloating->diagramContext, numTimeSamples, stepDurationMin, stepDurationMax, inputFloating, true, &mp);

	dirCol->AddEqualTimeIntervalsConstraints();		//Is having all these constraints less efficient than just having one time decision variable?
	addInputConstraints(*dirCol, *helpFloating);
	addStateConstraints(*dirCol, *helpFloating);
	dirCol->SetInitialTrajectory(guess.u, guess.x);

	for(int i = 0; i < numTimeSamples; ++i) {
		if(i == 0)		//Already constrained when applicable
			continue;
		auto currentState = dirCol->state(i);
		symbolic::Variable baseZVar = currentState[helpFloating->floatingPzStateIndex()];
		symbolic::Variable planarAngleVar = currentState[helpFloating->floatingJoint->position_start() + 2];
		symbolic::Variable elbowVar = currentState[helpFloating->elbowJoint->position_start()];
		symbolic::Variable springVar = currentState[helpFloating->springJoint->position_start()];

		//Went ahead and wrote the analytical expression for the 2D case
		//Not as robust and generalizable as using the MultibodyPlant math, but I was concerned about performance and also it's way less code
		//Todo: unify methods and remove constants
		symbolic::Expression heightOffGround = baseZVar - 0.2*cos(planarAngleVar) - (0.2 - springVar) * cos(planarAngleVar - elbowVar);
		if(i == numTimeSamples - 1) {
			if(!isFinal) {
				mp.AddConstraint(heightOffGround == 0.0);
			}
		}
		else {
			mp.AddConstraint(heightOffGround >= 0.00001);		//Apparently just ">" is not allowed
		}
	}

	return dirCol;
}

std::shared_ptr<DirectCollocation> HybridOptimization::setupPinned(const Traj& guess, double timeVariationPercent) {
	double horizon = guess.x.end_time();
	int numTimeSamples = 20;
	double stepDuration = horizon / numTimeSamples;
	double stepDurationMin = stepDuration * (1.0 - timeVariationPercent);
	double stepDurationMax = stepDuration * (1.0 + timeVariationPercent);

	shared_ptr<DirectCollocation> dirCol = std::make_shared<DirectCollocation>(sysPinned->diagram.get(), *sysPinned->diagramContext, numTimeSamples, stepDurationMin, stepDurationMax, inputPinned, true, &mp);

	dirCol->AddEqualTimeIntervalsConstraints();
	addInputConstraints(*dirCol, *helpPinned);
	addStateConstraints(*dirCol, *helpPinned);
	dirCol->SetInitialTrajectory(guess.u, guess.x);

	//Normal force cannot be negative
	//f_N = (-kx + u) * cos(pin) + F_g,foot
	//Missing component from force perpendicular to spring axis?
	//Damping force ignored.
	double k = helpPinned->findSpring()->stiffness();
	double footMass = 0.3;		//Todo: get constants like this from MultibodyPlant
	for(int i = 0; i < numTimeSamples; ++i) {
		auto currentState = dirCol->state(i);
		auto currentInput = dirCol->input(i);
		const symbolic::Variable& springInputVar = currentInput[helpPinned->springActuator->input_start()];
		const symbolic::Variable& springPosVar = currentState[helpPinned->springJoint->position_start()];
		const symbolic::Variable& pinVar = currentState[helpPinned->pinJoint->position_start()];

		//Flat ground assumed
		symbolic::Expression normalForce = (-k*springPosVar + springInputVar) * cos(pinVar) + 9.81 * footMass;
		mp.AddConstraint(normalForce >= 0.0);
		mp.AddConstraint(pow((-k*springPosVar + springInputVar) * sin(pinVar), 2) <= pow(normalForce, 2));		//Friction limits. Coefficient of 1
		//Drake also has a friction constraint which may be worth investigating
		//Unsure of efficiency of pow(_, 2) - is 2 treated as an integer?
	}

	return dirCol;
}

//Explicitly track the x (and y if 3d) position of the contact point
//Not strictly necessary, since I could make a big constraint that tracks world position by tying together a pinned phase and the flight phases before and after it
//This way splits that constraint into two, which seems like it should be easier to think about
//The main advantage is that contact positions can be constrained directly
//Must be called before the link constraints are added
void HybridOptimization::createPinnedPosVars(DirectCollocation* dirCol, Eigen::VectorXd guess) {
	pinnedPosVars[dirCol] = mp.NewContinuousVariables(1, "horz pinned");
	mp.SetInitialGuess(pinnedPosVars[dirCol], guess);
}

void HybridOptimization::addInputConstraints(DirectCollocation& dirCol, const StateHelper& help) {
	auto u = dirCol.input();
	dirCol.AddRunningCost(10.0 * u[0] * u[0]);		//Todo: this is biased to add cost to the longer phases, since the timesteps are more spread out in time
	dirCol.AddRunningCost(10.0 * u[1] * u[1]);

	double maxElbowTorque = 5.0;
	double maxSpringForce = 80.0;		//Needs to be high to avoid infeasibility. Not physical, needs investigation
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] <= maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] >= -maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] <= maxSpringForce);
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] >= -maxSpringForce);
}

void HybridOptimization::addStateConstraints(DirectCollocation& dirCol, const StateHelper& help) {
	auto x = dirCol.state();

	double maxSpring = 0.1;		//SDF limits at 0.15. Needs further investigation - longer travel distance means more energy storage
	double maxElbow = M_PI * 3.0/4.0;
	dirCol.AddConstraintToAllKnotPoints(x[help.springJoint->position_start()] <= maxSpring);
	dirCol.AddConstraintToAllKnotPoints(x[help.springJoint->position_start()] >= -maxSpring);
	dirCol.AddConstraintToAllKnotPoints(x[help.elbowJoint->position_start()] <= maxElbow);
	dirCol.AddConstraintToAllKnotPoints(x[help.elbowJoint->position_start()] >= -maxElbow);

	if(help.pinJoint) {
		double maxPin = M_PI / 2.0;
		dirCol.AddConstraintToAllKnotPoints(x[help.pinJoint->position_start()] <= maxPin);
		dirCol.AddConstraintToAllKnotPoints(x[help.pinJoint->position_start()] >= -maxPin);
	}
}

void HybridOptimization::constrainStateToPos(drake::solvers::VectorXDecisionVariable state, Eigen::VectorXd pos) {
	mp.AddConstraint(state[helpFloating->elbowJoint->position_start()] == 0.0);
	mp.AddConstraint(state[helpFloating->springJoint->position_start()] == 0.0);
	mp.AddConstraint(state(Eigen::seqN(helpFloating->floatingJoint->position_start(), helpFloating->floatingJoint->num_positions())) == Vector3<double>(pos[0], pos.tail(1)[0], 0.0));
	mp.AddConstraint(state(Eigen::seqN(helpFloating->plant.num_positions(), helpFloating->plant.num_velocities())) == Eigen::VectorXd::Zero(helpFloating->plant.num_velocities()));
}

void HybridOptimization::linkAtContactStart2d(DirectCollocation& dirColFloating, DirectCollocation& dirColPinned) {
	//A positive spring joint state retracts the spring when floating and extends it when pinned, so it must be negated when converting between robot configurations
	//Same thing for elbow joint - when floating, it rotates about the +y, and when pinned, it rotates about the -y
	mp.AddConstraint(dirColFloating.final_state()[helpFloating->elbowJoint->position_start()] == -dirColPinned.initial_state()[helpPinned->elbowJoint->position_start()]);
	mp.AddConstraint(dirColFloating.final_state()[helpFloating->springJoint->position_start()] == -dirColPinned.initial_state()[helpPinned->springJoint->position_start()]);

	//Planar floating joint rotates around -y axis. Elbow rotates around +y axis (when floating). Pin rotates around +y axis.
	//Could use CalcRelativeRotationMatrix, which would generalize to 3d, but it is more complex and its autodiff gradients are possibly bugged
	symbolic::Expression touchdownAngle = -dirColFloating.final_state()[helpFloating->floatingJoint->position_start() + 2] + dirColFloating.final_state()[helpFloating->elbowJoint->position_start()];
	mp.AddConstraint(touchdownAngle == dirColPinned.initial_state()[helpPinned->pinJoint->position_start()]);

	//Set the one decision variable tracking the x world pos of the foot during the contact phase
	//Adding this seems to have a nontrivial perf impact. I could manually write an expression, at least for the 2d case - would be good for verification and perf testing
	mp.AddConstraint(make_shared<FramePosConstraint>(plantFloatingAd, plantContextFloatingAd, footFloatingAd, 0, false), {dirColFloating.final_state().head(sysFloating->plant->num_positions()), pinnedPosVars[&dirColPinned]});


	//Velocities
	//In 1d case, all velocity of the base link gets transferred to the prismatic joint
	mp.AddConstraint(make_shared<ImpactConstraint>(plantFloatingAd, plantContextFloatingAd, *helpFloating, *helpPinned), {dirColFloating.final_state(), dirColPinned.initial_state()(Eigen::seqN(helpPinned->plant.num_positions(), helpPinned->plant.num_velocities()))});

	//mp.AddConstraint(dirColFloating.final_state()[helpFloating->elbowVelStateIndex()] == -dirColPinned.initial_state()[helpPinned->elbowVelStateIndex()]);
	//mp.AddConstraint(dirColFloating.final_state()[helpFloating->springVelStateIndex()] == -dirColPinned.initial_state()[helpPinned->springVelStateIndex()]);

	//symbolic::Expression touchdownAngVel = -dirColFloating.final_state()[helpFloating->floatingVelStateIndex() + 2] + dirColFloating.final_state()[helpFloating->elbowVelStateIndex()];
	//mp.AddConstraint(touchdownAngVel == dirColPinned.initial_state()[helpPinned->pinVelStateIndex()]);
}

void HybridOptimization::linkAtContactEnd2d(DirectCollocation& dirColPinned, DirectCollocation& dirColFloating) {
	mp.AddConstraint(dirColPinned.final_state()[helpPinned->elbowJoint->position_start()] == -dirColFloating.initial_state()[helpFloating->elbowJoint->position_start()]);
	mp.AddConstraint(dirColPinned.final_state()[helpPinned->springJoint->position_start()] == -dirColFloating.initial_state()[helpFloating->springJoint->position_start()]);

	//Set world x position from pinned robot state and x pos of contact point
	mp.AddConstraint(make_shared<FramePosConstraint>(plantPinnedAd, plantContextPinnedAd, basePinnedAd, 0, true), {dirColPinned.final_state().head(sysPinned->plant->num_positions()), pinnedPosVars[&dirColPinned], dirColFloating.initial_state()(Eigen::seqN(helpFloating->floatingJoint->position_start(), 1))});

	symbolic::Expression planarAngle = -dirColPinned.final_state()[helpPinned->elbowJoint->position_start()] - dirColPinned.final_state()[helpPinned->pinJoint->position_start()];
	mp.AddConstraint(planarAngle == dirColFloating.initial_state()[helpFloating->floatingJoint->position_start() + 2]);

	//World z position
	mp.AddConstraint(make_shared<FramePosConstraint>(plantPinnedAd, plantContextPinnedAd, basePinnedAd, 2, false), {dirColPinned.final_state().head(sysPinned->plant->num_positions()), dirColFloating.initial_state()(Eigen::seqN(helpFloating->floatingPzStateIndex(), 1))});
	//mp.AddConstraint(dirColPinned.final_state()[helpPinned->springJoint->position_start()] + 0.4 == dirColFloating.initial_state()[helpFloating->floatingPzStateIndex()]);


	//Velocities
	mp.AddConstraint(dirColPinned.final_state()[helpPinned->elbowVelStateIndex()] == -dirColFloating.initial_state()[helpFloating->elbowVelStateIndex()]);
	mp.AddConstraint(dirColPinned.final_state()[helpPinned->springVelStateIndex()] == -dirColFloating.initial_state()[helpFloating->springVelStateIndex()]);

	//World x and z velocities
	mp.AddConstraint(make_shared<TakeoffBaseVelConstraint>(plantPinnedAd, plantContextPinnedAd), {dirColPinned.final_state(), dirColFloating.initial_state()(Eigen::seqN(helpFloating->floatingVelStateIndex(), 2))});
	//mp.AddConstraint(dirColPinned.final_state()[helpPinned->springVelStateIndex()] == dirColFloating.initial_state()[helpFloating->floatingVzStateIndex()]);
	//mp.AddConstraint(dirColFloating.initial_state()[helpFloating->floatingVelStateIndex()] == 0.0);

	symbolic::Expression planarAngVel = -dirColPinned.final_state()[helpPinned->elbowVelStateIndex()] - dirColPinned.final_state()[helpPinned->pinVelStateIndex()];
	mp.AddConstraint(planarAngVel == dirColFloating.initial_state()[helpFloating->floatingVelStateIndex() + 2]);
}