#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/solve.h"
#include "drake/geometry/utilities.h"
#include "drake/systems/framework/system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "Utils.h"

class StateHelper;

inline double fallTime(double height, double gravity) {
	return std::sqrt(-height * 2.0 / gravity);
}

inline Eigen::VectorXd replaceZ(Eigen::VectorXd vec, double z) {
	vec.tail(1)[0] = z;
	return vec;
}

Eigen::VectorXd makeState(Eigen::VectorXd vec, double z, const StateHelper& help);

drake::trajectories::PiecewisePolynomial<double> makeBallisticTraj(Eigen::VectorXd initialPos, Eigen::VectorXd finalPos, double trajTime, int sampleCount, double g, const StateHelper& help);

//Makes a sine wave "bowl" to represent harmonic motion
drake::trajectories::PiecewisePolynomial<double> makeSpringTraj(double initialSpeed, double trajTime, double robotHeight, int sampleCount, const StateHelper& help);

class FootstepGuesser {
public:
	FootstepGuesser(Eigen::VectorXd initialPosIn, Eigen::VectorXd finalPosIn, int numContacts);
	Traj makeFullGuess(const StateHelper& help);		//enforceTotalTime rescales time to match the original time from the constructor
	Traj makeContactGuess(int contactIndex, const StateHelper& help);
	Traj makeFlightGuess(int flightIndex, const StateHelper& help);		//flightIndex can be one more than contactIndex

	Eigen::VectorXd initialPos;
	Eigen::VectorXd finalPos;
	//double totalTime;

	static constexpr double apexHeight = 0.45;
	static constexpr double robotHeight = 0.4;		//Floor to COM of head
	static constexpr double springConstant = 1000.0;
	static constexpr double mass = 2.5;
	static constexpr double g = -9.81;

	double springPeriod;
	double ballisticTime;
	double bounceTime;
	double contactSpeed;

	double firstBallisticTime;
	double lastBallisticTime;
	//int additionalBounces;

	std::vector<Eigen::VectorXd> contacts;
	std::vector<double> contactTimes;		//Time at beginning of corresponding contact phase

	private:
	drake::trajectories::PiecewisePolynomial<double> makeZeroInput(drake::trajectories::PiecewisePolynomial<double> xTraj, const StateHelper& help);
};

namespace drake::planning::trajectory_optimization {
	class DirectCollocation;
}

//This method doesn't work
drake::trajectories::PiecewisePolynomial<double> optimizeOverScene();

class HybridOptimization {
public:
	HybridOptimization();
	void createProblem(Eigen::VectorXd initialPosIn, Eigen::VectorXd finalPosIn, int numContacts, bool allowVariableTime);
	void solve();
	Traj convertPinnedToFloating(const Traj& pinnedTraj, double px_wfoot);		//Assumes 2d
	Traj reconstructFullTraj();		//Get the full solution from all the subproblems in floating coordinates

private:
	std::unique_ptr<RobotSystem> sysFloating, sysPinned;
	std::unique_ptr<StateHelper> helpFloating, helpPinned;
	drake::systems::InputPortIndex inputFloating, inputPinned;
	drake::solvers::MathematicalProgram mp;
	std::vector<std::shared_ptr<drake::planning::trajectory_optimization::DirectCollocation>> dirColsFloating;
	std::vector<std::shared_ptr<drake::planning::trajectory_optimization::DirectCollocation>> dirColsPinned;
	std::map<drake::planning::trajectory_optimization::DirectCollocation*, drake::solvers::VectorXDecisionVariable> pinnedPosVars;
	drake::solvers::MathematicalProgramResult res;

	//Autodiff components
	std::unique_ptr<drake::systems::Diagram<drake::AutoDiffXd>> diagramFloatingAd, diagramPinnedAd;
	const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plantFloatingAd;
	const drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plantPinnedAd;
	std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>> diagramContextFloatingAd, diagramContextPinnedAd;
	drake::systems::Context<drake::AutoDiffXd>* plantContextFloatingAd;
	drake::systems::Context<drake::AutoDiffXd>* plantContextPinnedAd;
	const drake::multibody::Frame<drake::AutoDiffXd>* footFloatingAd;
	const drake::multibody::Frame<drake::AutoDiffXd>* basePinnedAd;

	std::shared_ptr<drake::planning::trajectory_optimization::DirectCollocation> setupFloating(const Traj& guess, double timeVariationPercent, bool isFinal);
	std::shared_ptr<drake::planning::trajectory_optimization::DirectCollocation> setupPinned(const Traj& guess, double timeVariationPercent);
	void createPinnedPosVars(drake::planning::trajectory_optimization::DirectCollocation* dirCol, Eigen::VectorXd guess);
	static void addInputConstraints(drake::planning::trajectory_optimization::DirectCollocation& dirCol, const StateHelper& help);
	static void addStateConstraints(drake::planning::trajectory_optimization::DirectCollocation& dirCol, const StateHelper& help);
	void constrainStateToPos(drake::solvers::VectorXDecisionVariable state, Eigen::VectorXd pos);		//Assumes 2d and floating. Zeroes all velocities
	void linkAtContactStart2d(drake::planning::trajectory_optimization::DirectCollocation& dirColFloating, drake::planning::trajectory_optimization::DirectCollocation& dirColPinned);
	void linkAtContactEnd2d(drake::planning::trajectory_optimization::DirectCollocation& dirColFloating, drake::planning::trajectory_optimization::DirectCollocation& dirColPinned);
};