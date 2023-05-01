#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
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
	FootstepGuesser(Eigen::VectorXd initialPosIn, Eigen::VectorXd finalPosIn, double totalTimeIn);
	Traj makeFullGuess(bool enforceTotalTime, const StateHelper& help);		//enforceTotalTime rescales time to match the original time from the constructor

	Eigen::VectorXd initialPos;
	Eigen::VectorXd finalPos;
	double totalTime;

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
	int additionalBounces;

	std::vector<Eigen::VectorXd> contacts;
	std::vector<double> contactTimes;		//Time at beginning of corresponding contact phase
};
