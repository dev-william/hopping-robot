#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "StateHelper.h"
#include "Utils.h"
#include <iostream>

double fallTime(double height, double gravity) {
	return std::sqrt(-height * 2.0 / gravity);
}

drake::trajectories::PiecewisePolynomial<double> makeBallisticTraj(Eigen::VectorXd initialPos, Eigen::VectorXd finalPos, double trajTime, int stateCount, int posIndex, int velIndex, int sampleCount, double g) {
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
		Eigen::VectorXd sample = Eigen::VectorXd::Zero(stateCount);
		Eigen::VectorXd sampleDot = Eigen::VectorXd::Zero(stateCount);
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

//Makes a sine wave "bowl" to represent harmonic motion
drake::trajectories::PiecewisePolynomial<double> makeSpringTraj(double initialSpeed, double trajTime, double robotHeight, int stateCount, int index_pz_base, int index_vz_base, int index_pz_spring, int index_vz_spring, int sampleCount) {
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

		Eigen::VectorXd sample = Eigen::VectorXd::Zero(stateCount);
		Eigen::VectorXd sampleDot = Eigen::VectorXd::Zero(stateCount);
		sample[index_pz_base] = robotHeight + pz;
		sample[index_vz_base] = vz;
		sampleDot[index_pz_base] = vz;
		sampleDot[index_vz_base] = vzDot;

		sample[index_pz_spring] = -pz;
		sample[index_vz_spring] = vz;
		sampleDot[index_pz_spring] = vz;
		sampleDot[index_vz_spring] = vzDot;

		samples.push_back(sample);
		sampleDots.push_back(sampleDot);
	}

	return drake::trajectories::PiecewisePolynomial<double>::CubicHermite(times, samples, sampleDots);
}

Eigen::VectorXd replaceZ(Eigen::VectorXd vec, double z) {
	vec.tail(1)[0] = z;
	return vec;
}

Eigen::VectorXd makeState(Eigen::VectorXd vec, double z, StateHelper& help) {
	Eigen::VectorXd state = Eigen::VectorXd::Zero(help.plant.num_multibody_states());
	state(Eigen::seqN(help.floatingJoint->position_start(), vec.size())) = replaceZ(vec, z);
	return state;
}

//initialPos and finalPos should be 2d (xz) or 3d (xyz)
//Velocity and all joint states assumed to be zero at beginning and end
//Assumes flat ground plane
Traj makeInitialGuess(StateHelper& help, Eigen::VectorXd initialPos, Eigen::VectorXd finalPos, double totalTime) {
	double apexHeight = 0.45;
	double robotHeight = 0.4;		//Floor to COM of head
	double springConstant = 1000.0;
	double mass = 2.5;
	double g = -9.81;

	double springPeriod = 2*M_PI / std::sqrt(springConstant / mass);		//Gravity probably changes this, at least for a half cycle
	double ballisticTime = 2.0*fallTime(apexHeight - robotHeight, g);
	double bounceTime = ballisticTime + springPeriod / 2.0;
	double contactSpeed = std::abs(g * ballisticTime / 2.0);

	double initialZ = initialPos.tail(1)[0];
	double finalZ = finalPos.tail(1)[0];
	
	double firstBallisticTime = fallTime(initialZ - robotHeight, g);
	double lastBallisticTime = fallTime(finalZ - robotHeight, g);
	int additionalBounces = std::max(0, (int) std::lround((totalTime - firstBallisticTime - lastBallisticTime - springPeriod / 2.0) / bounceTime));
	Eigen::VectorXd deltaPosPerBounce = replaceZ(finalPos - initialPos, 0.0) / (double) additionalBounces;		//Unused if divided by zero


	Traj output;
	int stateCount = help.plant.num_multibody_states();
	auto baseSpringTraj = makeSpringTraj(contactSpeed, springPeriod / 2.0, robotHeight, stateCount, help.floatingJoint->position_start() + initialPos.size() - 1, help.floatingVzStateIndex(), help.springJoint->position_start(), help.springVelStateIndex(), 10);

	output.x.ConcatenateInTime(makeBallisticTraj(initialPos, replaceZ(initialPos, robotHeight), firstBallisticTime, stateCount, help.floatingJoint->position_start(), help.floatingVelStateIndex(), 10, g));
	
	auto firstSpringTraj = (baseSpringTraj + makeState(initialPos, 0.0, help));
	firstSpringTraj.shiftRight(output.x.end_time());
	output.x.ConcatenateInTime(firstSpringTraj);

	for(int i = 0; i < additionalBounces; ++i) {
		Eigen::VectorXd ballisticStart = replaceZ(initialPos, robotHeight) + i * deltaPosPerBounce;
		auto ballisticEnd = ballisticStart + deltaPosPerBounce;
		auto ballisticTraj = makeBallisticTraj(ballisticStart, ballisticEnd, ballisticTime, stateCount, help.floatingJoint->position_start(), help.floatingVelStateIndex(), 10, g);
		ballisticTraj.shiftRight(output.x.end_time());
		output.x.ConcatenateInTime(ballisticTraj);

		auto springTraj = (baseSpringTraj + makeState(ballisticEnd, 0.0, help));
		springTraj.shiftRight(output.x.end_time());
		output.x.ConcatenateInTime(springTraj);
	}

	auto finalBallisticTraj = makeBallisticTraj(replaceZ(finalPos, robotHeight), finalPos, lastBallisticTime, stateCount, help.floatingJoint->position_start(), help.floatingVelStateIndex(), 10, g);
	finalBallisticTraj.shiftRight(output.x.end_time());
	output.x.ConcatenateInTime(finalBallisticTraj);

	double timeScaleFactor = totalTime / output.x.end_time();		//Assuming start_time is 0
	output.x.ScaleTime(timeScaleFactor);
	std::cout << "Scaled initial guess time by " << timeScaleFactor << "\n";
	Eigen::VectorXd uConstant = Eigen::VectorXd::Zero(help.plant.num_total_inputs());
	output.u = trajectories::PiecewisePolynomial<double>::ZeroOrderHold({output.x.start_time(), output.x.end_time()}, {uConstant, uConstant});

	return output;
}