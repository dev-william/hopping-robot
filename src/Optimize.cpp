#include "Optimize.h"

#include "StateHelper.h"
#include <iostream>

Eigen::VectorXd makeState(Eigen::VectorXd vec, double z, const StateHelper& help) {
	Eigen::VectorXd state = help.getDefaultState();
	state(Eigen::seqN(help.floatingJoint->position_start(), vec.size())) = replaceZ(vec, z);
	return state;
}

FootstepGuesser::FootstepGuesser(Eigen::VectorXd initialPosIn, Eigen::VectorXd finalPosIn, double totalTimeIn) : initialPos(initialPosIn), finalPos(finalPosIn), totalTime(totalTimeIn) {
	springPeriod = 2*M_PI / std::sqrt(springConstant / mass);		//Gravity probably changes this, at least for a half cycle
	ballisticTime = 2.0*fallTime(apexHeight - robotHeight, g);
	bounceTime = ballisticTime + springPeriod / 2.0;
	contactSpeed = std::abs(g * ballisticTime / 2.0);

	double initialZ = initialPos.tail(1)[0];
	double finalZ = finalPos.tail(1)[0];
	
	firstBallisticTime = fallTime(initialZ - robotHeight, g);
	lastBallisticTime = fallTime(finalZ - robotHeight, g);
	additionalBounces = std::max(0, (int) std::lround((totalTime - firstBallisticTime - lastBallisticTime - springPeriod / 2.0) / bounceTime));

	contacts.push_back(replaceZ(initialPos, robotHeight));
	contactTimes.push_back(firstBallisticTime);

	Eigen::VectorXd deltaPosPerBounce = replaceZ(finalPos - initialPos, 0.0) / (double) additionalBounces;		//Unused if divided by zero
	for(int i = 0; i < additionalBounces; ++i) {
		Eigen::VectorXd ballisticEnd = contacts[0] + (i+1) * deltaPosPerBounce;
		contacts.push_back(ballisticEnd);
		contactTimes.push_back(contactTimes.back() + ballisticTime + springPeriod / 2.0);
	}
}

Traj FootstepGuesser::makeFullGuess(bool enforceTotalTime, const StateHelper& help) {
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

	if(enforceTotalTime) {
		double timeScaleFactor = totalTime / output.x.end_time();		//Assuming start_time is 0
		output.x.ScaleTime(timeScaleFactor);
		std::cout << "Scaled initial guess time by " << timeScaleFactor << "\n";
	}
	Eigen::VectorXd uConstant = Eigen::VectorXd::Zero(help.plant.num_total_inputs());
	output.u = trajectories::PiecewisePolynomial<double>::ZeroOrderHold({output.x.start_time(), output.x.end_time()}, {uConstant, uConstant});

	return output;
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
	int index_pz_base = help.floatingPzStateIndex();
	int index_vz_base = help.floatingVzStateIndex();
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