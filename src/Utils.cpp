#include "Utils.h"
#include "RobotSystem.h"
#include "StateHelper.h"
#include <iostream>
#include <chrono>
#include <thread>

//From https://github.com/lava/matplotlib-cpp
#include "matplotlibcpp.h"

using namespace drake;
using namespace std;
namespace plt = matplotlibcpp;

void playTrajectory(trajectories::PiecewisePolynomial<double>& stateTraj, RobotSystem& sys, double rate) {
	std::unique_ptr<systems::Context<double>> diagramContext = sys.diagram->CreateDefaultContext();
	systems::Context<double>& plantContext = sys.diagram->GetMutableSubsystemContext(*sys.plant, diagramContext.get());

	double playbackFrameTime = 0.033;
	int frameCount = (int) std::round((stateTraj.end_time() - stateTraj.start_time()) / rate / playbackFrameTime);
	for(int i = 0; i < frameCount; ++i) {
		double t = ((double) i / (double) frameCount) * (stateTraj.end_time() - stateTraj.start_time()) + stateTraj.start_time();
		auto x = stateTraj.value(t);
		Eigen::VectorXd pos = x.reshaped()(Eigen::seqN(0, sys.plant->num_positions()));
		sys.plant->SetPositions(&plantContext, pos);
		sys.diagram->ForcedPublish(*diagramContext);
		double z = pos[1];
		//std::cout << z << "\n";
		std::this_thread::sleep_for(chrono::milliseconds(33));		//Drake visualizer does not have anything built in for animations
		if(i == 0) {
			std::this_thread::sleep_for(chrono::seconds(3));
		}
	}
}

void plotTraj(const Traj& traj, const StateHelper& help) {
	const std::vector<double>& breaksX = traj.x.get_segment_times();
	const std::vector<double>& breaksU = traj.u.get_segment_times();
	std::vector<double> z;
	std::vector<double> spring;
	for(double t : breaksX) {
		Eigen::VectorXd sample = traj.x.value(t);
		z.push_back(sample[help.floatingPzStateIndex()]);
		spring.push_back(sample[help.springJoint->position_start()]);
	}

	std::vector<double> springInput;
	for(double t : breaksU) {
		Eigen::VectorXd sample = traj.u.value(t);
		springInput.push_back(sample[help.springActuator->input_start()]);
	}

	plt::plot(breaksX, z);
	plt::plot(breaksX, spring);
	plt::show();
	
	plt::plot(breaksU, springInput);
	plt::show();
}