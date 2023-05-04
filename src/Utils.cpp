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

	unique_ptr<systems::Context<double>> plantContext = help.plant.CreateDefaultContext();		//Unsure if this is legal (skipping the diagram context)
	const multibody::Frame<double>& footFrame = help.plant.GetFrameByName("foot");

	std::vector<double> x, z, spring, elbow, planarAngle;
	std::vector<double> footHeight;
	std::vector<double> normalForce;
	for(double t : breaksX) {
		Eigen::VectorXd sample = traj.x.value(t);
		x.push_back(sample[help.floatingJoint->position_start()]);
		z.push_back(sample[help.floatingPzStateIndex()]);
		spring.push_back(sample[help.springJoint->position_start()]);
		elbow.push_back(sample[help.elbowJoint->position_start()]);
		planarAngle.push_back(sample[help.floatingJoint->position_start() + 2]);

		double currSpringInput = traj.u.value(t)(help.springActuator->input_start(), 0);
		double currFn = (1000.0*spring.back() - currSpringInput) * cos(-planarAngle.back() + elbow.back()) + 9.81*0.3;
		normalForce.push_back(currFn / 100.0);

		Eigen::Vector3d footPos;
		help.plant.SetPositions(plantContext.get(), sample.head(help.plant.num_positions()));
		help.plant.CalcPointsPositions(*plantContext, footFrame, Eigen::Vector3d::Zero(), help.plant.world_frame(), &footPos);
		footHeight.push_back(footPos[2]);
	}

	std::vector<double> springInput;
	std::vector<double> elbowInput;
	for(double t : breaksU) {
		Eigen::VectorXd sample = traj.u.value(t);
		springInput.push_back(sample[help.springActuator->input_start()]);
		elbowInput.push_back(sample[help.elbowActuator->input_start()]);
	}

	plt::plot(breaksX, z, {{"label", "z"}});
	plt::plot(breaksX, spring, {{"label", "spring"}});
	plt::plot(breaksX, normalForce, {{"label", "normal force / 100"}});
	plt::plot(breaksX, footHeight, {{"label", "foot height"}});
	plt::plot(breaksX, x, {{"label", "x"}});
	plt::plot(breaksX, planarAngle, {{"label", "Planar angle"}});
	plt::legend();
	plt::show();
	
	plt::plot(breaksU, springInput);
	plt::show();
}