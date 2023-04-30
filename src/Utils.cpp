#include "Utils.h"
#include "RobotSystem.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace drake;
using namespace std;

void playTrajectory(trajectories::PiecewisePolynomial<double>& stateTraj, RobotSystem& sys) {
	std::unique_ptr<systems::Context<double>> diagramContext = sys.diagram->CreateDefaultContext();
	systems::Context<double>& plantContext = sys.diagram->GetMutableSubsystemContext(*sys.plant, diagramContext.get());

	int frameCount = 100;
	for(int i = 0; i < frameCount; ++i) {
		double t = ((double) i / (double) frameCount) * (stateTraj.end_time() - stateTraj.start_time()) + stateTraj.start_time();
		auto x = stateTraj.value(t);
		Eigen::VectorXd pos = x.reshaped()(Eigen::seqN(0, sys.plant->num_positions()));
		sys.plant->SetPositions(&plantContext, pos);
		sys.diagram->ForcedPublish(*diagramContext);
		double z = pos[1];
		std::cout << z << "\n";
		std::this_thread::sleep_for(chrono::milliseconds(33));		//Drake visualizer does not have anything built in for animations
		if(i == 0) {
			std::this_thread::sleep_for(chrono::seconds(3));
		}
	}
}