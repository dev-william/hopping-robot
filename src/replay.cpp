#include "RobotSystem.h"
#include "Utils.h"
#include "StateHelper.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"

using namespace drake;
using namespace trajectories;

int main() {
	Traj loaded;
	loaded.x = yaml::LoadYamlFile<PiecewisePolynomial<double>>("stateTraj.yaml");
	loaded.u = yaml::LoadYamlFile<PiecewisePolynomial<double>>("inputTraj.yaml");

	double timestep = 0.001;
	bool use3d = false;			//Todo: autodetect use3d and pinned
	bool pinned = false;

	RobotSystem sys(timestep, use3d, false, pinned);
	geometry::DrakeVisualizer<double> viz;
	viz.AddToBuilder(&sys.builder, *sys.sceneGraph);
	sys.finalize();
	StateHelper help(*sys.plant);

	playTrajectory(loaded.x, sys, 0.02);

	plotTraj(loaded, help);
}