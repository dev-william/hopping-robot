#include "RobotSystem.h"
#include "Utils.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"

using namespace drake;

int main() {
	trajectories::PiecewisePolynomial<double> stateTrajLoaded = yaml::LoadYamlFile<trajectories::PiecewisePolynomial<double>>("traj.yaml");

	double timestep = 0.001;
	bool use3d = false;			//Todo: autodetect use3d and pinned
	bool pinned = false;

	RobotSystem sys(timestep, use3d, false, pinned);
	geometry::DrakeVisualizer<double> viz;
	viz.AddToBuilder(&sys.builder, *sys.sceneGraph);
	sys.finalize();

	playTrajectory(stateTrajLoaded, sys);
}