#include "RobotSystem.h"
#include "Utils.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/trajectory_source.h"

#include <iostream>

using namespace drake;
using namespace trajectories;

int main() {
	PiecewisePolynomial<double> stateTraj = yaml::LoadYamlFile<PiecewisePolynomial<double>>("stateTraj.yaml");
	PiecewisePolynomial<double> inputTraj = yaml::LoadYamlFile<PiecewisePolynomial<double>>("inputTraj.yaml");

	double timestep = 0.001;
	bool use3d = false;
	bool pinned = false;

	RobotSystem sys(timestep, use3d, false, pinned);
	sys.plantFinalize();
	systems::TrajectorySource<double>* src = sys.builder.AddSystem<systems::TrajectorySource>(inputTraj);
	sys.builder.Connect(src->get_output_port(), sys.plant->get_actuation_input_port());

	geometry::DrakeVisualizer<double> viz;
	viz.AddToBuilder(&sys.builder, *sys.sceneGraph);
	sys.finalize();


	Eigen::VectorXd initialState = stateTraj.value(0.0);
	sys.plant->SetPositionsAndVelocities(sys.plantContext, initialState);

	systems::Simulator<double> sim(*sys.diagram, std::move(sys.diagramContext));
	sim.set_target_realtime_rate(0.5);
	sim.Initialize();
	sim.AdvanceTo(inputTraj.end_time());

	std::cout << "Final positions: " << sys.plant->GetPositions(*sys.plantContext) << "\n";
	std::cout << "Final velocities: " << sys.plant->GetVelocities(*sys.plantContext) << "\n";
}