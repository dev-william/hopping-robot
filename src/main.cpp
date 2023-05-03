#include <iostream>
#include <chrono>
#include <thread>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/common/yaml/yaml_io.h"

//#include "drake/geometry/meshcat.h"
#include "drake/geometry/drake_visualizer.h"

#include "RobotSystem.h"
#include "StateHelper.h"
#include "Optimize.h"
#include "Utils.h"

//Command for launching "Meldis" - works with Drake Visualizer API. Maybe I'm supposed to use Meshcat directly?
//env PYTHONPATH=${PYTHONPATH}:/opt/drake/lib/python3.10/site-packages python3 -m pydrake.visualization.meldis -w

using namespace std;
using namespace drake;
using namespace math;


int main() {
	double timestep = 0.001;        //If 0.0, then system is continuous
	bool use3d = false;
	bool pinned = false;

	RobotSystem sys(timestep, use3d, false, pinned);
	sys.plantFinalize();
	sys.addZeroInput();

	//std::shared_ptr<geometry::Meshcat> meshcat = std::make_shared<geometry::Meshcat>();
	//auto& viz = geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, sceneGraph, meshcat);
	geometry::DrakeVisualizer<double> viz;
	viz.AddToBuilder(&sys.builder, *sys.sceneGraph);

	sys.finalize();

	systems::Context<double>& plantContext = *sys.plantContext;
	StateHelper help(*sys.plant);


	bool allowVariableTime = true;
	HybridOptimization hybrid;
	hybrid.createProblem(Eigen::VectorXd{{0.0, 0.45}}, Eigen::VectorXd{{0.0, 0.5}}, 0.4, allowVariableTime);
	hybrid.solve();
	Traj optResult = hybrid.reconstructFullTraj();
	trajectories::PiecewisePolynomial<double> stateTraj = optResult.x;
	cout << "Trajectory duration: " << optResult.x.end_time() << " s\n";

	//trajectories::PiecewisePolynomial<double> stateTraj = optimizeOverScene();
	//FootstepGuesser guesser(Eigen::VectorXd{{0.0, 0.55}}, Eigen::VectorXd{{0.5, 0.55}}, 2.0);
	//Traj initialGuess = guesser.makeFullGuess(false, help);
	//trajectories::PiecewisePolynomial<double> stateTraj = initialGuess.x;
	yaml::SaveYamlFile("stateTraj.yaml", stateTraj);
	yaml::SaveYamlFile("inputTraj.yaml", optResult.u);

	//trajectories::PiecewisePolynomial<double> stateTrajLoaded = yaml::LoadYamlFile<trajectories::PiecewisePolynomial<double>>("traj.yaml");

	if(!stateTraj.empty()) {
		//viz.StartRecording();		//Meshcat wasn't working, and it's annoying how it gets disconnected and reconnected. Meldis seems better
		//Investigate TrajectorySource system
		cin.get();
		std::this_thread::sleep_for(chrono::seconds(2));

		playTrajectory(stateTraj, sys, 0.5);

		//viz.StopRecording();
		//viz.PublishRecording();
	}



	//Simulate uncontrolled robot
	//help.shoulderJoint->set_angle(&plantContext, 0.0);
	/*help.elbowJoint->set_angle(&plantContext, 0.02);
	help.springJoint->set_translation(&plantContext, 0.0);

	Eigen::Vector3d initialPos;
	initialPos << 0.0, 0.0, 0.45;
	if(!pinned) {
		if(use3d) {
			math::RigidTransformd initialTransform{initialPos};
			sys.plant->SetFreeBodyPoseInWorldFrame(&plantContext, sys.plant->GetBodyByName("base_link"), initialTransform);
		}
		else {
			multibody::PlanarJoint<double>& planarJoint = sys.plant->GetMutableJointByName<multibody::PlanarJoint>("floating");
			planarJoint.set_translation(&plantContext, Eigen::Vector2d(initialPos[0], initialPos[2]));
		}
	}

	sys.diagram->ForcedPublish(*sys.diagramContext);
	std::this_thread::sleep_for(chrono::seconds(4));

	systems::Simulator<double> sim(*sys.diagram, std::move(sys.diagramContext));
	sim.set_target_realtime_rate(0.5);
	sim.Initialize();
	sim.AdvanceTo(10.0);*/
}