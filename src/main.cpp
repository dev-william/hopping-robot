#include <iostream>
#include <chrono>
#include <thread>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/planning/trajectory_optimization/direct_transcription.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"
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

trajectories::PiecewisePolynomial<double> optimize() {
	using namespace planning::trajectory_optimization;

	double timestep = 0.0;        //If 0.0, then system is continuous
	bool use3d = false;

	RobotSystem sys(timestep, use3d, true, false);

	systems::InputPortIndex exportedInputIndex = sys.builder.ExportInput(sys.plant->get_actuation_input_port(), "exported_input");		//Give dircol access to plant input

	sys.finalize();


	double horizon = 0.36;
	int numTimeSamples = 80;
	double stepDuration = horizon / numTimeSamples;
	//DirectTranscription dirTran(&plant, plantContext, numTimeSamples, plant.get_actuation_input_port(modelIndex).get_index());	//Plant or diagram? sceneGraph needed for collisions
	DirectCollocation dirCol(sys.diagram.get(), *sys.diagramContext, numTimeSamples, stepDuration, stepDuration, exportedInputIndex, true);

	solvers::MathematicalProgram& mp = dirCol.prog();
	dirCol.AddEqualTimeIntervalsConstraints();

	std::cout << "Opt state size " << dirCol.final_state().rows() << "\n";

	StateHelper help(*sys.plant, use3d);

	mp.AddConstraint(dirCol.initial_state()[help.elbowJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.initial_state()[help.springJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.initial_state()(Eigen::seqN(help.floatingJoint->position_start(), help.floatingJoint->num_positions())) == Vector3<double>(0.0, 0.45, 0.0));
	mp.AddConstraint(dirCol.initial_state()(Eigen::seqN(sys.plant->num_positions(), sys.plant->num_velocities())) == Eigen::VectorXd::Zero(sys.plant->num_velocities()));

	mp.AddConstraint(dirCol.final_state()[help.elbowJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.final_state()(Eigen::seqN(help.floatingJoint->position_start(), help.floatingJoint->num_positions())) == Vector3<double>(0.0, 0.45, 0.0));
	for(int i = 0; i < sys.plant->num_velocities(); ++i) {
		//if(i == help.floatingJoint->velocity_start() + 1)
			//continue;
		mp.AddConstraint(dirCol.final_state()[i + sys.plant->num_positions()] == 0.0);
		//mp.AddConstraint(dirCol.final_state()(Eigen::seqN(plant.num_positions(), plant.num_velocities())) == Eigen::VectorXd::Zero(plant.num_velocities()));
	}

	auto u = dirCol.input();
	dirCol.AddRunningCost(10.0 * u[0] * u[0]);
	dirCol.AddRunningCost(10.0 * u[1] * u[1]);

	double maxElbowTorque = 2.0;
	double maxSpringForce = 5.0;
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] <= maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] >= -maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] <= maxSpringForce);
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] >= -maxSpringForce);

	Traj initial = makeInitialGuess(help, Eigen::VectorXd{{0.0, 0.45}}, Eigen::VectorXd{{0.0, 0.45}}, horizon);
	dirCol.SetInitialTrajectory(initial.u, initial.x);


	solvers::SnoptSolver solver;
	solvers::SolverOptions options = mp.solver_options();
	options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);		//Doesn't seem to do anything
	options.SetOption(solver.id(), "Iterations limit", 1000000);		//Increase iterations limit a lot
	//options.SetOption(solver.id(), "Major iterations limit", 500);
	options.SetOption(solver.id(), "Major optimality tolerance", 1e-4);
	mp.SetSolverOptions(options);

	auto start = chrono::steady_clock::now();
	solvers::MathematicalProgramResult res = solver.Solve(mp);
	auto end = chrono::steady_clock::now();
	std::cout << "Solve time: " << chrono::duration<double>(end - start).count() << " s\n\n";

	std::cout << "Success: " << res.is_success() << "\n";
	std::cout << "Solver: " << res.get_solver_id().name() << "\n";
	solvers::SnoptSolverDetails snoptDetails = res.get_solver_details<solvers::SnoptSolver>();
	std::cout << "SNOPT info code: " << snoptDetails.info << "\n";

	//if(res.is_success())
	return dirCol.ReconstructStateTrajectory(res);
}

trajectories::PiecewisePolynomial<double> optimize2() {
	using namespace planning::trajectory_optimization;

	double timestep = 0.0;        //If 0.0, then system is continuous
	bool use3d = false;

	RobotSystem sys(timestep, use3d, true, false);

	systems::InputPortIndex exportedInputIndex = sys.builder.ExportInput(sys.plant->get_actuation_input_port(), "exported_input");		//Give dircol access to plant input

	sys.finalize();


	solvers::MathematicalProgram mp;

	for(int i = 0; i < 1; ++i) {
		if(i % 2 == 0) {
			double horizon = 0.1;
			int numTimeSamples = 20;
			double stepDuration = horizon / numTimeSamples;
			DirectCollocation dirCol(sys.plant, *sys.plantContext, numTimeSamples, stepDuration, stepDuration, exportedInputIndex, true, &mp);
		}
	}

	return {};
}

int main() {
	double timestep = 0.001;        //If 0.0, then system is continuous
	bool use3d = false;
	bool pinned = true;

	RobotSystem sys(timestep, use3d, false, pinned);
	sys.addZeroInput();

	//std::shared_ptr<geometry::Meshcat> meshcat = std::make_shared<geometry::Meshcat>();
	//auto& viz = geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, sceneGraph, meshcat);
	geometry::DrakeVisualizer<double> viz;
	viz.AddToBuilder(&sys.builder, *sys.sceneGraph);

	sys.finalize();

	systems::Context<double>& plantContext = *sys.plantContext;
	StateHelper help(*sys.plant, use3d);

	//std::cout << "State size: " << plantContext.get_state().get_discrete_state().size() << "\n";
	VectorX<double> posVec = sys.plant->GetPositions(plantContext, sys.modelIndex);
	std::cout << "Position vector size: " << posVec.rows() << "\n";

	/*trajectories::PiecewisePolynomial<double> stateTraj = optimize();
	//Traj initialGuess = makeInitialGuess(help, Eigen::VectorXd{{0.0, 0.45}}, Eigen::VectorXd{{0.5, 0.45}}, 2.0);
	//trajectories::PiecewisePolynomial<double> stateTraj = initialGuess.x;
	yaml::SaveYamlFile("traj.yaml", stateTraj);

	//trajectories::PiecewisePolynomial<double> stateTrajLoaded = yaml::LoadYamlFile<trajectories::PiecewisePolynomial<double>>("traj.yaml");

	if(!stateTraj.empty()) {
		//viz.StartRecording();		//Meshcat wasn't working, and it's annoying how it gets disconnected and reconnected. Meldis seems better
		//Investigate TrajectorySource system
		cin.get();
		std::this_thread::sleep_for(chrono::seconds(2));

		playTrajectory(stateTraj, sys);

		//viz.StopRecording();
		//viz.PublishRecording();
	}*/



	//Simulate uncontrolled robot
	//help.shoulderJoint->set_angle(&plantContext, 0.0);
	help.elbowJoint->set_angle(&plantContext, 0.02);
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
	//std::cout << "Base body name: " << sys.plant->GetUniqueFreeBaseBodyOrThrow(sys.modelIndex).name() << "\n";

	sys.diagram->ForcedPublish(*sys.diagramContext);
	std::this_thread::sleep_for(chrono::seconds(4));

	systems::Simulator<double> sim(*sys.diagram, std::move(sys.diagramContext));
	sim.set_target_realtime_rate(0.5);
	sim.Initialize();
	sim.AdvanceTo(10.0);
}