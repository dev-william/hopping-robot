#include <iostream>
#include <chrono>
#include <thread>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/planning/trajectory_optimization/direct_transcription.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/common/yaml/yaml_io.h"

//#include "drake/geometry/meshcat.h"
#include "drake/geometry/drake_visualizer.h"

#include "state_helper.h"

//Command for launching "Meldis" - works with Drake Visualizer API. Maybe I'm supposed to use Meshcat directly?
//env PYTHONPATH=${PYTHONPATH}:/opt/drake/lib/python3.10/site-packages python3 -m pydrake.visualization.meldis -w

//Debugging hack for getting to private members of MultibodyPlant
namespace drake {
namespace multibody {
template<>
class MultibodyPlant<float> {
public:
	void debug(MultibodyPlant<double>& ext) {
		//std::cout << ext.geometry_id_to_body_index_ << "\n";
		std::cout << ext.geometry_id_to_body_index_.size() << "\n";
		std::cout << ext.num_collision_geometries_ << "\n";
		std::cout << ext.num_visual_geometries_ << "\n";
	}
};
}
}

using namespace std;
using namespace drake;
using namespace math;

//Basically does the same things as AddMultibodyPlantSceneGraph().
//Came about because I was trying to make multiple MultibodyPlants in one SceneGraph, which is apparently not intended despite docs not being clear about that
//This must be called before the parser
void connectToSceneGraph(multibody::MultibodyPlant<double>& plant, geometry::SceneGraph<double>& sceneGraph, systems::DiagramBuilder<double>& builder) {
	plant.RegisterAsSourceForSceneGraph(&sceneGraph);
	builder.Connect(plant.get_geometry_poses_output_port(), sceneGraph.get_source_pose_port(plant.get_source_id().value()));
	builder.Connect(sceneGraph.get_query_output_port(), plant.get_geometry_query_input_port());
}

void addGroundPlane(multibody::MultibodyPlant<double>& plant) {
	//Drake crashed at time of contact when a ground plane was added through an SDF rather than this code.
	//MultibodyPlants are just not intended to contact each other for some reason.
	//Ground plane code from Atlas example
	const Vector4<double> groundColor(0.5, 0.5, 0.5, 1.0);
	plant.RegisterVisualGeometry(plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundVisualGeometry", groundColor);
	const double static_friction = 1.0;
	const multibody::CoulombFriction<double> ground_friction(static_friction, static_friction);
	plant.RegisterCollisionGeometry(plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundCollisionGeometry", ground_friction);
}

//Call finalize on returned plant. sceneGraph optional
multibody::MultibodyPlant<double>* createRobotPlant(systems::DiagramBuilder<double>& builder, double timestep, bool use3d, geometry::SceneGraph<double>* sceneGraph, multibody::ModelInstanceIndex* modelIndexOut) {
	multibody::MultibodyPlant<double>* plant = builder.AddSystem<multibody::MultibodyPlant<double>>(timestep);
	if(sceneGraph)
		connectToSceneGraph(*plant, *sceneGraph, builder);
	multibody::Parser parser(plant);
	multibody::ModelInstanceIndex modelIndex = parser.AddModelFromFile("../sdf/robot_2d.sdf");
	if(modelIndexOut)
		*modelIndexOut = modelIndex;

	if(use3d) {
		//Create this explicitly so that it is named
		plant->AddJoint<multibody::QuaternionFloatingJoint>("floating", plant->world_body(), {}, plant->GetBodyByName("base_link"), {});
	}
	else {
		RigidTransformd frameTf = RigidTransformd(RollPitchYaw(90.0*M_PI/180.0, 0.0, 0.0), Eigen::Vector3d::Zero());
		//multibody::FixedOffsetFrame<double> planarFrame("Planar frame", plant.world_body(), frameTf);
		plant->AddJoint<multibody::PlanarJoint>("floating", plant->world_body(), frameTf, plant->GetBodyByName("base_link"), frameTf, Eigen::Vector3d::Zero());
	}

	return plant;
}

trajectories::PiecewisePolynomial<double> optimize() {
	using namespace planning::trajectory_optimization;

	systems::DiagramBuilder<double> builder;
	geometry::SceneGraph<double>& sceneGraph = *builder.AddSystem<geometry::SceneGraph>();

	double timestep = 0.0;        //If 0.0, then system is continuous
	bool use3d = false;

	multibody::ModelInstanceIndex modelIndex;
	multibody::MultibodyPlant<double>& plant = *createRobotPlant(builder, timestep, use3d, &sceneGraph, &modelIndex);

	addGroundPlane(plant);
	plant.Finalize();

	systems::InputPortIndex exportedInputIndex = builder.ExportInput(plant.get_actuation_input_port(), "exported_input");		//Give dircol access to plant input

	std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
	std::unique_ptr<systems::Context<double>> diagramContext = diagram->CreateDefaultContext();
	//systems::State& plantState = diagram->GetMutableSubsystemState(plant, diagramContext.get());
	systems::Context<double>& plantContext = diagram->GetMutableSubsystemContext(plant, diagramContext.get());


	double horizon = 0.4;
	int numTimeSamples = 100;
	double stepDuration = horizon / numTimeSamples;
	//DirectTranscription dirTran(&plant, plantContext, numTimeSamples, plant.get_actuation_input_port(modelIndex).get_index());	//Plant or diagram? sceneGraph needed for collisions
	DirectCollocation dirCol(diagram.get(), *diagramContext, numTimeSamples, stepDuration, stepDuration, exportedInputIndex, true);

	solvers::MathematicalProgram& mp = dirCol.prog();
	dirCol.AddEqualTimeIntervalsConstraints();

	std::cout << "Opt state size " << dirCol.final_state().rows() << "\n";

	StateHelper help(plant, use3d);

	mp.AddConstraint(dirCol.initial_state()[help.elbowJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.initial_state()[help.springJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.initial_state()(Eigen::seqN(help.floatingJoint->position_start(), help.floatingJoint->num_positions())) == Vector3<double>(0.0, 0.45, 0.0));
	mp.AddConstraint(dirCol.initial_state()(Eigen::seqN(plant.num_positions(), plant.num_velocities())) == Eigen::VectorXd::Zero(plant.num_velocities()));

	mp.AddConstraint(dirCol.final_state()[help.elbowJoint->position_start()] == 0.0);
	mp.AddConstraint(dirCol.final_state()(Eigen::seqN(help.floatingJoint->position_start(), help.floatingJoint->num_positions())) == Vector3<double>(0.0, 0.45, 0.0));
	for(int i = 0; i < plant.num_velocities(); ++i) {
		//if(i == help.floatingJoint->velocity_start() + 1)
			//continue;
		mp.AddConstraint(dirCol.final_state()[i + plant.num_positions()] == 0.0);
		//mp.AddConstraint(dirCol.final_state()(Eigen::seqN(plant.num_positions(), plant.num_velocities())) == Eigen::VectorXd::Zero(plant.num_velocities()));
	}

	auto u = dirCol.input();
	dirCol.AddRunningCost(10.0 * u[0] * u[0]);
	dirCol.AddRunningCost(10.0 * u[1] * u[1]);

	double maxElbowTorque = 2.0;
	double maxSpringForce = 5.0;
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] <= maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.elbowActuator->input_start()] >= -maxElbowTorque);
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] <= maxSpringForce);		//Not actually sure which input is which
	dirCol.AddConstraintToAllKnotPoints(u[help.springActuator->input_start()] >= -maxSpringForce);

	Eigen::Vector2d uConstant = Eigen::Vector2d(0.0, 0.0);
	auto uInitial = trajectories::PiecewisePolynomial<double>::ZeroOrderHold({-100, 100}, {uConstant, uConstant});
	//trajectories::PiecewisePolynomial<double> uInitial(Eigen::Vector2d(0.0, 0.0));		//constant hold from -inf to inf fails an assertion in SetInitialTrajectory. Probably a bug
	Eigen::VectorXd xConstant = Eigen::VectorXd::Zero(plant.num_positions());
	xConstant[help.floatingJoint->position_start() + 1] = 0.45;
	//trajectories::PiecewisePolynomial<double> xInitial(xConstant);
	auto xInitial = trajectories::PiecewisePolynomial<double>::ZeroOrderHold({-100, 100}, {xConstant, xConstant});
	std::cout << "Start time " << xInitial.start_time() << ", end time " << xInitial.end_time() << ", segments " << xInitial.get_number_of_segments() << "\n";
	std::cout << "Derivative " << xInitial.EvalDerivative(-100.0);
	dirCol.SetInitialTrajectory(uInitial, xInitial);


	solvers::SnoptSolver solver;
	solvers::SolverOptions options = mp.solver_options();
	options.SetOption(solvers::CommonSolverOption::kPrintToConsole, 1);		//Doesn't seem to do anything
	//options.SetOption(solver.id(), "iterationslimit", 1000000);
	options.SetOption(solver.id(), "Major iterations limit", 500);
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

	systems::DiagramBuilder<double> builder;
	geometry::SceneGraph<double>& sceneGraph = *builder.AddSystem<geometry::SceneGraph>();

	double timestep = 0.0;        //If 0.0, then system is continuous
	bool use3d = false;

	multibody::ModelInstanceIndex modelIndex;
	multibody::MultibodyPlant<double>& plant = *createRobotPlant(builder, timestep, use3d, &sceneGraph, &modelIndex);

	addGroundPlane(plant);
	plant.Finalize();

	systems::InputPortIndex exportedInputIndex = builder.ExportInput(plant.get_actuation_input_port(), "exported_input");		//Give dircol access to plant input

	std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
	std::unique_ptr<systems::Context<double>> diagramContext = diagram->CreateDefaultContext();
	//systems::State& plantState = diagram->GetMutableSubsystemState(plant, diagramContext.get());
	systems::Context<double>& plantContext = diagram->GetMutableSubsystemContext(plant, diagramContext.get());


	solvers::MathematicalProgram mp;

	for(int i = 0; i < 1; ++i) {
		if(i % 2 == 0) {
			double horizon = 0.1;
			int numTimeSamples = 20;
			double stepDuration = horizon / numTimeSamples;
			DirectCollocation dirCol(&plant, plantContext, numTimeSamples, stepDuration, stepDuration, exportedInputIndex, true, &mp);
		}
	}

	return {};
}

void playTrajectory(trajectories::PiecewisePolynomial<double>& stateTraj, systems::Diagram<double>& diagram, multibody::MultibodyPlant<double>& plant) {
	std::unique_ptr<systems::Context<double>> diagramContext = diagram.CreateDefaultContext();
	systems::Context<double>& plantContext = diagram.GetMutableSubsystemContext(plant, diagramContext.get());

	int frameCount = 100;
	for(int i = 0; i < frameCount; ++i) {
		double t = ((double) i / (double) frameCount) * (stateTraj.end_time() - stateTraj.start_time()) + stateTraj.start_time();
		auto x = stateTraj.value(t);
		Eigen::VectorXd pos = x.reshaped()(Eigen::seqN(0, plant.num_positions()));
		plant.SetPositions(&plantContext, pos);
		diagram.ForcedPublish(*diagramContext);
		double z = pos[1];
		std::cout << z << "\n";
		std::this_thread::sleep_for(chrono::milliseconds(33));		//Drake visualizer does not have anything built in for animations
		if(i == 0) {
			std::this_thread::sleep_for(chrono::seconds(3));
		}
	}
}

int main() {
	systems::DiagramBuilder<double> builder;

	geometry::SceneGraph<double>& sceneGraph = *builder.AddSystem<geometry::SceneGraph>();
	sceneGraph.set_name("Scene Graph");

	double timestep = 0.001;        //If 0.0, then system is continuous
	bool use3d = false;

	//Load robot
	multibody::ModelInstanceIndex modelIndex;
	multibody::MultibodyPlant<double>& plant = *createRobotPlant(builder, timestep, use3d, &sceneGraph, &modelIndex);

	addGroundPlane(plant);

	plant.Finalize();       //Processes model after all physical elements added to get it ready for computation

	auto zeroTorque = builder.AddSystem<systems::ConstantVectorSource<double>>(Eigen::Vector2d::Zero());
	builder.Connect(zeroTorque->get_output_port(), plant.get_actuation_input_port());

	//std::shared_ptr<geometry::Meshcat> meshcat = std::make_shared<geometry::Meshcat>();
	//auto& viz = geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, sceneGraph, meshcat);
	geometry::DrakeVisualizer<double> viz;
	viz.AddToBuilder(&builder, sceneGraph);

	std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
	std::unique_ptr<systems::Context<double>> diagramContext = diagram->CreateDefaultContext();
	systems::Context<double>& plantContext = diagram->GetMutableSubsystemContext(plant, diagramContext.get());

	StateHelper help(plant, use3d);

	//std::cout << "State size: " << plantContext.get_state().get_discrete_state().size() << "\n";
	/*VectorX<double> posVec = plant.GetPositions(plantContext, modelIndex);
	std::cout << "Position vector size: " << posVec.rows() << "\n";

	trajectories::PiecewisePolynomial<double> stateTraj = optimize();
	yaml::SaveYamlFile("traj.yaml", stateTraj);		//It appears my version of Drake does not have serialization support for this yet

	//trajectories::PiecewisePolynomial<double> stateTrajLoaded = yaml::LoadYamlFile<trajectories::PiecewisePolynomial<double>>("traj.yaml");

	if(!stateTraj.empty()) {
		//viz.StartRecording();		//Meshcat wasn't working, and it's annoying how it gets disconnected and reconnected. Meldis seems better
		//Investigate TrajectorySource system
		cin.get();
		std::this_thread::sleep_for(chrono::seconds(2));

		playTrajectory(stateTraj, *diagram, plant);

		//viz.StopRecording();
		//viz.PublishRecording();
	}*/

	//help.shoulderJoint->set_angle(&plantContext, 0.0);
	help.elbowJoint->set_angle(&plantContext, 0.02);
	help.springJoint->set_translation(&plantContext, 0.0);

	Eigen::Vector3d initialPos;
	initialPos << 0.0, 0.0, 0.45;
	if(use3d) {
		math::RigidTransformd initialTransform{initialPos};
		plant.SetFreeBodyPoseInWorldFrame(&plantContext, plant.GetBodyByName("base_link"), initialTransform);
	}
	else {
		multibody::PlanarJoint<double>& planarJoint = plant.GetMutableJointByName<multibody::PlanarJoint>("floating");
		planarJoint.set_translation(&plantContext, Eigen::Vector2d(initialPos[0], initialPos[2]));
	}

	diagram->ForcedPublish(*diagramContext);
	std::this_thread::sleep_for(chrono::seconds(4));

	systems::Simulator<double> sim(*diagram, std::move(diagramContext));        //Why is std::move needed?
	sim.set_target_realtime_rate(0.5);
	sim.Initialize();
	sim.AdvanceTo(10.0);
}