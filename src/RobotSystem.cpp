#include "RobotSystem.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include <iostream>

using namespace drake;
using namespace math;

//If don't need a sceneGraph, it's apparently possible to create a plant without a diagram
//See https://github.com/RobotLocomotion/drake/blob/master/multibody/plant/test/kuka_iiwa_model_tests.h

RobotSystem::RobotSystem(double timestep, bool use3d, bool less_collision, bool pinned, bool addSceneGraph) {
	if(addSceneGraph)
		sceneGraph = builder.AddSystem<geometry::SceneGraph>();

	createRobotPlant(builder, timestep, use3d, less_collision, pinned);

	if(addSceneGraph)
		addGroundPlane();
}

void RobotSystem::plantFinalize() {
	plant->Finalize();
}

void RobotSystem::finalize() {
	if(!plant->is_finalized())
		plant->Finalize();		//Processes model after all physical elements added to get it ready for computation
	diagram = builder.Build();
	diagramContext = diagram->CreateDefaultContext();
	plantContext = &diagram->GetMutableSubsystemContext(*plant, diagramContext.get());
}

void RobotSystem::addZeroInput() {
	auto zeroTorque = builder.AddSystem<systems::ConstantVectorSource<double>>(Eigen::Vector2d::Zero());
	builder.Connect(zeroTorque->get_output_port(), plant->get_actuation_input_port());
}

systems::InputPortIndex RobotSystem::exportInput() {
	return builder.ExportInput(plant->get_actuation_input_port(), "exported_input");
}

//Basically does the same things as AddMultibodyPlantSceneGraph().
//Came about because I was trying to make multiple MultibodyPlants in one SceneGraph, which is apparently not intended despite docs not being clear about that
//This must be called before the parser
void RobotSystem::connectToSceneGraph(multibody::MultibodyPlant<double>& plant, systems::DiagramBuilder<double>& builder) {
	plant.RegisterAsSourceForSceneGraph(sceneGraph);
	builder.Connect(plant.get_geometry_poses_output_port(), sceneGraph->get_source_pose_port(plant.get_source_id().value()));
	builder.Connect(sceneGraph->get_query_output_port(), plant.get_geometry_query_input_port());
}

void RobotSystem::addGroundPlane() {
	//Drake crashed at time of contact when a ground plane was added through an SDF rather than this code.
	//MultibodyPlants are just not intended to contact each other for some reason.
	//Ground plane code from Atlas example
	const Vector4<double> groundColor(0.5, 0.5, 0.5, 1.0);
	plant->RegisterVisualGeometry(plant->world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundVisualGeometry", groundColor);
	const double static_friction = 1.0;
	const multibody::CoulombFriction<double> ground_friction(static_friction, static_friction);
	plant->RegisterCollisionGeometry(plant->world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundCollisionGeometry", ground_friction);
}

//Must call finalize on returned plant.
void RobotSystem::createRobotPlant(systems::DiagramBuilder<double>& builder, double timestep, bool use3d, bool less_collision, bool pinned) {
	plant = builder.AddSystem<multibody::MultibodyPlant<double>>(timestep);
	if(sceneGraph)
		connectToSceneGraph(*plant, builder);
	multibody::Parser parser(plant);
	std::string modelFile;
	if(use3d) {
		modelFile = "../sdf/robot_2d.sdf";
	}
	else {
		if(pinned) {
			modelFile = "../sdf/robot_2d_pinned.sdf";		//Drake does not support joints between world and a body that isn't the base, unfortunately. Apparently they're working on it
		}
		else {
			if(less_collision)
				modelFile = "../sdf/robot_2d_less_collision.sdf";
			else
				modelFile = "../sdf/robot_2d.sdf";
		}
	}
	modelIndex = parser.AddModelFromFile(modelFile);

	if(use3d) {
		//Create this explicitly so that it is named
		plant->AddJoint<multibody::QuaternionFloatingJoint>("floating", plant->world_body(), {}, plant->GetBodyByName("base_link"), {});
	}
	else {
		if(pinned) {
			math::RigidTransformd footTf = plant->GetFrameByName("foot").GetFixedPoseInBodyFrame();
			//math::RigidTransformd ballOffset{Eigen::Vector3d{0.0, 0.0, footSphereRadius}};		//Todo: decide if I want to have this ballOffset
			plant->AddJoint<multibody::RevoluteJoint>("pin", plant->world_body(), {}, plant->GetBodyByName("foot_link"), footTf, Eigen::Vector3d::UnitY());
		}
		else {
			RigidTransformd frameTf = RigidTransformd(RollPitchYaw(90.0*M_PI/180.0, 0.0, 0.0), Eigen::Vector3d::Zero());
			//multibody::FixedOffsetFrame<double> planarFrame("Planar frame", plant.world_body(), frameTf);
			plant->AddJoint<multibody::PlanarJoint>("floating", plant->world_body(), frameTf, plant->GetBodyByName("base_link"), frameTf, Eigen::Vector3d::Zero());
		}
	}
}