#include <iostream>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/analysis/simulator.h"

//#include "drake/geometry/meshcat.h"
#include "drake/geometry/drake_visualizer.h"

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

using namespace drake;

//The docs say you should only have to do these calls "in extraordinary circumstances" in the MultibodyPlant docs,
//But as far as I am aware, the only alternative is AddMultibodyPlantSceneGraph(), which can only be used once
//Must be called before the parser
void connectToSceneGraph(multibody::MultibodyPlant<double>& plant, geometry::SceneGraph<double>& sceneGraph, systems::DiagramBuilder<double>& builder) {
	plant.RegisterAsSourceForSceneGraph(&sceneGraph);
	builder.Connect(plant.get_geometry_poses_output_port(), sceneGraph.get_source_pose_port(plant.get_source_id().value()));
	builder.Connect(sceneGraph.get_query_output_port(), plant.get_geometry_query_input_port());
}

int main() {
	systems::DiagramBuilder<double> builder;

	geometry::SceneGraph<double>& sceneGraph = *builder.AddSystem<geometry::SceneGraph>();
	sceneGraph.set_name("Scene Graph");

	double timestep = 0.001;        //If 0.0, then system is continuous

	//Load robot
	multibody::MultibodyPlant<double>& plant = *builder.AddSystem<multibody::MultibodyPlant<double>>(timestep);
	connectToSceneGraph(plant, sceneGraph, builder);
	multibody::Parser parser(&plant);
	multibody::ModelInstanceIndex plantIndex = parser.AddModelFromFile("../sdf/robot_2d.sdf");
	//plant.AddJoint<multibody::QuaternionFloatingJoint>("Floating joint", plant.world_body(), {}, plant.GetBodyByName("base_link"), {});

	//Drake crashed at time of contact when a ground plane was added through an SDF rather than this code.
	//No idea why. It seems multiple plants like that are not expected based on the API weirdness
	//Code from Atlas example
	const double static_friction = 1.0;
	const Vector4<double> groundColor(0.5, 0.5, 0.5, 1.0);
	plant.RegisterVisualGeometry(plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundVisualGeometry", groundColor);
	const multibody::CoulombFriction<double> ground_friction(static_friction, static_friction);
	plant.RegisterCollisionGeometry(plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundCollisionGeometry", ground_friction);

	plant.Finalize();       //Processes model after all physical elements added to get it ready for computation

	auto zeroTorque = builder.AddSystem<systems::ConstantVectorSource<double>>(Eigen::Vector3d::Zero());
	builder.Connect(zeroTorque->get_output_port(), plant.get_actuation_input_port());

	geometry::DrakeVisualizer<double> viz;
	viz.AddToBuilder(&builder, sceneGraph);

	std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
	std::unique_ptr<systems::Context<double>> diagramContext = diagram->CreateDefaultContext();
	//systems::State& plantState = diagram->GetMutableSubsystemState(plant, diagramContext.get());
	systems::Context<double>& plantContext = diagram->GetMutableSubsystemContext(plant, diagramContext.get());

	//std::cout << "State size: " << plantContext.get_state().get_discrete_state().size() << "\n";
	VectorX<double> posVec = plant.GetPositions(plantContext, plantIndex);
	std::cout << "Position vector size: " << posVec.rows() << "\n";

	multibody::RevoluteJoint<double>& shoulderJoint = plant.GetMutableJointByName<multibody::RevoluteJoint>("shoulder");
	multibody::RevoluteJoint<double>& elbowJoint = plant.GetMutableJointByName<multibody::RevoluteJoint>("elbow");
	multibody::PrismaticJoint<double>& springJoint = plant.GetMutableJointByName<multibody::PrismaticJoint>("spring");

	shoulderJoint.set_angle(&plantContext, 0.0);
	elbowJoint.set_angle(&plantContext, 0.0);
	springJoint.set_translation(&plantContext, 0.0);

	//Eigen::Vector3d pos;
	//pos << 0.8, 0.1, 0.0;
	//plant.SetPositions(&plantContext, pos);

	Eigen::Vector3d initialPos;
	initialPos << 0.0, 0.0, 0.35;
	math::RigidTransformd initialTransform{initialPos};
	plant.SetFreeBodyPoseInWorldFrame(&plantContext, plant.GetBodyByName("base_link"), initialTransform);

	systems::Simulator<double> sim(*diagram, std::move(diagramContext));        //Why is std::move needed?
	sim.set_target_realtime_rate(0.25);      //Don't run as fast as possible
	sim.Initialize();
	sim.AdvanceTo(10.0);
}