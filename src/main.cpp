#include <iostream>

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
	const double static_friction = 1.0;
	const Vector4<double> groundColor(0.5, 0.5, 0.5, 1.0);
	plant.RegisterVisualGeometry(plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundVisualGeometry", groundColor);
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

	//plant.AddJoint<multibody::QuaternionFloatingJoint>("Floating joint", plant.world_body(), {}, plant.GetBodyByName("base_link"), {});
	if(!use3d) {
		RigidTransformd frameTf = RigidTransformd(RollPitchYaw(90.0*M_PI/180.0, 0.0, 0.0), Eigen::Vector3d::Zero());
		//multibody::FixedOffsetFrame<double> planarFrame("Planar frame", plant.world_body(), frameTf);
		plant->AddJoint<multibody::PlanarJoint>("2d floating joint", plant->world_body(), frameTf, plant->GetBodyByName("base_link"), frameTf, Eigen::Vector3d::Zero());
	}

	return plant;
}

int main() {
	systems::DiagramBuilder<double> builder;

	geometry::SceneGraph<double>& sceneGraph = *builder.AddSystem<geometry::SceneGraph>();
	sceneGraph.set_name("Scene Graph");

	double timestep = 0.001;        //If 0.0, then system is continuous

	//Load robot
	bool use3d = false;
	multibody::ModelInstanceIndex modelIndex;
	multibody::MultibodyPlant<double>& plant = *createRobotPlant(builder, timestep, false, &sceneGraph, &modelIndex);

	addGroundPlane(plant);

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
	VectorX<double> posVec = plant.GetPositions(plantContext, modelIndex);
	std::cout << "Position vector size: " << posVec.rows() << "\n";

	multibody::RevoluteJoint<double>& shoulderJoint = plant.GetMutableJointByName<multibody::RevoluteJoint>("shoulder");
	multibody::RevoluteJoint<double>& elbowJoint = plant.GetMutableJointByName<multibody::RevoluteJoint>("elbow");
	multibody::PrismaticJoint<double>& springJoint = plant.GetMutableJointByName<multibody::PrismaticJoint>("spring");

	shoulderJoint.set_angle(&plantContext, 0.0);
	elbowJoint.set_angle(&plantContext, 0.2);
	springJoint.set_translation(&plantContext, 0.0);

	//Eigen::Vector3d pos;
	//pos << 0.8, 0.1, 0.0;
	//plant.SetPositions(&plantContext, pos);

	Eigen::Vector3d initialPos;
	initialPos << 0.0, 0.0, 0.45;
	if(use3d) {
		math::RigidTransformd initialTransform{initialPos};
		plant.SetFreeBodyPoseInWorldFrame(&plantContext, plant.GetBodyByName("base_link"), initialTransform);
	}
	else {
		multibody::PlanarJoint<double>& planarJoint = plant.GetMutableJointByName<multibody::PlanarJoint>("2d floating joint");
		planarJoint.set_translation(&plantContext, Eigen::Vector2d(initialPos[0], initialPos[2]));
	}

	systems::Simulator<double> sim(*diagram, std::move(diagramContext));        //Why is std::move needed?
	sim.set_target_realtime_rate(0.5);
	sim.Initialize();
	sim.AdvanceTo(10.0);
}