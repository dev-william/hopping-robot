#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

class RobotSystem {
public:
	RobotSystem(double timestep, bool use3d, bool less_collision, bool pinned, bool addSceneGraph = true);
	void plantFinalize();		//Optional but certain actions (like addZeroInput()) need the plant to be finalized first
	void finalize();		//Must be called after constructor

	//Call these functions before calling finalize()
	void addZeroInput();
	drake::systems::InputPortIndex exportInput();		//Give dircol access to input at the diagram level

	drake::systems::DiagramBuilder<double> builder;
	drake::geometry::SceneGraph<double>* sceneGraph = nullptr;
	drake::multibody::MultibodyPlant<double>* plant;
	drake::multibody::ModelInstanceIndex modelIndex;
	std::unique_ptr<drake::systems::Diagram<double>> diagram;
	std::unique_ptr<drake::systems::Context<double>> diagramContext;
	drake::systems::Context<double>* plantContext;

	static constexpr double footSphereRadius = 0.01;

private:
	void connectToSceneGraph(drake::multibody::MultibodyPlant<double>& plant, drake::systems::DiagramBuilder<double>& builder);
	void addGroundPlane();
	void createRobotPlant(drake::systems::DiagramBuilder<double>& builder, double timestep, bool use3d, bool less_collision, bool pinned);
};