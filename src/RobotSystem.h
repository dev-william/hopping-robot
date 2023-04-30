#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

class RobotSystem {
public:
	RobotSystem(double timestep, bool use3d, bool less_collision);
	void finalize();		//Must be called after constructor

	//Call these functions before calling finalize()
	void addZeroInput();

	drake::systems::DiagramBuilder<double> builder;
	drake::geometry::SceneGraph<double>* sceneGraph;
	drake::multibody::MultibodyPlant<double>* plant;
	drake::multibody::ModelInstanceIndex modelIndex;
	drake::systems::InputPortIndex exportedInputIndex;
	std::unique_ptr<drake::systems::Diagram<double>> diagram;
	std::unique_ptr<drake::systems::Context<double>> diagramContext;
	drake::systems::Context<double>* plantContext;

private:
	void connectToSceneGraph(drake::multibody::MultibodyPlant<double>& plant, drake::systems::DiagramBuilder<double>& builder);
	void addGroundPlane();
	void createRobotPlant(drake::systems::DiagramBuilder<double>& builder, double timestep, bool use3d, bool less_collision);
};