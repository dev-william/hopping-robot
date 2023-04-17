#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/analysis/simulator.h"

//#include "drake/geometry/meshcat.h"
#include "drake/geometry/drake_visualizer.h"

//Command for launching "Meldis" - works with Drake Visualizer API. Maybe I'm supposed to use Meshcat directly?
//env PYTHONPATH=${PYTHONPATH}:/opt/drake/lib/python3.10/site-packages python3 -m pydrake.visualization.meldis -w

using namespace drake;

int main() {
    systems::DiagramBuilder<double> builder;

    //geometry::SceneGraph<double>& sceneGraph = *builder.AddSystem<geometry::SceneGraph>();
    //sceneGraph.set_name("Scene Graph");

    //multibody::MultibodyPlant<double>& plant = *builder.AddSystem<multibody::MultibodyPlant>();
    //plant.RegisterAsSourceForSceneGraph(&sceneGraph);

    double timestep = 0.001;        //If 0.0, then system is continuous

    auto plantAndScene = multibody::AddMultibodyPlantSceneGraph(&builder, timestep);
    multibody::MultibodyPlant<double>& plant = plantAndScene.plant;
    geometry::SceneGraph<double>& sceneGraph = plantAndScene.scene_graph;

    multibody::Parser parser(&plant);
    multibody::ModelInstanceIndex plantIndex = parser.AddModelFromFile("/opt/drake/share/drake/examples/acrobot/Acrobot.sdf");
    //Add joint to world?
    plant.Finalize();       //Processes model after all physical elements added to get it ready for computation

    auto zeroTorque = builder.AddSystem<systems::ConstantVectorSource<double>>(0.0);
    builder.Connect(zeroTorque->get_output_port(), plant.get_actuation_input_port());

    geometry::DrakeVisualizer<double> viz;
    viz.AddToBuilder(&builder, sceneGraph);

    std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
    std::unique_ptr<systems::Context<double>> diagramContext = diagram->CreateDefaultContext();
    //systems::State& plantState = diagram->GetMutableSubsystemState(plant, diagramContext.get());
    systems::Context<double>& plantContext = diagram->GetMutableSubsystemContext(plant, diagramContext.get());

    Eigen::Vector2d pos;
    pos << 0.8, 0.1;
    plant.SetPositions(&plantContext, pos);

    systems::Simulator<double> sim(*diagram, std::move(diagramContext));        //Why is std::move needed?
    sim.set_target_realtime_rate(1.0);      //Don't run as fast as possible
    sim.Initialize();
    sim.AdvanceTo(10.0);
}