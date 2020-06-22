#include <iostream>
#include <stdio.h>
#include "Simbody.h"
using namespace SimTK;


int main(){
/*in this system, I develop 2 pendulums hanging one over the other.
 * The pendulums are massless, other than at the end of their length, 
 * where a big point mass (or ball) holds the mass.
 * it's a classical problem in my classical mechanics class, so it's fun simulating it.
 */

    // defining the system and gravity in the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -YAxis, 9.8);

    // Describe mass and visualization properties for a generic body.
    Body::Rigid bodyInfo(MassProperties(1.0, Vec3(0), UnitInertia(1)));
    bodyInfo.addDecoration(Transform(), DecorativeSphere(0.1));

    //Adding in our masses to represent the system. pendulum2 attatches to pendulum1 here.
    MobilizedBody::Pin pendulum1(matter.Ground(), Transform(Vec3(0)), bodyInfo, Transform(Vec3(0, 1, 0)));
    MobilizedBody::Pin pendulum2(pendulum1, Transform(Vec3(0)),
            bodyInfo, Transform(Vec3(0, 1, 0)));

    // visualizations
    system.setUseUniformBackground(true);
    Visualizer viz(system);
    system.addEventReporter(new Visualizer::Reporter(viz, 0.01));

    // Initialize the system and set the state.
    State state = system.realizeTopology();
    pendulum2.setRate(state, 5.0); //we are supposing that the system moves at 5.0 m/s

    // run the simulation for 30 seconds.
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    ts.initialize(state);
    ts.stepTo(30.0);

}
