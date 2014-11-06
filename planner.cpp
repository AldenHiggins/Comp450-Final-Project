/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/spaces/SO2StateSpace.h>
#include <omplapp/config.h>
#include <boost/math/constants/constants.hpp>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleSetup.h>
#include <fstream>
#include <math.h>

#define CIRCLE_ROBOT_RADIUS .2
#define SQUARE_ROBOT_LENGTH .3

#define CAR_MIN_ENV_VALUE -10
#define CAR_MAX_ENV_VALUE 10

using namespace ompl;

// Check the environment for a pendulum
bool pendulumStateValid(const control::SpaceInformation *si, const base::State *state)
{
    return true;
}

// Definition of the ODE for the pendulum
void pendulumODE (const control::ODESolver::StateType& q, const control::Control* control, control::ODESolver::StateType& qdot)
{
    const double *u = control->as<control::RealVectorControlSpace::ControlType>()->values;

    // Zero out qdot
    qdot.resize (q.size (), 0);
    // Set qdot
    qdot[0] = q[1];
    qdot[1] = -9.81 * cos(q[0]) + u[0];
}

// This is a callback method invoked after numerical integration.
void pendulumPostIntegration (const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State *result)
{
    // Normalize orientation between 0 and 2*pi
    base::SO2StateSpace SO2;
    SO2.enforceBounds (result->as<base::CompoundStateSpace::StateType>()->as<base::SO2StateSpace::StateType>(0));
}

void pendulumPlan()
{
    // Define the state space
    base::StateSpacePtr SO2(new base::SO2StateSpace());
    base::StateSpacePtr angularVelocity(new base::RealVectorStateSpace(1));
    base::StateSpacePtr stateSpace = SO2 + angularVelocity;

    // Bind angular velocity bounds
    base::RealVectorBounds velocityBound(1);
    velocityBound.setLow(-13.4);
    velocityBound.setHigh(13.4);
    angularVelocity->as<base::RealVectorStateSpace>()->setBounds(velocityBound);

    // Define start and goal states
    base::ScopedState<> start(stateSpace);
    base::ScopedState<> goal(stateSpace);

    start[0] = -1 * boost::math::constants::pi<double>()/2;
    start[1] = 0;

    goal[0] = boost::math::constants::pi<double>()/2;
    goal[1] = 0;

    // Enables KPIECE planner
    // stateSpace->registerDefaultProjection(base::ProjectionEvaluatorPtr(new PendulumProjection(stateSpace)));
    // Define control space
    control::ControlSpacePtr cmanifold(new control::RealVectorControlSpace(stateSpace, 1));

    // Set bounds of control space
    base::RealVectorBounds cbounds(1);
    cbounds.setLow(-2);
    cbounds.setHigh(2);
    cmanifold->as<control::RealVectorControlSpace>()->setBounds(cbounds);


    // Set up control space
    control::SimpleSetup setup(cmanifold);

    // Set state validity checking for this space
    setup.setStateValidityChecker(boost::bind(&pendulumStateValid, setup.getSpaceInformation().get(), _1));
    // Add ODE solver for the space
    control::ODESolverPtr odeSolver(new control::ODEBasicSolver<> (setup.getSpaceInformation(), &pendulumODE));
    // Add post integration function
    setup.setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, &pendulumPostIntegration));
    // Change planner variables
    setup.getSpaceInformation()->setPropagationStepSize(.1);
    setup.getSpaceInformation()->setMinMaxControlDuration(1, 3); // 2 3 default

    setup.setStartAndGoalStates(start, goal, 0.05);
    setup.setup();
    // Give the problem 30 seconds to solve
    if(setup.solve(30))
    {
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        setup.getSolutionPath().asGeometric().printAsMatrix(std::cout);

        std::ofstream fout("pathResults");
        // Start by writing out environment
        // First write the axis dimensions
        fout << -13.14 << ":" << 13.14 << std::endl;
        // Now write the objects
        fout << 0 << std::endl;
        setup.getSolutionPath().asGeometric().print(fout);
    }
    else
    {
       std::cout << "No solution found" << std::endl; 
    }
}

// State checker for the car robot
bool carStateValid(const control::SpaceInformation *si, const base::State *state)
{

    return true;
}

// Definition of the ODE for the car
void carODE(const control::ODESolver::StateType& q, const control::Control* control, control::ODESolver::StateType& qdot)
{
    const double *u = control->as<control::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];

    // Zero out qdot
    qdot.resize (q.size (), 0);

    qdot[0] = q[3] * cos(theta);
    qdot[1] = q[3] * sin(theta);
    qdot[2] = u[0];
    qdot[3] = u[1];
}

// This is a callback method invoked after numerical integration for the car robot
void carPostIntegration(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State *result)
{
    // Normalize orientation between 0 and 2*pi
    base::SO2StateSpace SO2;
    SO2.enforceBounds (result->as<base::CompoundStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(0)
        ->as<base::SO2StateSpace::StateType>(1));
}

void carPlan()
{
    // Define the state space
    base::StateSpacePtr SE2(new base::SE2StateSpace());
    base::StateSpacePtr velocity(new base::RealVectorStateSpace(1));
    base::StateSpacePtr stateSpace = SE2 + velocity;

    // Bind x/y bounds of the space
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    SE2->as<base::SE2StateSpace>()->setBounds(bounds);

    // Bind velocity bounds
    base::RealVectorBounds velocityBound(1);
    velocityBound.setLow(-4);
    velocityBound.setHigh(4);
    velocity->as<base::RealVectorStateSpace>()->setBounds(velocityBound);

    // Define start and goal states
    base::ScopedState<> start(stateSpace);
    base::ScopedState<> goal(stateSpace);

    start[0] = -5;
    start[1] = -5;
    start[2] = 0;
    start[3] = 0;

    goal[0] = 5;
    goal[1] = 5;
    goal[2] = 0;
    goal[3] = 0;

    // Enables KPIECE planner
    // stateSpace->registerDefaultProjection(base::ProjectionEvaluatorPtr(new CarProjection(stateSpace)));
    // Define control space
    control::ControlSpacePtr cmanifold(new control::RealVectorControlSpace(stateSpace, 2));

    // Set bounds of control space
    base::RealVectorBounds cbounds(2);
    // Bound for angular velocity
    cbounds.setLow(0,-1);
    cbounds.setHigh(0,1);
    // Bound for velocity
    cbounds.setLow(1,-20);
    cbounds.setHigh(1,20);
    cmanifold->as<control::RealVectorControlSpace>()->setBounds(cbounds);

    // Set up control space
    control::SimpleSetup setup(cmanifold);

    // Set state validity checking for this space
    setup.setStateValidityChecker(boost::bind(&carStateValid, setup.getSpaceInformation().get(), _1));
    // Add ODE solver for the space
    control::ODESolverPtr odeSolver(new control::ODEBasicSolver<> (setup.getSpaceInformation(), &carODE));
    // Add post integration function
    setup.setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, &carPostIntegration));
    // Change planner variables
    setup.getSpaceInformation()->setPropagationStepSize(.1);
    setup.getSpaceInformation()->setMinMaxControlDuration(1, 3); // 2 3 default

    setup.setStartAndGoalStates(start, goal, 0.05);
    setup.setup();
    // Give the problem 30 seconds to solve
    if(setup.solve(30))
    {
        /// print the path to screen
        std::cout << "Found solution:" << std::endl;
        setup.getSolutionPath().asGeometric().printAsMatrix(std::cout);

        // std::ofstream fout("pathResults");
        // // Start by writing out environment
        // // First write the axis dimensions
        // fout << carEnvironment->getStartAxis() << ":" << carEnvironment->getEndAxis() << std::endl;
        // // Now write the objects
        // std::vector<Rectangle> objects = carEnvironment->getObstacles();
        // fout << objects.size() << std::endl;
        // for (int objectIterator = 0; objectIterator < objects.size(); objectIterator++){
        //     Rectangle object = objects[objectIterator];
        //     fout << object.bottomLeftCorner.x << ":" << object.bottomLeftCorner.y << ":"
        //      << object.bottomRightCorner.x << ":" << object.bottomRightCorner.y << ":"
        //      << object.topLeftCorner.x << ":" << object.topLeftCorner.y << ":"
        //      << object.topRightCorner.x << ":" << object.topRightCorner.y << std::endl;
        // }
        // setup.getSolutionPath().asGeometric().print(fout);
    }
    else
    {
       std::cout << "No solution found" << std::endl; 
    }
}

int main()
{
    // Initialize car environment
    // carEnvironment = new Environment();

    // Choose type of scenario to plan for
    int env;
    do
    {
        std::cout << "Plan for: "<< std::endl;
        std::cout << " (1) Pendulum" << std::endl;
        std::cout << " (2) Car Robot" << std::endl;

        std::cin >> env;
    } while (env < 1 || env > 2);

    if (env == 1)
    {
        pendulumPlan();
    }
    else
    {
        carPlan();
    }
}
