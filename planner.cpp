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

#define JOINT_LENGTH 1
#define JOINT_MASS 1

using namespace ompl;

int numberOfJoints;

// State checker for the car robot
bool jointManipulatorStateValid(const control::SpaceInformation *si, const base::State *state)
{
    return true;
}

// Definition of the ODE for the car
void jointManipulatorODE(const control::ODESolver::StateType& q, const control::Control* control, control::ODESolver::StateType& qdot)
{
    const double *u = control->as<control::RealVectorControlSpace::ControlType>()->values;
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];
    const double q4 = q[4];
    const double q5 = q[5];

    // TODO: might be broken
    std::vector<double> jointEndsX;
    std::vector<double> jointEndsY;
    jointEndsX.push_back(JOINT_LENGTH * cos(q[0]));
    jointEndsY.push_back(JOINT_LENGTH * sin(q[0]));

    double qSum = q[0];

    for (int i = 1; i < numberOfJoints; i++){
        qSum += q[i * 2];
        jointEndsX.push_back(jointEndsX[i-1] + JOINT_LENGTH * cos(qSum));
        jointEndsY.push_back(jointEndsY[i-1] + JOINT_LENGTH * sin(qSum));
    }

    // Initialize inertia matrix
    std::vector<std::vector<double> > inertiaMatrix;
    for (int i = 0; i < numberOfJoints; i++){
        std::vector<double> row;
        for (int j = 0; j < numberOfJoints; j++){
            row.push_back(0);
        }
        inertiaMatrix.push_back(row);
    }
    // Add the inertia mat at each step
    qSum = q[0];
    for (int i = 0; i < numberOfJoints; i++){
        qSum += q[i * 2];
        std::vector<double> jacobianLX;
        std::vector<double> jacobianLY;
        for(int j = 0; j < numberOfJoints; j++){
            if (j <= i ){
                jacobianLX.push_back(-1 * jointEndsY[j] + jointEndsY[i] - JOINT_LENGTH/2 * sin(qSum));
                jacobianLY.push_back(jointEndsX[j] + -1 * jointEndsX[i] + JOINT_LENGTH/2 * cos(qSum));
            }
            else {
                jacobianLX.push_back(0);
                jacobianLY.push_back(0);
            }
        }

        for (int r = 0; r < numberOfJoints; r++){
            for (int c = 0; c < numberOfJoints; c++){
                inertiaMatrix[r][c] = JOINT_MASS * JOINT_MASS * (jacobianLX[r] * jacobianLX[c] + jacobianLY[r] * jacobianLY[c]);    
                if (r <= i && c <= i){
                    // Add in moment of inertia
                    inertiaMatrix[r][c] += ((JOINT_MASS * JOINT_LENGTH * JOINT_LENGTH)/12.);
                }            
            }
        }
    }



    // // Zero out qdot
    // qdot.resize (q.size (), 0);

    // qdot[0] = q[3] * cos(theta);
    // qdot[1] = q[3] * sin(theta);
    // qdot[2] = u[0];
    // qdot[3] = u[1];
}

// This is a callback method invoked after numerical integration for the car robot
void jointManipulatorPostIntegration(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State *result)
{
    // Normalize orientation between 0 and 2*pi
    base::SO2StateSpace SO2;
    for (int i = 0; i < numberOfJoints; i++){
        SO2.enforceBounds (result->as<base::CompoundStateSpace::StateType>()->as<base::SO2StateSpace::StateType>(i * 2));
    }
}

void carPlan()
{
    // Define the state space
    // base::StateSpacePtr SE2(new base::SE2StateSpace());
    // base::StateSpacePtr velocity(new base::RealVectorStateSpace(1));
    base::StateSpacePtr stateSpace;// = SE2 + velocity;
    for (int i = 0; i < numberOfJoints; i++){
        base::StateSpacePtr SO2(new base::SO2StateSpace());
        // // Bind x/y bounds of the space
        // base::RealVectorBounds bounds(2);
        // bounds.setLow(-10);
        // bounds.setHigh(10);
        // SO2->as<base::SO2StateSpace>()->setBounds(bounds);
        base::StateSpacePtr velocity(new base::RealVectorStateSpace(1));
        // Bind velocity bounds
        base::RealVectorBounds velocityBound(1);
        velocityBound.setLow(-4);
        velocityBound.setHigh(4);
        velocity->as<base::RealVectorStateSpace>()->setBounds(velocityBound);
        stateSpace = stateSpace + SO2 + velocity;
    }

    

    

    // Define start and goal states
    base::ScopedState<> start(stateSpace);
    base::ScopedState<> goal(stateSpace);
    for (int i = 0; i < numberOfJoints * 2; i++){
        start[i] = 0;
        if (i % 2 == 0){
            goal[i] = i;
        }
        else{
            goal[i] = 0;
        }
    }

    // Enables KPIECE planner
    // stateSpace->registerDefaultProjection(base::ProjectionEvaluatorPtr(new CarProjection(stateSpace)));
    // Define control space
    control::ControlSpacePtr cmanifold(new control::RealVectorControlSpace(stateSpace, numberOfJoints));

    // Set bounds of control space
    base::RealVectorBounds cbounds(numberOfJoints);
    for(int i = 0; i < numberOfJoints; i++){
        cbounds.setLow(i,-4);
        cbounds.setHigh(i,4);
    }
    cmanifold->as<control::RealVectorControlSpace>()->setBounds(cbounds);

    // Set up control space
    control::SimpleSetup setup(cmanifold);

    // Set state validity checking for this space
    setup.setStateValidityChecker(boost::bind(&jointManipulatorStateValid, setup.getSpaceInformation().get(), _1));
    // Add ODE solver for the space
    control::ODESolverPtr odeSolver(new control::ODEBasicSolver<> (setup.getSpaceInformation(), &jointManipulatorODE));
    // Add post integration function
    // setup.setPlanner(control::ODESolver::getStatePropagator(odeSolver, &jointManipulatorPostIntegration));
    setup.setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, &jointManipulatorPostIntegration));
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
    numberOfJoints = 3;
    // Initialize car environment
    // carEnvironment = new Environment();

    // Choose type of scenario to plan for
    // int env;
    // do
    // {
    //     std::cout << "Plan for: "<< std::endl;
    //     std::cout << " (1) Pendulum" << std::endl;
    //     std::cout << " (2) Car Robot" << std::endl;

    //     std::cin >> env;
    // } while (env < 1 || env > 2);

    carPlan();

}
