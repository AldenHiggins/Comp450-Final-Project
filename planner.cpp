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

#define DISTANCE_FROM_CENTER 1

using namespace ompl;

int numberOfJoints;


/*
   Recursive definition of determinate using expansion by minors.
*/
double Determinant(double **a,int n)
{
   int i,j,j1,j2;
   double det = 0;
   double **m = NULL;

   if (n < 1) { /* Error */

   } else if (n == 1) { /* Shouldn't get used */
      det = a[0][0];
   } else if (n == 2) {
      det = a[0][0] * a[1][1] - a[1][0] * a[0][1];
   } else {
      det = 0;
      for (j1=0;j1<n;j1++) {
         m = (double **) malloc((n-1)*sizeof(double *));
         for (i=0;i<n-1;i++)
            m[i] = (double *) malloc((n-1)*sizeof(double));
         for (i=1;i<n;i++) {
            j2 = 0;
            for (j=0;j<n;j++) {
               if (j == j1)
                  continue;
               m[i-1][j2] = a[i][j];
               j2++;
            }
         }
         det += pow(-1.0,j1+2.0) * a[0][j1] * Determinant(m,n-1);
         for (i=0;i<n-1;i++)
            free(m[i]);
         free(m);
      }
   }
   return(det);
}

/*
   Find the cofactor matrix of a square matrix
*/
void CoFactor(double **a,int n,double **b)
{
   int i,j,ii,jj,i1,j1;
   double det;
   double **c;

   c = (double **) malloc((n-1)*sizeof(double *));
   for (i=0;i<n-1;i++)
     c[i] = (double *) malloc((n-1)*sizeof(double));

   for (j=0;j<n;j++) {
      for (i=0;i<n;i++) {

         /* Form the adjoint a_ij */
         i1 = 0;
         for (ii=0;ii<n;ii++) {
            if (ii == i)
               continue;
            j1 = 0;
            for (jj=0;jj<n;jj++) {
               if (jj == j)
                  continue;
               c[i1][j1] = a[ii][jj];
               j1++;
            }
            i1++;
         }

         /* Calculate the determinate */
         det = Determinant(c,n-1);

         /* Fill in the elements of the cofactor */
         b[i][j] = pow(-1.0,i+j+2.0) * det;
      }
   }
   for (i=0;i<n-1;i++)
      free(c[i]);
   free(c);
}

/*
   Transpose of a square matrix, do it in place
*/
void Transpose(double **a,int n)
{
  int i,j;
  double tmp;

  for (i=1;i<n;i++) {
    for (j=0;j<i;j++) {
       tmp = a[i][j];
       a[i][j] = a[j][i];
       a[j][i] = tmp;
    }
  }

}
/*
  Invert the inputted matrix into B
*/
void Inverse(double **a, double **b, int n)
{

  double det = Determinant(a,n);
  CoFactor(a,n,b);
  for(int i = 0; i < n; i++){
    for(int j = 0; j < n; j++){
      b[i][j] = b[i][j]/det;
    }
  }

  Transpose(b,n);

}
// Convert a vector of vectors into a double **
double** setupHMM(std::vector<std::vector<double> > &vals, int N, int M)
{
   double** temp;
   temp = new double*[N];
   for(unsigned i=0; (i < N); i++)
   { 
      temp[i] = new double[M];
      for(unsigned j=0; (j < M); j++)
      {
          temp[i][j] = vals[i][j];
      } 
   }

   return temp;
}

void initializeNByNVector(std::vector<std::vector<double> > *vecToInit, int n){
  for (int i = 0; i < n; i++){
        std::vector<double> row;
        for (int j = 0; j < n; j++){
            row.push_back(0);
        }
        vecToInit->push_back(row);
    }
}

// State checker for the car robot
bool jointManipulatorStateValid(const control::SpaceInformation *si, const base::State *state){
    return true;
}

// Definition of the ODE for the car
void jointManipulatorODE(const control::ODESolver::StateType& q, const control::Control* control, control::ODESolver::StateType& qdot){
    const double *u = control->as<control::RealVectorControlSpace::ControlType>()->values;
    const double q0 = q[0];
    const double q1 = q[1];
    const double q2 = q[2];
    const double q3 = q[3];
    const double q4 = q[4];
    const double q5 = q[5];

    /** Convert the theta values for the joints into x,y pairs for the ends **/
    std::vector<double> jointEndsX;
    std::vector<double> jointEndsY;
    // Theta accumulator
    std::vector<double> thetaArr;
    // Initialize the first element
    jointEndsX.push_back(JOINT_LENGTH * cos(q[0]));
    jointEndsY.push_back(JOINT_LENGTH * sin(q[0]));

    double qSum = q[0];
    thetaArr.push_back(qSum);
    for (int i = 1; i < numberOfJoints; i++) {
        qSum += q[i * 2];
        thetaArr.push_back(qSum);
        jointEndsX.push_back(jointEndsX[i-1] + JOINT_LENGTH * cos(qSum));
        jointEndsY.push_back(jointEndsY[i-1] + JOINT_LENGTH * sin(qSum));
    }

    
    /** Build the inertia matrix **/
    std::vector<std::vector<double> > inertiaMatrix;
    initializeNByNVector(&inertiaMatrix, numberOfJoints);

    qSum = q[0];
    std::vector<std::vector<double> > jacobianLXArr;
    std::vector<std::vector<double> > jacobianLYArr;
    // Sum up all the Jacobians for each i into the inertia matrix
    for (int i = 0; i < numberOfJoints; i++) {
        qSum += q[i * 2];
        std::vector<double> jacobianLX;
        std::vector<double> jacobianLY;
        // Generate the Jacobian
        for(int j = 0; j < numberOfJoints; j++) {
            if (j <= i ) {
                jacobianLX.push_back(-1 * jointEndsY[j] + jointEndsY[i] - JOINT_LENGTH/2 * sin(qSum));
                jacobianLY.push_back(jointEndsX[j] + -1 * jointEndsX[i] + JOINT_LENGTH/2 * cos(qSum));
            }
            else {
                jacobianLX.push_back(0);
                jacobianLY.push_back(0);
            }
        }
        jacobianLXArr.push_back(jacobianLX);
        jacobianLYArr.push_back(jacobianLY);

        // Multiply the Jacobian by its transpose and add in the moment of inertia if r/c < i then
        // throw those values into the accumulating inertiaMatrix
        for (int r = 0; r < numberOfJoints; r++) {
            for (int c = 0; c < numberOfJoints; c++) {
                inertiaMatrix[r][c] = JOINT_MASS * JOINT_MASS * (jacobianLX[r] * jacobianLX[c] + jacobianLY[r] * jacobianLY[c]);    
                if (r <= i && c <= i) {
                    // Add in moment of inertia
                    inertiaMatrix[r][c] += ((JOINT_MASS * JOINT_LENGTH * JOINT_LENGTH)/12.);
                }            
            }
        }
    }
    // Invert the inertia matrix
    double** Hinv = (double**)malloc(numberOfJoints * sizeof (double*));
    for (int z = 0; z < numberOfJoints; z++) {
      Hinv[z] = (double*) malloc(numberOfJoints * sizeof(double));
    }
    Inverse(setupHMM(inertiaMatrix,numberOfJoints,numberOfJoints), Hinv, numberOfJoints);


    /** Generate the partial derivatives of the Inertia Matrix for the Coriolis vector calculation  **/
    // Will contain the partial derivatives
    std::vector<std::vector<std::vector< double > > > vectorOfMatricies;

    for (int i = 0; i < numberOfJoints; i++) {
      // The matrix for this partial derivative
      std::vector<std::vector<double> > resultMatrix;
      initializeNByNVector(&resultMatrix, numberOfJoints);

      for (int j = 0; j < numberOfJoints; j++) {
        // Generate the partial jacobian derivative for this joint
        std::vector<double> partialJX;
        std::vector<double> partialJY;
        double thetaSum = q[0];
        for (int k = 0; k < numberOfJoints; k++) {
            int r = std::max(j, k);
            if (j <=i && k <= i) {
              partialJX.push_back(-1 * jointEndsX[r] + jointEndsX[i] - DISTANCE_FROM_CENTER * cos(thetaArr[i]));
              partialJY.push_back(-1 * jointEndsY[r] + jointEndsY[i] - DISTANCE_FROM_CENTER * sin(thetaArr[i]));
            }
            else {
              partialJX.push_back(0);
              partialJY.push_back(0);
            }
        }
        // Multiply the partial derivative by the jacobian transpose
        // TODO: I thought we multiplied by the transpose and not by the derivative transpose....
        std::vector<std::vector<double> > multipliedMatrix;
        initializeNByNVector(&multipliedMatrix, numberOfJoints);
        for (int r = 0; r < numberOfJoints; r++) {
            for (int c = 0; c < numberOfJoints; c++) {
                multipliedMatrix[r][c] = JOINT_MASS * (partialJX[r] * partialJX[c] + partialJY[r] * partialJY[c]);
            }
        }
        // Generate the transpose of the result and add it in, then increment the result matrix
        double **matrixTranspose = setupHMM(multipliedMatrix, numberOfJoints, numberOfJoints);
        Transpose(matrixTranspose, numberOfJoints);
        for (int x = 0; x < numberOfJoints; x++) {
          for (int y = 0; y < numberOfJoints; y++) {
            multipliedMatrix[x][y] += matrixTranspose[x][y];
            resultMatrix[x][y] += multipliedMatrix[x][y];
          }
        }
      }

      vectorOfMatricies.push_back(resultMatrix);
    }
    std::vector<double> littleH;
    for (int i = 0; i < numberOfJoints; i++) {
      double littleHI = 0;
      for (int j = 0; j < numberOfJoints; j++) {
        for (int k = 0; k < numberOfJoints; k++) {
          double hijk = vectorOfMatricies[k][i][j] - 0.5  * vectorOfMatricies[i][j][k];
          double qDotj = q[j * 2 + 1];
          double qDoti = q[i * 2 + 1];
          littleHI += hijk * qDotj * qDoti;
        }
      }
      littleH.push_back(littleHI);
    }
}


// This is a callback method invoked after numerical integration for the car robot
void jointManipulatorPostIntegration(const base::State* /*state*/, const control::Control* /*control*/, const double /*duration*/, base::State *result){
    // Normalize orientation between 0 and 2*pi
    base::SO2StateSpace SO2;
    for (int i = 0; i < numberOfJoints; i++){
        SO2.enforceBounds (result->as<base::CompoundStateSpace::StateType>()->as<base::SO2StateSpace::StateType>(i * 2));
    }
}

void carPlan(){
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

    
    std::vector<std::vector<double> > a;
    std::vector<std::vector<double> > b;
    
    int n = 3;


    std::vector<double> row1;
    std::vector<double> row2;
    std::vector<double> row3;

    row1.push_back(1);
    row1.push_back(2);
    row1.push_back(0);
    row2.push_back(-1);
    row2.push_back(1);
    row2.push_back(1);
    row3.push_back(1);
    row3.push_back(2);
    row3.push_back(3);

    a.push_back(row1);
    a.push_back(row2);
    a.push_back(row3);


    double ** c = setupHMM(a,n,n);
    double ** d = setupHMM(a,n,n);


    Inverse(c,d,n);

    std::cout << d[0][0] << d[0][1];

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

int main(){
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
