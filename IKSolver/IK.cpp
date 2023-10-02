#include "IK.h"
#include "FK.h"
#include "minivectorTemplate.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <cassert>
#if defined(_WIN32) || defined(WIN32)
  #ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
  #endif
#endif
#include <math.h>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

namespace
{

// Converts degrees to radians.
template<typename real>
inline real deg2rad(real deg) { return deg * M_PI / 180.0; }

template<typename real>
Mat3<real> Euler2Rotation(const real angle[3], RotateOrder order)
{
  Mat3<real> RX = Mat3<real>::getElementRotationMatrix(0, deg2rad(angle[0]));
  Mat3<real> RY = Mat3<real>::getElementRotationMatrix(1, deg2rad(angle[1]));
  Mat3<real> RZ = Mat3<real>::getElementRotationMatrix(2, deg2rad(angle[2]));

  switch(order)
  {
    case RotateOrder::XYZ:
      return RZ * RY * RX;
    case RotateOrder::YZX:
      return RX * RZ * RY;
    case RotateOrder::ZXY:
      return RY * RX * RZ;
    case RotateOrder::XZY:
      return RY * RZ * RX;
    case RotateOrder::YXZ:
      return RZ * RX * RY;
    case RotateOrder::ZYX:
      return RX * RY * RZ;
  }
  assert(0);
}

// Performs forward kinematics, using the provided "fk" class.
// This is the function whose Jacobian matrix will be computed using adolc.
// numIKJoints and IKJointIDs specify which joints serve as handles for IK:
//   IKJointIDs is an array of integers of length "numIKJoints"
// Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
// Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
template<typename real>
void forwardKinematicsFunction(
    int numIKJoints, const int * IKJointIDs, const FK & fk,
    const std::vector<real> & eulerAngles, std::vector<real> & handlePositions)
{
  // Students should implement this.
  // The implementation of this function is very similar to function computeLocalAndGlobalTransforms in the FK class.
  // The recommended approach is to first implement FK::computeLocalAndGlobalTransforms.
  // Then, implement the same algorithm into this function. To do so,
  // you can use fk.getJointUpdateOrder(), fk.getJointRestTranslation(), and fk.getJointRotateOrder() functions.
  // Also useful is the multiplyAffineTransform4ds function in minivectorTemplate.h . <- using this
  // It would be in principle possible to unify this "forwardKinematicsFunction" and FK::computeLocalAndGlobalTransforms(),
  // so that code is only written once. We considered this; but it is actually not easily doable.
  // If you find a good approach, feel free to document it in the README file, for extra credit.

    int numJoints = fk.getNumJoints();

    // these couldn't be arrays for some reason and had to be vectors??
    std::vector<Vec3<real>> globalRestTranslation(fk.getNumJoints());
    std::vector<Mat3<real>> globalRestRotation(fk.getNumJoints());
    std::vector<Vec3<real>> localTrans(fk.getNumJoints());
    std::vector<Mat3<real>> localRotat(fk.getNumJoints());

    for (int i = 0; i < fk.getNumJoints(); i++) // compute local before global first, like in computeLocalAndGlobalTransforms
    {
        real eulerRotation[3];
        eulerRotation[0] = eulerAngles[3 * i + 0];
        eulerRotation[1] = eulerAngles[3 * i + 1];
        eulerRotation[2] = eulerAngles[3 * i + 2];

        Mat3<real> eulerR = Euler2Rotation(eulerRotation, fk.getJointRotateOrder(i));

        real jointRotation[3];
        Vec3d jointOrient = fk.getJointOrient(i);
        jointRotation[0] = jointOrient[0];
        jointRotation[1] = jointOrient[1];
        jointRotation[2] = jointOrient[2];

        Mat3<real> jointR = Euler2Rotation(jointRotation, XYZ); // fk.getJointRotateOrder(i) switch to this if XYZ breaks

        localRotat[i] = (jointR * eulerR); // joint * euler, copied from FK.cpp

        // now need the translations
        Vec3d localTranslate = fk.getJointRestTranslation(i);
        Vec3<real> localT;
        localT[0] = localTranslate[0];
        localT[1] = localTranslate[1];
        localT[2] = localTranslate[2];
        localTrans[i] = localT;
    }

    // Then, recursively compute the globalTransforms, from the root to the leaves of the hierarchy.
    for (int i = 0; i < fk.getNumJoints(); i++)
    {

        if (fk.getJointParent(fk.getJointUpdateOrder(i)) >= 0) // just make sure this isnt the root
        {
            Mat3<real> jointOut;
            Vec3<real> transOut;

            multiplyAffineTransform4ds(globalRestRotation[fk.getJointParent(fk.getJointUpdateOrder(i))], globalRestTranslation[fk.getJointParent(fk.getJointUpdateOrder(i))],
                localRotat[fk.getJointUpdateOrder(i)], localTrans[fk.getJointUpdateOrder(i)], globalRestRotation[fk.getJointUpdateOrder(i)], globalRestTranslation[fk.getJointUpdateOrder(i)]);
        }
        else // this is the root, so copy it over
        {
            globalRestRotation[i] = localRotat[fk.getJointUpdateOrder(i)];
            globalRestTranslation[i] = localTrans[fk.getJointUpdateOrder(i)];
        }

    }

    // copy results over

    //   IKJointIDs is an array of integers of length "numIKJoints"
    // Input: numIKJoints, IKJointIDs, fk, eulerAngles (of all joints)
    // Output: handlePositions (world-coordinate positions of all the IK joints; length is 3 * numIKJoints)
    for (int i = 0; i < numIKJoints; i++)
    {
        Vec3<real> globalPos = globalRestTranslation[IKJointIDs[i]];
        handlePositions[3 * i + 0] = globalPos[0];
        handlePositions[3 * i + 1] = globalPos[1];
        handlePositions[3 * i + 2] = globalPos[2];
    }


}

} // end anonymous namespaces

IK::IK(int numIKJoints, const int * IKJointIDs, FK * inputFK, int adolc_tagID)
{
  this->numIKJoints = numIKJoints;
  this->IKJointIDs = IKJointIDs;
  this->fk = inputFK;
  this->adolc_tagID = adolc_tagID;

  FKInputDim = fk->getNumJoints() * 3;
  FKOutputDim = numIKJoints * 3;

  train_adolc();
}

void IK::train_adolc()
{
    // setup of adol_c:
    //   define adol_c inputs and outputs. 
    //   set forwardKinematicsFunction" as the function that will be computed by adol_c.
    //   IK takes the gradient of FK in order to compute the Jacobian matrix

    // first, call trace_on to ask ADOL-C to begin recording how function f is implemented
    trace_on(adolc_tagID);

    // then define input and output dimensions
    vector<adouble> eulerRotations(FKInputDim); // define the input of the function f
    for (int i = 0; i < FKInputDim; i++)
    {
        eulerRotations[i] <<= 0.0; // The <<= syntax tells ADOL-C that these are the input variables.
    }

    vector<adouble> handleJoints(FKOutputDim); // define the output of the function

    // then state what the computation is
    forwardKinematicsFunction(numIKJoints, IKJointIDs, (*fk), eulerRotations, handleJoints);

    vector<double> output(FKOutputDim);
    for (int i = 0; i < FKOutputDim; i++)
    {
        handleJoints[i] >>= output[i]; // Use >>= to tell ADOL-C that y[i] are the output variables
    }

    // Finally, call trace_off to stop recording the function f.
    trace_off(); // ADOL-C tracking finished

}

void IK::doIK(const Vec3d * targetHandlePositions, Vec3d * jointEulerAngles)
{
    // Use adolc to evalute the forwardKinematicsFunction and its gradient (Jacobian). It was trained in train_adolc().
    // IK method used is Tikhonov, jointEulerAngles contains the input Euler angles and is modified to contain the new output

    int numJoints = fk->getNumJoints(); // Note that is NOT the same as numIKJoints!
    int m = FKOutputDim;
    int n = FKInputDim;
    int outputDimen = getFKOutputDim();
    vector <double> outputHandle; // this is the size of outputDimen
    for (int i = 0; i < m; i++)
    {
        outputHandle.push_back(0.0);
    }

    // call FK function, takes the euler angles of the joints
    ::function(adolc_tagID, m, n, (*jointEulerAngles).data(), outputHandle.data());

    // initialize Jacobian matrix
    vector<double> jacobianMatrix; // We store the matrix in row-major order. this is of size m*n
    for (int i = 0; i < (m * n); i++)
    {
        jacobianMatrix.push_back(0.0);
    } 

    vector<double*> jacobianMatrixEachRow; // pointer array where each pointer points to one row of the jacobian matrix
    for (int i = 0; i < m; i++)
    {
        jacobianMatrixEachRow.push_back(&(jacobianMatrix[i * n]));
    }

    ::jacobian(adolc_tagID, m, n, (*jointEulerAngles).data(), jacobianMatrixEachRow.data()); // each row is the gradient of one output component of the function

    // now its time to use eigen for solving

    // the IK equation is: ((jT*J) + alpha*I)(termSolvingFor) = (jT*(changeInPositions))

    Eigen::MatrixXd JMatrix(m, n); // J is the Jacobian matrix of f, size mxn. 

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            JMatrix(i, j) = jacobianMatrix[i * n + j]; // go to right row (i) then col (j)
        }
    }

    Eigen::MatrixXd jT = JMatrix.transpose();

    // calculate change in positions from the target to the current, this is the change in b term from the rhs

    Eigen::VectorXd positionChange(m);

    for (int i = 0; i < numIKJoints; i++)
    {
        positionChange[3 * i + 0] = targetHandlePositions[i][0] - outputHandle.data()[3 * i + 0];
        positionChange[3 * i + 1] = targetHandlePositions[i][1] - outputHandle.data()[3 * i + 1];
        positionChange[3 * i + 2] = targetHandlePositions[i][2] - outputHandle.data()[3 * i + 2];

    }

    // set up the rhs so we can solve for change in theta
    Eigen::VectorXd rhs(m);
    rhs = jT * positionChange;

    double alphaVal = 0.001; // modify this value for either smoother or more jagged IK
    Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(n, n); // nxn matrix

    // set up the lhs of the equation so we can solve for change in theta
    Eigen::MatrixXd lhs(n, n);
    lhs = (jT * JMatrix) + (alphaVal * identityMatrix);

    // solve the system to solve for change in theta (missing term from the lhs) 
    Eigen::VectorXd angleChange = lhs.ldlt().solve(rhs);

    for (int i = 0; i < numJoints; i++) // add in the angle change to the current joint angles
    {
        jointEulerAngles[i][0] += angleChange[3 * i + 0];
        jointEulerAngles[i][1] += angleChange[3 * i + 1];
        jointEulerAngles[i][2] += angleChange[3 * i + 2];
    }
}

