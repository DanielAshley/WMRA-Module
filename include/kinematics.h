/* This function gives the Transformation Matrix of the new USF WMRA with 7 DOF, given the joint angles in Radians.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% April 2007 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#ifndef WMRA_KINEMATICS_H // if not defined
#define WMRA_KINEMATICS_H // define MyHeader


#include <vector>
#include <time.h>
#include "matrix.h" 
#include "WmraTypes.h"

#define EPS 2.2204460492503131e-016 

using namespace std;
using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

Matrix WMRA_DH(vector<double> q);
Matrix kinematics(vector<double> q);
Matrix kinematics(vector<double> q, Matrix dq, Matrix Twc, Matrix& Ta, Matrix& Twco, Matrix& T1, Matrix& T2, Matrix& T3, Matrix& T4, Matrix& T5, Matrix& T6, Matrix& T7);
Matrix kinematics(vector<double> q, Matrix& Ta, Matrix& T1, Matrix& T2, Matrix& T3, Matrix& T4, Matrix& T5, Matrix& T6, Matrix& T7);
Matrix WMRA_rotx(double t);
Matrix WMRA_rotz(double t);
Matrix WMRA_transl(double x, double y, double z);
Matrix WMRA_roty(double t);
Matrix WMRA_w2T(int ind, Matrix Tp, Matrix q);
Matrix WMRA_WCD();
Matrix WMRA_p2T(double x, double y, double a);
Matrix rotationMatrix(double pitch, double roll, double yaw);
Matrix pose2TfMat(WMRA::Pose dest);
WMRA::Pose TransfomationToPose(Matrix T);

#endif
