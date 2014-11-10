/* This function gives the Jacobian Matrix and its determinant based on frame 
0 of the new USF WMRA, given the Transformation Matrices of each link.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% April 2007 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#ifndef _J07_H // if not defined
#define _J07_H // define MyHeader
#define PI 3.14159265


#include "matrix.h" 
#include "kinematics.h"
using namespace std;
using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

void WMRA_J07(Matrix T1, Matrix T2, Matrix T3, Matrix T4, Matrix T5, Matrix T6, Matrix T7, Matrix& J0, double& detJ0 );
//void WMRA_Jlimit(Matrix &qmin, Matrix &qmax);
//Matrix WMRA_Opt(Matrix Jo, float detJo, Matrix dq, Matrix dx, Matrix q);

#endif
