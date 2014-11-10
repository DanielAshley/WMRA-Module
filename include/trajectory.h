/* This function finds the trajectory points along a streight line, given the initial and final transformations. Single-angle rotation about a single axis is used
See Eqs. 1.73-1.103 pages 30-32 of Richard Paul's book " Robot Manipulators"

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% April 2007 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#ifndef _TRAJ_H // if not defined
#define _TRAJ_H // define MyHeader

#include "matrix.h"  
using namespace std;
using namespace math;
#define PI 3.14159265

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

vector<Matrix> WMRA_traj(int ind, Matrix Ti, Matrix Td, int n);
Matrix WMRA_BPolynomial(double qi, double qf, double n);
Matrix WMRA_Linear(double qi, double qf, double n);
Matrix WMRA_Polynomial(double qi, double qf, double n);

#endif