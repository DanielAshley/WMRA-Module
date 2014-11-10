/* This function gives the Jacobian Matrix and its determinant based on frame 
0 of the new USF WMRA, given the Transformation Matrices of each link.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%Modified By:Ana Catalina Torres%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% April 2010 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#include "matrix.h" 
#include "jacobian.h"

#include <limits>

using namespace std;
using namespace math;

void WMRA_J07(Matrix T1, Matrix T2, Matrix T3, Matrix T4, Matrix T5, Matrix T6, Matrix T7, Matrix& J0, double& detJ0 ){
	Matrix T(4,4), J0temp(6,6), J0trans(7,6);
	T.Unit(4);
	J0.SetSize(6,7);
	//	cout << "\nT is:\n" << T << endl;

	J0(0,6) = -T(0,0)*T(1,3)+T(1,0)*T(0,3);
	J0(1,6) = -T(0,1)*T(1,3)+T(1,1)*T(0,3);
	J0(2,6) = -T(0,2)*T(1,3)+T(1,2)*T(0,3);
	J0(3,6) = T(2,0);
	J0(4,6) = T(2,1);
	J0(5,6) = T(2,2);

	T = T7 * T;
	//	cout << "\nT is:\n" << T << endl;

	J0(0,5) = -T(0,0)*T(1,3)+T(1,0)*T(0,3);
	J0(1,5) = -T(0,1)*T(1,3)+T(1,1)*T(0,3);
	J0(2,5) = -T(0,2)*T(1,3)+T(1,2)*T(0,3);
	J0(3,5) = T(2,0);
	J0(4,5) = T(2,1);
	J0(5,5) = T(2,2);

	T = T6 * T;
	//	cout << "\nT is:\n" << T << endl;

	J0(0,4) = -T(0,0)*T(1,3)+T(1,0)*T(0,3);
	J0(1,4) = -T(0,1)*T(1,3)+T(1,1)*T(0,3);
	J0(2,4) = -T(0,2)*T(1,3)+T(1,2)*T(0,3);
	J0(3,4) = T(2,0);
	J0(4,4) = T(2,1);
	J0(5,4) = T(2,2);

	T = T5 * T;
	//	cout << "\nT is:\n" << T << endl;

	J0(0,3) = -T(0,0)*T(1,3)+T(1,0)*T(0,3);
	J0(1,3) = -T(0,1)*T(1,3)+T(1,1)*T(0,3);
	J0(2,3) = -T(0,2)*T(1,3)+T(1,2)*T(0,3);
	J0(3,3) = T(2,0);
	J0(4,3) = T(2,1);
	J0(5,3) = T(2,2);

	T = T4 * T;
	//	cout << "\nT is:\n" << T << endl;

	J0(0,2) = -T(0,0)*T(1,3)+T(1,0)*T(0,3);
	J0(1,2) = -T(0,1)*T(1,3)+T(1,1)*T(0,3);
	J0(2,2) = -T(0,2)*T(1,3)+T(1,2)*T(0,3);
	J0(3,2) = T(2,0);
	J0(4,2) = T(2,1);
	J0(5,2) = T(2,2);

	T = T3 * T;
	//	cout << "\nT is:\n" << T << endl;

	J0(0,1) = -T(0,0)*T(1,3)+T(1,0)*T(0,3);
	J0(1,1) = -T(0,1)*T(1,3)+T(1,1)*T(0,3);
	J0(2,1) = -T(0,2)*T(1,3)+T(1,2)*T(0,3);
	J0(3,1) = T(2,0);
	J0(4,1) = T(2,1);
	J0(5,1) = T(2,2);

	T = T2 * T;
	//	cout << "\nT is:\n" << T << endl;

	J0(0,0) = -T(0,0)*T(1,3)+T(1,0)*T(0,3);
	J0(1,0) = -T(0,1)*T(1,3)+T(1,1)*T(0,3);
	J0(2,0) = -T(0,2)*T(1,3)+T(1,2)*T(0,3);
	J0(3,0) = T(2,0);
	J0(4,0) = T(2,1);
	J0(5,0) = T(2,2);

	T = T1 * T;
	//	cout << "\nT is:\n" << T << endl;

	J0temp.Null(6,6);
	int i, j; 

	for ( i=0 ; i < 3 ; i++ ) {
		for ( j = 0 ; j < 3 ; j++ ) {
			J0temp(i,j)=T(i,j);
		}
	}
	for ( i=3 ; i < 6 ; i++ ) {
		for ( j = 3 ; j < 6 ; j++ ) {
			J0temp(i,j)=T(i-3,j-3);
		}
	}

	J0 = J0temp * J0;
	J0trans = ~J0;
	J0temp = J0 * J0trans;
	detJ0= sqrt(J0temp.Det());
}
