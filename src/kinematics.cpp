/* This function gives the Transformation Matrix of the new USF WMRA with 7 DOF, given the joint angles in Radians.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Modified By:Daniel Ashley %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% January 2013%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#include "matrix.h"  
#include "kinematics.h"
#include <vector>
#include <time.h>
#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace math;

//forward kinematics
Matrix kinematics(vector<double> q){

	Matrix T(4,4);

	// Inputting the D-H Parameters in a Matrix form:
	Matrix DH(1,1);
	DH=WMRA_DH(q);	
	// Calculating the transformation matrices of each link:
	Matrix T1(4,4),T2(4,4),T3(4,4),T4(4,4),T5(4,4),T6(4,4),T7(4,4),Tttemp(4,4);
	//T1
	T1 = WMRA_rotx(DH(0,0))*WMRA_transl(DH(0,1),0,0)*WMRA_rotz(DH(0,3))*WMRA_transl(0,0,DH(0,2));
	//T2
	T2 = WMRA_rotx(DH(1,0))*WMRA_transl(DH(1,1),0,0)*WMRA_rotz(DH(1,3))*WMRA_transl(0,0,DH(1,2));
	//T3
	T3 = WMRA_rotx(DH(2,0))*WMRA_transl(DH(2,1),0,0)*WMRA_rotz(DH(2,3))*WMRA_transl(0,0,DH(2,2));
	//T4
	T4 = WMRA_rotx(DH(3,0))*WMRA_transl(DH(3,1),0,0)*WMRA_rotz(DH(3,3))*WMRA_transl(0,0,DH(3,2));
	//T5
	T5 = WMRA_rotx(DH(4,0))*WMRA_transl(DH(4,1),0,0)*WMRA_rotz(DH(4,3))*WMRA_transl(0,0,DH(4,2));
	//T6
	T6 = WMRA_rotx(DH(5,0))*WMRA_transl(DH(5,1),0,0)*WMRA_rotz(DH(5,3))*WMRA_transl(0,0,DH(5,2));
	//T7
	T7 = WMRA_rotx(DH(6,0))*WMRA_transl(DH(6,1),0,0)*WMRA_rotz(DH(6,3))*WMRA_transl(0,0,DH(6,2));

	//Calculating the total Transformation Matrix of the given arm position:
	T=T1*T2*T3*T4*T5*T6*T7;

	return T;
}



/*****************************************************
//q = current joint positions, size=8 or 10 depending is Wheelchair is included
//dq = Wheelchair position, size=2
//Twc = Transformation Matrix of the Wheelchair
//Ta = current joint position, ARM ONLY
//Twco = Updated Transformation Matrix of the Wheelchair
//T1-T7 = Thransform Matrix of each of the joints
*****************************************************/
Matrix kinematics(vector<double> q, Matrix dq, Matrix Twc, Matrix& Ta, Matrix& Twco, Matrix& T1, Matrix& T2, Matrix& T3, Matrix& T4, Matrix& T5, Matrix& T6, Matrix& T7){

	// Inputting the D-H Parameters in a Matrix form:
	Matrix DH(1,1), T(4,4);
	DH=WMRA_DH(q);

	//Calculating the transformation matrices of each link:
	//T1
	T1 = WMRA_rotx(DH(0,0))*WMRA_transl(DH(0,1),0,0)*WMRA_rotz(DH(0,3))*WMRA_transl(0,0,DH(0,2));
	//T2
	T2 = WMRA_rotx(DH(1,0))*WMRA_transl(DH(1,1),0,0)*WMRA_rotz(DH(1,3))*WMRA_transl(0,0,DH(1,2));
	//T3
	T3 = WMRA_rotx(DH(2,0))*WMRA_transl(DH(2,1),0,0)*WMRA_rotz(DH(2,3))*WMRA_transl(0,0,DH(2,2));
	//T4
	T4 = WMRA_rotx(DH(3,0))*WMRA_transl(DH(3,1),0,0)*WMRA_rotz(DH(3,3))*WMRA_transl(0,0,DH(3,2));
	//T5
	T5 = WMRA_rotx(DH(4,0))*WMRA_transl(DH(4,1),0,0)*WMRA_rotz(DH(4,3))*WMRA_transl(0,0,DH(4,2));
	//T6
	T6 = WMRA_rotx(DH(5,0))*WMRA_transl(DH(5,1),0,0)*WMRA_rotz(DH(5,3))*WMRA_transl(0,0,DH(5,2));
	//T7
	T7 = WMRA_rotx(DH(6,0))*WMRA_transl(DH(6,1),0,0)*WMRA_rotz(DH(6,3))*WMRA_transl(0,0,DH(6,2));

	//Calculating the Transformation Matrix of the initial arm position:
	Ta=T1*T2*T3*T4*T5*T6*T7;
	Twc = WMRA_w2T(1, Twc, dq);
	Twco=Twc;
	T=Twc*Ta;
	return T;
}

Matrix kinematics(vector<double> q, Matrix& Ta, Matrix& T1, Matrix& T2, Matrix& T3, Matrix& T4, Matrix& T5, Matrix& T6, Matrix& T7){

	// Inputting the D-H Parameters in a Matrix form:
	Matrix DH(7,4), T(4,4);
	DH=WMRA_DH(q);

	//Calculating the transformation matrices of each link:
	//T1
	T1 = WMRA_rotx(DH(0,0))*WMRA_transl(DH(0,1),0,0)*WMRA_rotz(DH(0,3))*WMRA_transl(0,0,DH(0,2));
	//T2
	T2 = WMRA_rotx(DH(1,0))*WMRA_transl(DH(1,1),0,0)*WMRA_rotz(DH(1,3))*WMRA_transl(0,0,DH(1,2));
	//T3
	T3 = WMRA_rotx(DH(2,0))*WMRA_transl(DH(2,1),0,0)*WMRA_rotz(DH(2,3))*WMRA_transl(0,0,DH(2,2));
	//T4
	T4 = WMRA_rotx(DH(3,0))*WMRA_transl(DH(3,1),0,0)*WMRA_rotz(DH(3,3))*WMRA_transl(0,0,DH(3,2));
	//T5
	T5 = WMRA_rotx(DH(4,0))*WMRA_transl(DH(4,1),0,0)*WMRA_rotz(DH(4,3))*WMRA_transl(0,0,DH(4,2));
	//T6
	T6 = WMRA_rotx(DH(5,0))*WMRA_transl(DH(5,1),0,0)*WMRA_rotz(DH(5,3))*WMRA_transl(0,0,DH(5,2));
	//T7
	T7 = WMRA_rotx(DH(6,0))*WMRA_transl(DH(6,1),0,0)*WMRA_rotz(DH(6,3))*WMRA_transl(0,0,DH(6,2));

	//Calculating the Transformation Matrix of the initial arm position:
	Ta=T1*T2*T3*T4*T5*T6*T7;
	//Twc = WMRA_w2T(1, Twc, dq);
	//Twco=Twc;
	T=Ta;
	return T;
}


Matrix WMRA_DH(vector<double> q){

	Matrix DH1(7,4);

	//Inputting the D-H Parameters in a Matrix form, dimensions are in millimeters and radians:

	// Dimentions based on the actual physical arm:
	float DHtemp[7][4]={
		{-M_PI/2, 0,	120.4 /*102*/,	 q[0]},
		{ M_PI/2, 0,	134.25 /*133*/,	 q[1]},  
		{-M_PI/2, 0,  503.345 /*502*/,	 q[2]},
		{ M_PI/2, 0,  133.2 /*135*/,		 q[3]},
		{-M_PI/2, 0,  386.658 /*375+24*/,	 q[4]},
		{ M_PI/2, 0,  -23.5,		 q[5]},
		{-M_PI/2, 0,  172.5+205 /*360*//*161+70 *//*143*/, q[6]}};

		// Dimentions based on the Virtual Reality arm model:
		/*  double DH[7][4]={{-M_PI/2, 0, 109.72, q(0,0)},
		{ M_PI/2, 0, 118.66, q(1,0)},  
		{-M_PI/2, 0, 499.67, q(2,0)},
		{ M_PI/2, 0, 121.78, q(3,0)},
		{-M_PI/2, 0, 235.67, q(4,0)},
		{ M_PI/2, 0,   0,    q(5,0)},
		{-M_PI/2, 0, 276.68, q(6,0)}};
		*/
		int i,j;
		for (j=0; j < 4; j++){
			for (i=0; i < 7; i++){
				DH1(i,j) = DHtemp[i][j];				
			}
		}
		return DH1;
}

// This function gives the homogeneous transformation matrix, given the rotation angle about the X axis.
Matrix WMRA_rotx(double t){

	Matrix T(4,4);

	double c, s;
	c=cos(t);
	s=sin(t);
	T.Unit(4);
	T(1,1)= c;
	T(1,2)= -s;
	T(2,1)= s;
	T(2,2)= c;

	return T;
}

// This function gives the homogeneous transformation matrix, given the rotation angle about the Z axis.
Matrix WMRA_rotz(double t){

	Matrix T(4,4);

	double c, s;
	c=cos(t);
	s=sin(t);
	T.Unit(4);
	T(0,0)= c;
	T(0,1)= -s;
	T(1,0)= s;
	T(1,1)= c;

	return T;
}

// This function gives the homogeneous transformation matrix, given the X, Y, Z cartesian translation values.
Matrix WMRA_transl(double x, double y, double z){

	Matrix T(4,4);

	T.Unit(4);
	T(0,3)= x;
	T(1,3)= y;
	T(2,3)= z;

	return T;
}

// This function gives the homogeneous transformation matrix, given the rotation angle about the Y axis.
Matrix WMRA_roty(double t){

	Matrix T(4,4);

	double c, s;
	c=cos(t);
	s=sin(t);
	T.Unit(4);
	T(0,0)= c;
	T(0,2)= s;
	T(2,0)= -s;
	T(2,2)= c;

	return T;
}

Matrix WMRA_w2T(int ind, Matrix Tp, Matrix q){

	Matrix T(4,4);
	Matrix L(1,1);
	// Reading the Wheelchair's constant dimentions, all dimentions are converted in millimeters:
	L=WMRA_WCD();
	// Deciding if the motion is in reference to the arm base (1) or the wheel axle center (0):
	int i;	
	if (ind==0){
		for (i=1; i<4; i++){
			L(0,i) = 0;
		}
	}

	// Defining the inverse of Transformation Matrix between the wheelchair center and the WMRA's base:
	Matrix Twa(4,4), Twainv(4,4);
	Twa.Unit(4);
	for (i=0; i<3; i++){
		Twa(i,3) = L(0,i+1);
	}

	// The previous transformation matrix from the ground to the wheelchair center:
	Twainv = !Twa;
	Tp = Tp * Twainv;

	// Defining the Transformation Matrix between the ground and the wheelchair center and WMRA's base:
	if (abs(q(1,0))<=EPS){          // Streight line motion.
		for (i=0; i<2; i++){
			Tp(i,3)= Tp(i,3) + q(0,0)*Tp(i,0);
		}
		T = Tp * Twa;
	}
	else {
		double po, p, r;
		po=atan2(Tp(1,0),Tp(0,0));
		p=q(1,0);
		r=q(0,0)/p-L(0,0)/2;
		Matrix Tgw(4,4);
		Tgw.Unit(4);
		Tgw(0,0)= cos(po+p);
		Tgw(0,1)= -sin(po+p);
		Tgw(0,3)= Tp(0,3)+sin(M_PI/2+po+p/2)*(r+L(0,0)/2)*sin(p)/cos(p/2);
		Tgw(1,0)= sin(po+p);
		Tgw(1,1)= cos(po+p);
		Tgw(1,3)= Tp(1,3)-cos(M_PI/2+po+p/2)*(r+L(0,0)/2)*sin(p)/cos(p/2);
		Tgw(2,3)= Tp(2,3);

		T= Tgw * Twa;
	}

	return T;
}

Matrix WMRA_WCD(){

	Matrix L(1,5);
	int Ltemp[5] = {500,400,230,200,170};
	int i;
	for (i=0; i < 5; i++){
		L(0,i) = Ltemp[i];
	}

	// All dimentions are in millimeters.
	//L(0,0)=560;  // Distance between the two driving wheels.
	//L(0,1)=440;  // Horizontal distance between the wheels axix of rotation and the arm mounting position (along x).
	//L(0,2)=230;  // Horizontal distance between the middle point between the two driving wheels and the arm mounting position (along y).
	//L(0,3)=182;  // Vertical distance between the wheels axix of rotation and the arm mounting position (along z).
	//L(0,4)=168;  // Radius of the driving wheels.

	return L;
}

Matrix WMRA_p2T(double x, double y, double a){

	Matrix T(4,4);

	//Reading the Wheelchair's constant dimentions, all dimentions are converted in millimeters:
	Matrix L(1,5);
	L=WMRA_WCD();
	//Defining the Transformation Matrix:

	T.Unit(4);	
	T(0,0)= cos(a);
	T(0,1)= -sin(a);
	T(0,3)= x; 
	T(1,0)= sin(a);
	T(1,1)= cos(a);
	T(1,3)= y;
	T(2,3)= L(0,3)+L(0,4);

	return T;
}

//Matrix rotationMatrix(double pitch, double roll, double yaw)
//{
//	Matrix temp_rotation(4,4);
//	temp_rotation = WMRA_rotz(yaw)*WMRA_roty(pitch)*WMRA_rotx(roll);
//	return temp_rotation;
//}

Matrix pose2TfMat(WMRA::Pose dest){
	Matrix temp(4,4);
	temp.Unit(4);
	temp = WMRA_rotz(dest.yaw)*WMRA_roty(dest.pitch)*WMRA_rotx(dest.roll);
   //temp = WMRA_rotx(dest.roll)  *WMRA_roty(dest.pitch)* WMRA_rotz(dest.yaw);
	temp(0,3) = dest.x;
	temp(1,3) = dest.y;
	temp(2,3) = dest.z;
	return temp;
}

WMRA::Pose TransfomationToPose(Matrix T){
   double x,y,z;
   if(T(2,0) != 1 || T(2,0) != -1 ){
      y = -asin(T(2,0));
      //y = atan2( -1*T(2,0), sqrt( pow(T(0,0),2) + pow(T(1,0),2) ));
      x = atan2(T(2,1)/cos(y), T(2,2)/cos(y));
      z = atan2(T(1,0)/cos(y), T(0,0)/cos(y));
   }
   else{
      z = 0;
      if(T(2,0) == 1 ){
         y = -M_PI/2;
         x= z + atan2(-1* T(0,1), -1*T(0,2));
      }
      else{ //T(2,0) == -1 
         y = M_PI/2;
         x= z + atan2(T(0,1), T(0,2));
      }
   }

   WMRA::Pose pose;
   pose.pitch = y;
   pose.roll = x;
   pose.yaw = z;
   pose.x = T(0,3);
   pose.y = T(1,3);
   pose.z = T(2,3);
   return pose;

}