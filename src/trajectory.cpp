/* This function finds the trajectory points along a streight line, given the initial and final transformations. Single-angle rotation about a single axis is used
See Eqs. 1.73-1.103 pages 30-32 of Richard Paul's book " Robot Manipulators"

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Modified By:Ana Catalina Torres %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% April 2010%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#include "matrix.h"  
#include "Utility.h"
#include "trajectory.h"

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>

using namespace std;
using namespace math;

vector<Matrix> WMRA_traj(int ind, Matrix Ti, Matrix Td, int numWayPoints){

	double ***Tt;
   int n = numWayPoints;
	//Finding the rotation of the desired point based on the initial point:
	Matrix R(3,3), Titemp(3,3),Tdtemp(3,3);
	int i,j,m;
   //deep copy Matrix
	for ( int i=0 ; i < 3 ; i++ ) {
		for ( int j = 0 ; j < 3 ; j++ ) {
			Titemp(i,j)=Ti(i,j);
			Tdtemp(i,j)=Td(i,j);
		}
	}
   Matrix Titemp2(3,3); //#debug clean this up later
   Titemp2 = ~Titemp;
	Titemp=Titemp2;
	R=Titemp*Tdtemp;
	
	//Initial single-angle representation of the rotation:
	double a, s, c, v;
	a=atan2(sqrt(pow((R(2,1)-R(1,2)),2)+pow((R(0,2)-R(2,0)),2)+pow((R(1,0)-R(0,1)),2)),(R(0,0)+R(1,1)+R(2,2)-1));
	s=sin(a);
	c=cos(a);
	v=1-c;
	
	//Finding the single-vector components for the rotation:
	double kx, ky, kz;
	if (a<0.001){
		kx=1;
		ky=0;
		kz=0;
	}
	else if (a < (M_PI/2) + 0.001){  //#debug is there a bug when a = PI/2?
		kx=(R(2,1)-R(1,2))/(2*s);
		ky=(R(0,2)-R(2,0))/(2*s);
		kz=(R(1,0)-R(0,1))/(2*s);
	} 
	else {
		kx=sign((R(2,1)-R(1,2)))*sqrt((R(0,0)-c)/v);
		ky=sign((R(0,2)-R(2,0)))*sqrt((R(1,1)-c)/v);
		kz=sign((R(1,0)-R(0,1)))*sqrt((R(2,2)-c)/v);
		if (kx>ky && kx>kz){
			ky=(R(1,0)+R(0,1))/(2*kx*v);
			kz=(R(0,2)+R(2,0))/(2*kx*v);
		}
		else if (ky>kx && ky>kz){
			kx=(R(1,0)+R(0,1))/(2*ky*v);
			kz=(R(2,1)+R(1,2))/(2*ky*v);
		}
		else {
			kx=(R(0,2)+R(2,0))/(2*kz*v);
			ky=(R(2,1)+R(1,2))/(2*kz*v);
		}
	}
	
	// Running the desired trajectory method: 
	// 1 == Polynomial with Blending function,
	// 2 == Polynomial without Blending function,
	// 3 == Linear function.
	Matrix at(n,1), xt(n,1), yt(n,1), zt(n,1);
	Titemp=~Titemp;
	if (ind == 2){
		at=WMRA_Polynomial(0,a,n);
		xt=WMRA_Polynomial(Ti(0,3), Td(0,3), n); // 
		yt=WMRA_Polynomial(Ti(1,3), Td(1,3), n);
		zt=WMRA_Polynomial(Ti(2,3), Td(2,3), n);
	}
	else if (ind == 3) {
		at=WMRA_Linear(0,a,n);
		xt=WMRA_Linear(Ti(0,3), Td(0,3), n);
		yt=WMRA_Linear(Ti(1,3), Td(1,3), n);
		zt=WMRA_Linear(Ti(2,3), Td(2,3), n);
	}
	else {
		at=WMRA_BPolynomial(0,a,n);
		xt=WMRA_BPolynomial(Ti(0,3), Td(0,3), n);
		yt=WMRA_BPolynomial(Ti(1,3), Td(1,3), n);
		zt=WMRA_BPolynomial(Ti(2,3), Td(2,3), n);
	}
	
    /* create a 3D nx4x4 double array */

   vector<Matrix> wayPoints;
   wayPoints.resize(numWayPoints);

	Tt = new double**[n];
	for (i = 0; i < n; ++i) {
		Tt[i] = new double*[4];
		for (j = 0; j < 4; ++j){
			Tt[i][j] = new double[4];
		}
	}
   /* set the first Matrix to Ti*/
   wayPoints[0] = Ti;
	for ( i=0 ; i < 4 ; i++ ) {
		for ( j = 0 ; j < 4 ; j++ ) {
			Tt[0][i][j]=Ti(i,j);
         wayPoints[0](i,j) = Ti(i,j);
		}
	}
   

	for (int i = 1; i < numWayPoints ; i++){
		// Single-angle Change:
		double da;
		da=at(i,0)-at(0,0);
		s=sin(da);
		c=cos(da);
		v=1-c;
		// Rotation and Position Change:
		
		Matrix dR(3,3);
		dR(0,0)=pow(kx,2)*v+c;
		dR(0,1)=kx*ky*v-kz*s;
		dR(0,2)=kx*kz*v+ky*s;
		dR(1,0)=kx*ky*v+kz*s;
		dR(1,1)=pow(ky,2)*v+c;
		dR(1,2)=ky*kz*v-kx*s;
		dR(2,0)=kx*kz*v-ky*s;
		dR(2,1)=ky*kz*v+kx*s;
		dR(2,2)=pow(kz,2)*v+c;
		
		//Finding the trajectory points along the trajectory line:
		Matrix Tti1(3,3),Tti(4,4);
		Tti1 = Titemp * dR;
		Tti.Unit(4);
		for ( m=0 ; m < 3 ; m++ ) {
			for ( j = 0 ; j < 3 ; j++ ) {
				Tti(m,j)=Tti1(m,j);
			}
		}
		Tti(0,3)=xt(i,0);
		Tti(1,3)=yt(i,0);
		Tti(2,3)=zt(i,0);
		
      wayPoints[i] = Tti;
		for ( m=0 ; m < 4 ; m++ ) {
			for ( j = 0 ; j < 4 ; j++ ) {
				Tt[i][m][j]=Tti(m,j);            
			}
		}
	}
	
	/*//Rotational Trajectory:
	// Single-angle Change:
	
	da=2*M_PI/(n-1);
	kx=1; 
	ky=0;
	kz=0;
	s=sin(da);
	c=cos(da);
	v=1-c;
	
	//Rotation and Position Change:

	Matrix dR(3,3);
		dR(0,0)=pow(kx,2)*v+c;
		dR(0,1)=kx*ky*v-kz*s;
		dR(0,2)=kx*kz*v+ky*s;
		dR(1,0)=kx*ky*v+kz*s;
		dR(1,1)=pow(ky,2)*v+c;
		dR(1,2)=ky*kz*v-kx*s;
		dR(2,0)=kx*kz*v-ky*s;
		dR(2,1)=ky*kz*v+kx*s;
		dR(2,2)=pow(kz,2)*v+c;
		
	// Finding the trajectory points along the trajectory line:
	Tt = new double**[n];
	for (i = 0; i < n; ++i) {
		Tt[i] = new double*[4];
		for (j = 0; j < 4; ++j){
			Tt[i][j] = new double[4];
		}
	}
	for ( i=0 ; i < 4 ; i++ ) {
		for ( j = 0 ; j < 4 ; j++ ) {
			Tt[0][i][j]=Ti(i,j);
		}
	}
	
	Matrix Tti1(3,3),Tti(4,4);
	for (i=1; i<n; i++){
		Titemp=~Titemp;
		dR= dR ^ (i)
		Tti1 = Titemp * dR;
		for ( m=0 ; m < 3 ; m++ ) {
			for ( j = 0 ; j < 3 ; j++ ) {
				Tti(m,j)=Tti1(m,j);
			}
		}
		Tti(0,3)=Ti(0,3)+2000*cos(i*da);
		Tti(1,3)=Ti(1,3)+2000*sin(i*da);
		Tti(2,3)=Ti(2,3);
		Tti(3,0)=0;
		Tti(3,1)=0;
		Tti(3,2)=0;
		Tti(3,3)=1;

		for ( m=0 ; m < 4 ; m++ ) {
			for ( j = 0 ; j < 4 ; j++ ) {
				Tt[i][m][j]=Tti(m,j);
			}
		}
	}
*/
	//return Tt;
   return wayPoints;
}

Matrix WMRA_BPolynomial(double qi, double qf, double n){
	Matrix qtb(2,1);
	// Blending Factor:
	int b;
	b=5;

	// Initializing the time:
	double tt, tf, dt, qddb, tb, qdb, qb;
	double a01, a11, a21, a31, a41, a51, a02, a12, a22, a32, a42, a52;
	int i;
	tt=0;
	tf=abs(qf-qi);
	dt=tf/(n-1);
	
	if (tf > 0.001){
		// Blending procedure:
		// Time, position, velocity, and acceleration of the variable at the first blending point:
		qddb=b*4*(qf-qi)/pow(tf,2);
		tb=tf/2-sqrt(pow(qddb,2)*pow(tf,2)-4*qddb*(qf-qi))/abs(2*qddb);
		qdb=qddb*tb;
		qb=qi+qddb*pow(tb,2)/2;
		// Calculating the polynomial factors at the first blending point: From Eq.7.18 page 210 of Craig Book
		a01=qi;
		a11=0;
		a21=0.5*qddb;
		a31=(20*(qb-qi)-8*qdb*tb-2*qddb*pow(tb,2))/(2*pow(tb,3));
		// a41=(30*(qi-qb)+14*qdb*tb+qddb*pow(tb,2))/(2*pow(tb,4)); % Uncomment for 5th order polynomial.
		// a51=(12*(qb-qi)-6*qdb*tb)/(2*pow(tb,5)); % Uncomment for 5th order polynomial.
		// Calculating the polynomial factors at the second blending point: From Eq.7.18 page 210 of Craig Book
		a02=qb+qdb*(tf-2*tb);
		a12=qdb;
		a22=-0.5*qddb;
		a32=(20*(qf-a02)-12*a12*tb+2*qddb*pow(tb,2))/(2*pow(tb,3));
		// a42=(30*(a02-qf)+16*a12*tb-qddb*pow(tb,2))/(2*pow(tb,4)); % Uncomment for 5th order polynomial
		// a52=(12*(qf-a02)-6*a12*tb)/(2*pow(tb,5)); % Uncomment for 5th order polynomial.
	}
	
	// Calculating the intermediate joint angles along the trajectory from the initial to the final position:
	double *qttemp;
	qttemp = new double[n];
	for (i=0; i<n; i++){
		if (tf<=0.001){
			qttemp[i]=qi;
		}
		else if (tt<=tb){
			qttemp[i]=a01+a11*tt+a21*pow(tt,2)+a31*pow(tt,3); //+a41*pow(tt,4)+a51*pow(tt,5); // Uncomment before "+a41" for 5th order polynomial.
		}
		else if (tt>=(tf-tb)){
			qttemp[i]=a02+a12*(tt+tb-tf)+a22*pow((tt+tb-tf),2)+a32*pow((tt+tb-tf),3); //+a42*pow((tt+tb-tf),4)+a52*pow((tt+tb-tf),5); // Uncomment before "+42" for 5th order polynomial.
		}
		else {
			qttemp[i]=qb-qdb*(tb-tt);
		}
		tt = tt + dt;
	}    
	qtb.SetSize(n,1);
	for (i=0; i < n; i++){
		qtb(i,0) = qttemp[i];
	}
	delete [] qttemp; 
	return qtb;
}


Matrix WMRA_Linear(double qi, double qf, double n){
	
	Matrix qt(2,1);
	int i;
	double dq;
	dq =(qf-qi)/(n-1);

	double *qttemp;
	qttemp = new double[n];
	//for (i=1; i<n+1; i++){
	//	qttemp[i-1]=qi+dq*(i-1);
	//}
   for (i=0; i<n; i++){
		qttemp[i]=qi+dq*(i);
	}

	qt.SetSize(n,1);
	for (i=0; i < n; i++){
		qt(i,0) = qttemp[i];
	}
	delete [] qttemp; 

	return qt;
}


Matrix WMRA_Polynomial(double qi, double qf, double numWayPoints){
	
	Matrix qtp(numWayPoints,1);

	double tt=0;
	double tf=abs(qf-qi);
	double dt=tf/(numWayPoints-1);
	
	double *qttemp;
	qttemp = new double[numWayPoints];

	for (int i=0; i<numWayPoints; i++){
		if (tf<=0.001){
			qttemp[i]=qi;
		}
		else {
			qttemp[i]=qi+(qf-qi)*3*pow(tt,2)/pow(tf,2)-(qf-qi)*2*pow(tt,3)/pow(tf,3); //From Eq.7.3 and 7.6 page 204,205 of Craig Book
		}
		tt = tt + dt;
	}
	qtp.SetSize(numWayPoints,1);
	for (int i=0; i < numWayPoints; i++){
		qtp(i,0) = qttemp[i];
	}
	delete [] qttemp; 

	return qtp;
}