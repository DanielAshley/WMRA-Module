/* This function is for the resolved rate and optimization solution of the USF WMRA with 9 DOF.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% COPY RIGHTS RESERVED %%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Developed By: Redwan M. Alqasemi %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Modified By:Ana Catalina Torres %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% April 2010%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Function Declaration:*/

#include "matrix.h"  
#include "optimization.h"
#include "kinematics.h"
#include "Utility.h"
#include <limits>

// #DEBUG - Remove after debuging complete
#include <fstream>
#include <iostream>

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace math;

KinematicOptimizer::KinematicOptimizer()
{
	weight_f.open("data/weight.csv");
	manipulability.open("data/man.csv");
	dHo.Null(7,1);
}

void KinematicOptimizer::WMRA_Jlimit(Matrix& qmin, Matrix& qmax){

	//double qmintemp[7]= {-360*M_PI/180,-360*M_PI/180,-360*M_PI/180,-360*M_PI/180,-360*M_PI/180,-90*M_PI/180,-360*M_PI/180};
   //double qmintemp[7]= {-170*M_PI/180,-170*M_PI/180,-170*M_PI/180,-170*M_PI/180,-170*M_PI/180,-79*M_PI/180,-200*M_PI/180};
   //double qmintemp[7]= {-10*M_PI/180,-15*M_PI/180,-170*M_PI/180,-170*M_PI/180,-170*M_PI/180,-90*M_PI/180,-200*M_PI/180};
   double qmintemp[7]= {-10*M_PI/180,-15*M_PI/180,-170*M_PI/180,-170*M_PI/180,-170*M_PI/180, 05*M_PI/180,-200*M_PI/180};
   
   //double qmaxtemp[7] = {360*M_PI/180,360*M_PI/180,360*M_PI/180,360*M_PI/180,360*M_PI/180,90*M_PI/180,360*M_PI/180};
	//double qmaxtemp[7] = {170*M_PI/180,170*M_PI/180,170*M_PI/180,170*M_PI/180,170*M_PI/180,79*M_PI/180,200*M_PI/180};
	double qmaxtemp[7] = {200*M_PI/180,120*M_PI/180,170*M_PI/180,170*M_PI/180,170*M_PI/180,90*M_PI/180,200*M_PI/180};
	int i;
	for (i=0; i < 7; i++){
		qmin(0,i) = qmintemp[i];
		qmax(0,i) = qmaxtemp[i];
	}
}

Matrix KinematicOptimizer::WMRA_Opt2(Matrix Jo, double detJo, vector<double> dx, vector<double> q_prev, double dt){

	// Reading the Wheelchair's constant dimentions, all dimentions are converted in millimeters:
	Matrix L(1,1);
	L=WMRA_WCD();
	Matrix qmin(1,7), qmax(1,7);
   //this line????
   WMRA_Jlimit(qmin, qmax); // #DEBUG - Joint Limits

	double inf = std::numeric_limits<double>::infinity();
	int WCA ;
	Matrix pinvJo(7,7);
	Matrix dH(7,1);
	Matrix mul(1,1),  mul1(1,1), mul2(1,1);
	Matrix W(7,7), dia(7,1), Winv(7,7);
	double dof;



   //**need to calculate current angle ***//

   //Matrix Winv;
   Winv.Unit(7);
   Matrix Jotrans= ~Jo;
   mul = Jo * Winv * Jotrans;
   detJo= sqrt(mul.Det());

   // Calculating the variable scale factor, sf:   
 //  int wo=20000000;
	//int ko=350000;
   double wo = 20000000;
   double ko = 350000;
   double sf;
	if (detJo<wo){
		sf=ko*pow((1-detJo/wo),2);        // from eq. 9.79 page 268 of Nakamura's book.
	}
	else {
		sf=0;
	}

   Matrix sfm;
	sfm.Unit(dx.size());
	sfm *= sf;
	pinvJo= Winv * Jotrans * (!(mul +sfm));
   //pinvJo = Winv * Jotrans * (!mul);  //#debug try this

   Matrix temp_dx;
	temp_dx.SetSize(6,1);
   for(int i = 0; i < dx.size(); i++){
		temp_dx(i,0) = dx[i];
	}
	Matrix dq = pinvJo * temp_dx;

   // claculate current predicted angle
   vector<double> q(7); //current angle
   for(int i=0; i < 7; i++){
      q[i] = q_prev[i] + dq(i,0); //current angle = prev angle + delta angle
   }


	// Creating the gradient of the optimization function to avoid joint limits:
   //Matrix dH;
	dH.Null(7,1);

	// JLA
	for (int j=0; j<7; j++){
		dH(j,0)=-0.25*pow((qmax(0,j)-qmin(0,j)),2)*(2*q[j]-qmax(0,j)-qmin(0,j))/(pow((qmax(0,j)-q[j]),2)*pow((q[j]-qmin(0,j)),2));
		// Re-defining the weight in case the joint is moving away from it's limit or the joint limit was exceeded:
		if (abs(dH(j,0)) < abs(dHo(j,0)) && q[j] < qmax(0,j) && q[j] > qmin(0,j)){
			dH(j,0)=0;
		}
		else if (abs(dH(j,0)) < abs(dHo(j,0)) && (q[j] >= qmax(0,j) || q[j] <= qmin(0,j))){
			dH(j,0)=inf;
		}
		else if (abs(dH(j,0)) > abs(dHo(j,0)) && (q[j] >= qmax(0,j) || q[j] <= qmin(0,j))){
			dH(j,0)=0;
		}
	}
	//KinematicOptimizer::dHo = dH;
	//cout<<"dHo is \n\n"<<dHo<<"\n\n";
	// The case when arm-only control is required with no wheelchair motion:

	W.Null(7,7);
	Winv.Null(7,7);


	// #DEBUG - Weights (DELETE this comment)
	// The weight matrix to be used for the Weighted Least Norm Solution with Joint Limit Avoidance:
	
	for (int j=0; j < 7; j++){
		for (int k=0; k < 7; k++){
			if (j==k){
				W(j,k) = 1 + abs(dH(j,0));
				Winv(j,k) = 1 / W(j,k);	   // The inverse of the diagonal weight matrix:				
			}
		}
	}

	KinematicOptimizer::weight_f << W(0,0) << "," << W(1,1) << "," << W(2,2) << "," << W(3,3) << "," << W(4,4) << "," << W(5,5) << "," << W(6,6) << endl;
	

	// Redefining the determinant based on the weight:

	Jotrans = ~Jo;
	mul = Jo * Winv * Jotrans;
	detJo = sqrt(mul.Det());
	KinematicOptimizer::manipulability << detJo << endl;

	dof=dx.size();
	// SR-Inverse and Weighted Least Norm Optimization:
	// Calculating the variable scale factor, sf:
	if (detJo<wo){
		sf=ko*pow((1-detJo/wo),2);        // from eq. 9.79 page 268 of Nakamura's book.
	}
	else {
		sf=0;
	}
	//	cout<<"sf is\n\n"<<sf<<"\n\n";
	// Calculating the SR-Inverse of the Jacobian:

	sfm.Unit(dx.size());
	sfm*=sf;
	pinvJo= Winv * Jotrans * (!(mul +sfm));

	// Calculating the joint angle change optimized based on the Weighted Least Norm Solution:
	// Here, dq of the wheels are translated from radians to distances travelled after using the Jacobian.

	dq = pinvJo * temp_dx;


   bool JLO = true;
   if(JLO){
      for (int k=0; k<7; k++){
         if (q[k] >= qmax(0,k) || q[k] <= qmin(0,k)){
            dq(k,0)=0;
         }
      }
      Matrix dqmax(7,1);
      for (int k=0; k<7; k++){
         dqmax(k,0)=0.9;
      }
      dqmax*=(dt); 
      for (int k=0; k<dq.RowNo(); k++){
         if (abs(dq(k,0)) >= dqmax(k,0)){
            dq(k,0)=sign(dq(k,0))*dqmax(k,0);
         }
      }
   }

   /** update DHo for next iteration with adjusted joint angles **/
   for(int i=0; i < 7; i++){
      q[i] = q_prev[i] + dq(i,0); //current angle = prev angle + delta angle
      dH(i,0)=-0.25*pow((qmax(0,i)-qmin(0,i)),2)*(2*q[i]-qmax(0,i)-qmin(0,i))/(pow((qmax(0,i)-q[i]),2)*pow((q[i]-qmin(0,i)),2));
   }   
   KinematicOptimizer::dHo = dH;

	return dq;
}

Matrix KinematicOptimizer::WMRA_Opt(Matrix Jo, double detJo, vector<double> dx, vector<double> q, double dt){

	// Reading the Wheelchair's constant dimentions, all dimentions are converted in millimeters:
	Matrix L(1,1);
	L=WMRA_WCD();
	Matrix qmin(1,7), qmax(1,7);
   //this line????
   WMRA_Jlimit(qmin, qmax); // #DEBUG - Joint Limits

	double inf = std::numeric_limits<double>::infinity();
	int WCA, wo, ko, j, k;
	Matrix pinvJo(7,7);
	Matrix dH(7,1);
	Matrix mul(1,1),  mul1(1,1), mul2(1,1), Jotrans(7,7);
	Matrix W(7,7), dia(7,1), Winv(7,7);
	double dof;


	// Creating the gradient of the optimization function to avoid joint limits:
	dH.Null(7,1);

	// JLA
	for (j=0; j<7; j++){
		dH(j,0)=-0.25*pow((qmax(0,j)-qmin(0,j)),2)*(2*q[j]-qmax(0,j)-qmin(0,j))/(pow((qmax(0,j)-q[j]),2)*pow((q[j]-qmin(0,j)),2));
		// Re-defining the weight in case the joint is moving away from it's limit or the joint limit was exceeded:
		if (abs(dH(j,0)) < abs(dHo(j,0)) && q[j] < qmax(0,j) && q[j] > qmin(0,j)){
			dH(j,0)=0;
		}
		else if (abs(dH(j,0)) < abs(dHo(j,0)) && (q[j] >= qmax(0,j) || q[j] <= qmin(0,j))){
			dH(j,0)=inf;
		}
		else if (abs(dH(j,0)) > abs(dHo(j,0)) && (q[j] >= qmax(0,j) || q[j] <= qmin(0,j))){
			dH(j,0)=0;
		}
	}
	KinematicOptimizer::dHo = dH;
	//cout<<"dHo is \n\n"<<dHo<<"\n\n";
	// The case when arm-only control is required with no wheelchair motion:

	W.Null(7,7);
	Winv.Null(7,7);
	wo=20000000;
	ko=350000;

	// #DEBUG - Weights (DELETE this comment)
	// The weight matrix to be used for the Weighted Least Norm Solution with Joint Limit Avoidance:
	
	for (j=0; j < 7; j++){
		for (k=0; k < 7; k++){
			if (j==k){
				W(j,k)=1+abs(dH(j,0));
				dia(j,0)=W(j,k);
				// The inverse of the diagonal weight matrix:
				Winv(j,k)=1/(dia(j,0));					
			}
		}
	}

	KinematicOptimizer::weight_f << W(0,0) << "," << W(1,1) << "," << W(2,2) << "," << W(3,3) << "," << W(4,4) << "," << W(5,5) << "," << W(6,6) << endl;
	

	// Redefining the determinant based on the weight:

	Jotrans=~Jo;
	//		cout<<"Jo' is\n\n"<<Jotrans<<"\n\n";
	mul1=Winv*Jotrans;
	//		cout<<"mul1 is\n\n"<<mul1<<"\n\n";
	mul2=Jo*mul1;
	//		cout<<"mul2 is\n\n"<<mul2<<"\n\n";
	mul=mul2;
	//mul= Jo * Winv * Jotrans;
	//		cout<<"mul is\n\n"<<mul<<"\n\n";
	detJo= sqrt(mul.Det());
	//		cout<<"detJo is\n\n"<<detJo<<"\n\n";
	KinematicOptimizer::manipulability << detJo << endl;
	dof=dx.size();
	//		cout<<"dof is\n\n"<<dof<<"\n\n";

	// SR-Inverse and Weighted Least Norm Optimization:
	double sf;

	// Calculating the variable scale factor, sf:
	if (detJo<wo){
		sf=ko*pow((1-detJo/wo),2);        // from eq. 9.79 page 268 of Nakamura's book.
	}
	else {
		sf=0;
	}
	//	cout<<"sf is\n\n"<<sf<<"\n\n";
	// Calculating the SR-Inverse of the Jacobian:
	Matrix sfm(2,2);
	sfm.Unit(dof);
	sfm*=(sf);
	mul= mul + sfm;
	mul=!mul;
	pinvJo=Winv*Jotrans*mul;
	//cout<<"pinvJo is\n\n"<<pinvJo<<"\n\n";
	// Calculating the joint angle change optimized based on the Weighted Least Norm Solution:
	// Here, dq of the wheels are translated from radians to distances travelled after using the Jacobian.
	Matrix temp_dx;
	temp_dx.SetSize(6,1);
	for(int i = 0; i < 6; i++)
	{
		temp_dx(i,0) = dx[i];
	}
	Matrix dq(6,1);
	dq = pinvJo * temp_dx;
	//dq.SetSize(7,1);
	//	dq(7,0)= 0;//dq(7,0)*L(0,4);

	//cout<<"dq is\n\n"<<dq<<"\n\n";

   bool JLO = true;
   if(JLO){
      for (k=0; k<7; k++){
         if (q[k] >= qmax(0,k) || q[k] <= qmin(0,k)){
            dq(k,0)=0;
         }
      }
      Matrix dqmax;
      dqmax.Null(7,1);
      for (k=0; k<7; k++){
         dqmax(k,0)=0.5;
      }
      dqmax*=(dt); 
      for (k=0; k<dq.RowNo(); k++){
         if (abs(dq(k,0)) >= dqmax(k,0)){
            dq(k,0)=sign(dq(k,0))*dqmax(k,0);
         }
      }
   }
	return dq;
}


Matrix KinematicOptimizer::WMRA_Opt(int i, double JLA, double JLO, Matrix Jo, double detJo, Matrix dq, vector<double> delta, double dt, vector<double> cur){

	Matrix dHo;
	dHo.Null(7,1);
	Matrix q(cur.size(), 1), dx(delta.size(),1);
	for(int i = 0; i < cur.size(); i++)
		q(i,0) = cur[i];

	for(int i = 0; i < delta.size(); i++)
		dx(i,0) = delta[i];


	// Reading the Wheelchair's constant dimentions, all dimentions are converted in millimeters:
	Matrix L(1,1);
	L=WMRA_WCD();
	Matrix qmin(1,7), qmax(1,7);
	WMRA_Jlimit(qmin, qmax); 

	double inf = std::numeric_limits<double>::infinity();
	int WCA, wo, ko, j, k;
	Matrix pinvJo(7,7);
	Matrix dH(7,1);
	Matrix mul(1,1),  mul1(1,1), mul2(1,1), Jotrans(7,7);
	Matrix W(7,7), dia(7,1), Winv(7,7);
	double dof;
	// The case when wheelchair-only control is required with no arm motion:
	if (i==0){
		WCA=3;
		// Calculating the Inverse of the Jacobian, which is always non-singular:
		Matrix Jotemp(2,2);
		for (j=0; j<2; j++){
			for (k=0; k<2; k++){
				Jotemp(j,k)=Jo(j,k);
			}
		}
		pinvJo=!Jotemp;
		// Calculating the joint angle change:
		// Here, dq of the wheels are translated from radians to distances travelled after using the Jacobian.
		dq = pinvJo * dx;
		dq(0,0)=dq(0,0)*L(0,4);
	}
	else {
		// Creating the gradient of the optimization function to avoid joint limits:
		dH.Null(7,1);
		//cout<<"dHo is \n\n"<<dHo<<"\n\n";
		if (JLA==1){
			for (j=0; j<7; j++){
				dH(j,0)=-0.25*pow((qmax(0,j)-qmin(0,j)),2)*(2*q(j,0)-qmax(0,j)-qmin(0,j))/(pow((qmax(0,j)-q(j,0)),2)*pow((q(j,0)-qmin(0,j)),2));
				// Re-defining the weight in case the joint is moving away from it's limit or the joint limit was exceeded:
				if (abs(dH(j,0)) < abs(dHo(j,0)) && q(j,0) < qmax(0,j) && q(j,0) > qmin(0,j)){
					dH(j,0)=0;
				}
				else if (abs(dH(j,0)) < abs(dHo(j,0)) && (q(j,0) >= qmax(0,j) || q(j,0) <= qmin(0,j))){
					dH(j,0)=inf;
				}
				else if (abs(dH(j,0)) > abs(dHo(j,0)) && (q(j,0) >= qmax(0,j) || q(j,0) <= qmin(0,j))){
					dH(j,0)=0;
				}
			}
		}
		dHo = dH;
		//cout<<"dHo is \n\n"<<dHo<<"\n\n";
		// The case when arm-only control is required with no wheelchair motion:
		if (dq.RowNo()==7){
			W.Null(7,7);
			Winv.Null(7,7);
			WCA=2;
			wo=20000000;
			ko=350000;
			// The weight matrix to be used for the Weighted Least Norm Solution with Joint Limit Avoidance:
			for (j=0; j < 7; j++){
				for (k=0; k < 7; k++){
					if (j==k){
						W(j,k)=1+abs(dH(j,0));
						dia(j,0)=W(j,k);
						// The inverse of the diagonal weight matrix:
						Winv(j,k)=1/(dia(j,0));					
					}
				}
			}
		}
		// The case when wheelchair-and-arm control is required:
		else {
			WCA=1;
			wo=34000000;
			ko=13;
			// The weight matrix to be used for the Weighted Least Norm Solution:
			W.Null(9,9);
			W(7,7)=10;
			W(8,8)=10;
			for (j=0; j < 7; j++){
				for (k=0; k < 7; k++){
					if (j==k){
						W(j,k)=1+abs(dH(j,0));
					}
				}
			}
			dia.SetSize(9,1);
			Winv.Null(9,9);
			for (j=0; j < 9; j++){
				for (k=0; k < 9; k++){
					if (j==k){
						dia(j,0)=W(j,k);
						// The inverse of the diagonal weight matrix:
						Winv(j,k)=1/(dia(j,0));
					}
				}
			}
		}
		//	cout<<"dia is\n\n"<<dia<<"\n\n";
		//	cout<<"Winv is\n\n"<<Winv<<"\n\n";

		// Redefining the determinant based on the weight:
		if (i==1 || i==2){
			Jotrans=~Jo;
			//		cout<<"Jo' is\n\n"<<Jotrans<<"\n\n";
			mul1=Winv*Jotrans;
			//		cout<<"mul1 is\n\n"<<mul1<<"\n\n";
			mul2=Jo*mul1;
			//		cout<<"mul2 is\n\n"<<mul2<<"\n\n";
			mul=mul2;
			//mul= Jo * Winv * Jotrans;
			//		cout<<"mul is\n\n"<<mul<<"\n\n";
			detJo= sqrt(mul.Det());
			//		cout<<"detJo is\n\n"<<detJo<<"\n\n";
		}
		dof=dx.RowNo();
		//		cout<<"dof is\n\n"<<dof<<"\n\n";
	}

	// SR-Inverse and Weighted Least Norm Optimization:
	double sf;
	if (i==1){
		// Calculating the variable scale factor, sf:
		if (detJo<wo){
			sf=ko*pow((1-detJo/wo),2);        // from eq. 9.79 page 268 of Nakamura's book.
		}
		else {
			sf=0;
		}
		//	cout<<"sf is\n\n"<<sf<<"\n\n";
		// Calculating the SR-Inverse of the Jacobian:
		Matrix sfm(2,2);
		sfm.Unit(dof);
		sfm*=(sf);
		mul= mul + sfm;
		mul=!mul;
		pinvJo=Winv*Jotrans*mul;
		//cout<<"pinvJo is\n\n"<<pinvJo<<"\n\n";
		// Calculating the joint angle change optimized based on the Weighted Least Norm Solution:
		// Here, dq of the wheels are translated from radians to distances travelled after using the Jacobian.
		if (WCA==2){
			dq = pinvJo * dx;
		}
		else {
			dq = pinvJo * dx;
			//		cout<<"dq is\n\n"<<dq<<"\n\n";
			dq(7,0)=dq(7,0)*L(0,4);
		}
		//	cout<<"dq is\n\n"<<dq<<"\n\n";
	}	

	// Pseudo Inverse and Weighted Least Norm Optimization:
	else if (i==2){
		// Calculating the Pseudo Inverse of the Jacobian:
		mul=!mul;
		pinvJo=Winv*Jotrans*mul;
		// Calculating the joint angle change optimized based on the Weighted Least Norm Solution:
		// Here, dq of the wheels are translated from radians to distances travelled after using the Jacobian.
		if (WCA==2){
			dq=pinvJo*dx;
		}
		else {
			dq=pinvJo*dx;
			dq(7,0)=dq(7,0)*L(0,4);
		}
	}

	// SR-Inverse and Projection Gradient Optimization based on Euclidean norm of errors:
	else if (i==3){
		Jotrans=~Jo;
		// Calculating the variable scale factor, sf:
		if (detJo<wo){
			sf=ko*pow((1-detJo/wo),2);        // from eq. 9.79 page 268 of Nakamura's book.
		}
		else {
			sf=0;
		}
		// Calculating the SR-Inverse of the Jacobian:
		Matrix sfm(2,2);
		sfm.Unit(dof);
		sfm*=(sf);
		//	cout<<"\n sfm  is: "<<sfm<<"\n";
		//	cout<<"\n Jo is: "<<Jo<<"\n";
		//	cout<<"\n Jotrans is: "<<Jotrans<<"\n";
		mul= Jo * Jotrans;
		//	cout<<"\n mul is: "<<mul<<"\n";
		mul= mul + sfm;
		//	cout<<"\n mul 2 is: "<<mul<<"\n";
		mul=!mul;
		//	cout<<"\n mul 3 is: "<<mul<<"\n";
		pinvJo=Jotrans * mul;
		//	cout<<"\n pinvJo is: "<<pinvJo<<"\n";
		// Calculating the joint angle change optimized based on minimizing the Euclidean norm of errors:
		// Here, dq of the wheels are translated from distances travelled to radians, and back after using the Jacobian.
		Matrix unit(7,7);
		if (WCA==2){
			unit.Unit(7);
			mul=unit-pinvJo*Jo;
			//dq=pinvJo*dx+mul*dq;
			mul*=(0.001);
			dq=pinvJo*dx+mul*dH;
			//		cout<<"\n dq is: "<<pinvJo<<"\n";

		}
		else {
			//dq(7,0)=dq(7,0)/L(0,4);
			unit.Unit(9);
			mul=unit-pinvJo*Jo;
			//dq=pinvJo*dx+mul*dq;
			mul*=(0.001);
			dH.SetSize(9,1);
			dq=pinvJo*dx+mul*dH;
			dq(7,0)=dq(7,0)*L(0,4);
			//		cout<<"\n dq is: "<<pinvJo<<"\n";
		}
	}
	// Pseudo Inverse and Projection Gradient Optimization based on Euclidean norm of errors:
	else if (i==4){
		Jotrans=~Jo;
		// Calculating the Pseudo Inverse of the Jacobian:
		mul= Jo * Jotrans;
		mul=!mul;
		pinvJo=Jotrans * mul;
		// Calculating the joint angle change optimized based on minimizing the Euclidean norm of errors:
		// Here, dq of the wheels are translated from distances travelled to radians, and back after using the Jacobian.
		Matrix unit(7,7);
		if (WCA==2){
			unit.Unit(7);
			mul=unit-pinvJo*Jo;
			//dq=pinvJo*dx+mul*dq;
			mul*=(0.001);
			dq=pinvJo*dx+mul*dH;
		}
		else {
			//dq(7,0)=dq(7,0)/L(0,4);
			unit.Unit(9);
			mul=unit-pinvJo*Jo;
			//dq=pinvJo*dx+mul*dq;
			mul*=(0.001);
			dH.SetSize(9,1);
			dq=pinvJo*dx+mul*dH;
			dq(7,0)=dq(7,0)*L(0,4);
		}
	}
	if (JLO==1){
		// A safety condition to stop the joint that reaches the joint limits in the arm:
		if (WCA!=3){
			for (k=0; k<7; k++){
				if (q(k,0) >= qmax(0,k) || q(k,0) <= qmin(0,k)){
					dq(k,0)=0;
				}
			}
			//	cout<<"dq is\n\n"<<dq<<"\n\n";
		}

		// A safety condition to slow the joint that exceeds the velocity limits in the WMRA:
		Matrix dqmax;
		if (WCA==3){
			dqmax(0,0)=100;
			dqmax(1,0)=0.15;
			dqmax*=(dt);     // Joity velocity limits when the time increment is dt second.
		}
		else {
			dqmax.Null(9,1);
			for (k=0; k<7; k++){
				dqmax(k,0)=0.5;
			}
			dqmax(7,0)=100;
			dqmax(8,0)=0.15;
			//		cout<<"dqmax is\n\n"<<dqmax<<"\n\n";
			//		cout<<"dt is\n\n"<<dt<<"\n\n";
			dqmax*=(dt);     // Joint velocity limits when the time increment is dt second.
			//		cout<<"dqmax is\n\n"<<dqmax<<"\n\n";
		}
		for (k=0; k<dq.RowNo(); k++){
			if (abs(dq(k,0)) >= dqmax(k,0)){
				dq(k,0)=sign(dq(k,0))*dqmax(k,0);
			}
		}
		//	cout << "\ndq is\n\n" << dq  ;
	}

	return dq;
}
