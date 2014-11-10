

#include <vector>
#include <stdio.h>
#include <iostream>
#include "matrix.h" 
#include "Utility.h"

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>

//#define PI 3.14159265

using namespace std;

double degToRad(double deg){
   return deg *(M_PI/180);
}
double radToDeg(double rad){
   return rad *(180/M_PI);
}

void dest(vector<double> &tgt, int val)
{
	vector<double> temp;
	temp.resize(10);

	if(val == 1)
	{
		temp[0] = (M_PI/2)+0.1;
		temp[1] = (M_PI/2);//-0.5;
		temp[2] = 0;//-(0.5);
		temp[3] = (M_PI/2);//-0.5;
		temp[4] = (M_PI/2);//+0.7;
		temp[5] = -(M_PI/2);//-0.4;
		temp[6] = 0;//.6;
		temp[7] = 0;
		temp[8] = 0;
		temp[9] = 0;
	}
	else if(val == 2)
	{
		temp[0] = (M_PI/2);
		temp[1] = (M_PI/2);
		temp[2] = 0;
		temp[3] = (M_PI/2);
		temp[4] = (M_PI/2);
		temp[5] = -(M_PI/2);
		temp[6] = 0;
		temp[7] = 0;
		temp[8] = 0;
		temp[9] = 0;
	}
	else if(val == 0)
	{
		double temp2;
		for(int i = 0; i < temp.size(); i++)
		{
			cout << "Joint " << i << " destination in radians: ";
			cin >> temp2;
			cout << endl;
			temp[i] = temp2;
		}
	}
	tgt = temp;
}

int sign(double x){
	int y;
	if (x>0){
		y=1;
	}
	else if (x==0){
		y=0;
	}
	else {
		y=-1;
	}
	return y;
}

void WMRA_delta(vector<double> &tgt, Matrix Tid, Matrix Tdd){

	vector<double> ep,eo,delta;
	ep.resize(3);
	eo.resize(3);
	delta.resize(6);

	int i,j;
	for (i=0; i < 3; i++){
		ep[i] = Tdd(i,3)-Tid(i,3);
	}
	
	vector<double> temp3;
	temp3.resize(3);
	for (j=0; j<3; j++){
		vector<double> temp1, temp2;
		temp1.push_back(Tid(0,j));
		temp1.push_back(Tid(1,j));
		temp1.push_back(Tid(2,j));
		temp2.push_back(Tdd(0,j));
		temp2.push_back(Tdd(1,j));
		temp2.push_back(Tdd(2,j));
		vector<double> crossed(3);
		cross(crossed, temp1, temp2);
		temp3[0]=temp3[0]+crossed[0];
		temp3[1]=temp3[1]+crossed[1];
		temp3[2]=temp3[2]+crossed[2];
	}
	eo[0]=temp3[0]/2;
	eo[1]=temp3[1]/2;
	eo[2]=temp3[2]/2;
	//eo*=(0.5);

	// delta definition
	for (i=0;i<3;i++){
		delta[i]=ep[i]; 
	}
	for (i=3;i<6;i++){
		delta[i]=eo[i-3];
	}

////eo=0.5*( cross(Ti(1:3,1),Td(1:3,1)) + cross(Ti(1:3,2),Td(1:3,2)) + cross(Ti(1:3,3),Td(1:3,3)) );  
////From equation 17 on page 189 of (Robot Motion Planning and Control) Book by Micheal Brady et al. Taken from the paper (Resolved-Acceleration Control of Mechanical Manipulators) By John Y. S. Luh et al.
	tgt = delta;
}

void cross(vector<double> &tgt, vector<double> a, vector<double> b)
{
  vector<double> r;
  r.resize(3);
  r[0] = a[1]*b[2]-a[2]*b[1];
  r[1] = a[2]*b[0]-a[0]*b[2];
  r[2] = a[0]*b[1]-a[1]*b[0];
  tgt = r;
}
