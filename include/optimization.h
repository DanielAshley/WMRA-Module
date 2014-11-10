#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H


#include "matrix.h"  
#include "kinematics.h"
#include "Utility.h"
#include <limits>

// #DEBUG - Remove after debuging complete
#include <fstream>
#include <iostream>

//KinematicOptimizer
class KinematicOptimizer{
public:
	KinematicOptimizer();
	Matrix WMRA_Opt(int i, double JLA, double JLO, Matrix Jo, double detJo, Matrix dq, vector<double> delta, double dt, vector<double> cur);
	void WMRA_Jlimit(Matrix& qmin, Matrix& qmax);
	Matrix WMRA_Opt(Matrix Jo, double detJo, vector<double> dx, vector<double> q, double dt);
   Matrix WMRA_Opt2(Matrix Jo, double detJo, vector<double> dx, vector<double> q, double dt);
private:
	Matrix dHo;
	std::ofstream weight_f, manipulability;
};
#endif;