#ifndef UTILITY_H
#define UTILITY_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "matrix.h" 
#include "trajectory.h"

using namespace std;

double degToRad(double deg);
double radToDeg(double rad);
void dest(vector<double> &tgt, int val);
int sign(double x);
void WMRA_delta(vector<double> &tgt, Matrix Tid, Matrix Tdd);
void cross(vector<double> &tgt, vector<double> a, vector<double> b);

#endif;