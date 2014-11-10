#include "stringUtility.h"
using namespace std;

string toString(int val){
    ostringstream stream;
	stream << val;
    return stream.str();
}

string toString(float val){
    ostringstream stream;
	stream << val;
    return stream.str();
}

string toString(double val){
    ostringstream stream;
	stream << val;
    return stream.str();

}

