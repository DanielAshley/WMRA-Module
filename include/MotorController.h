#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <exception>
#include <vector>
#include <ctype.h>
#include "WmraTypes.h"
#include "galilController.h"

using namespace std;

class MotorController {
public:
	enum motorControlMode  {POS_CONTROL = 0, LINEAR, VELOCITY};
	MotorController();
	~MotorController();

	/**
	* \brief Initializes the GalilController and sets the default variables in the Galil Controller.
	*/
	bool initialize();

	/**
	* \brief sets the motors run mode
	* \@param [in] mode The desired mode of the motors, either POS_CONTROL or LINEAR.
	*/
	bool setMotorMode(motorControlMode mode);

	/**
	* \brief gets the motors run mode
	*/
	int getMotorMode(){return motorMode;}

	/**
	* \brief adds the next linear segment. sends to galil controller
	* \@param [in] angles vector of joint angles in radians. 
	* \@param [in] speeds section speed in rad/s^-1 
	*/
	bool addLinearMotionSegment(vector<double> angles, vector<double> speeds);

	/**
	* \brief adds the linear end segment. sends to galil controller
	*/
	bool endLIseq();

	/// \brief starts the linear interpolation movement that is stored on the galil controller. sends to galil controller
	bool beginLI();

	/// \brief starts the linear interpolation movement that is stored on the galil controller. sends to galil controller
	bool clearLI();

	/// \brief  waits until all waypoints are finished. blocking.
	bool waitLinearMotionEnd(); 

	/// \brief return true if initialized
	bool isInitialized();

	/// \brief return true if movement is simulated
	bool isSimulated();
	
	/// \brief return true if debug mode is true
	bool isDebug();

	/// \brief return true if motion has finished
	bool motionFinished();

	/// \brief sends the default values to the galil controller
	bool wmraSetup();

	/// \brief emergancy stop
	bool Stop();

	/**
	* \brief emergancy stop a single motor. sends to galil controller
	* \@param [in] motorNum the number(int) of the motor to be stopped. 
	*/
	bool Stop(int motorNum);

	/**
	* \brief returns the current motor angle in radians. reads from galil controller
	* \@param [in] motorNum the number(int) of the motor for the position to be read. 
	*/
	double readPos(int motorNum);

	/// \brief returns all of the current motor angle in radians. reads from galil controller
	std::vector<double> readPosAll();

	/// \brief returns all of the current motor angle in radians. reads from galil controller
	std::vector<int> readPosAll_raw();

	/// \brief returns the error in
	double readPosErr(int motorNum); 

	/**
	* \brief sets the maximum velocity. sends to galil controller
	* \@param [in] motorNum the number(int) of the motor for the velocity to be set.
	* \@param [in] angularVelocity the angular velocity in rad/s
	*/
	bool setMaxVelocity(int motorNum, double angularVelocity);

	/**
	* \brief sets the acceleration. sends to galil controller
	* \@param [in] motorNum the number(int) of the motor for the acceleration to be set.
	* \@param [in] angularAccelaration the angular acceleration in rad/s^2
	*/
	bool setAccel(int motorNum, double angularAccelaration);

	/**
	* \brief sets all motors acceleration. sends to galil controller
	* \@param [in] acclVal the angular acceleration in rad/s^2 for all motors. vector<int>.
	*/
	bool setAccelAll(std::vector<int> acclVal);

	/**
	* \brief sets the deceleration. sends to galil controller
	* \@param [in] motorNum the number(int) of the motor for the deceleration to be set.
	* \@param [in] angularDecelaration the angular deceleration in rad/s^2
	*/
	bool setDecel(int motorNum, double angularDecelaration);

	/**
	* \brief converts encoder counts to angles(radians) for selected motor
	* \@param [in] motorNum the number(int) of the motor for conversion.
	* \@param [in] enc the encoder counts to be converted.
	*/
	double encToAng(int motorNum, long enc);

	/**
	* \brief converts angles(radians) to encoder counts for selected motor
	* \@param [in] motorNum the number(int) of the motor for conversion.
	* \@param [in] angle the angle(radians) to be converted.
	*/
	long angToEnc(int motorNum, double angle);

	/**
	* \brief sets and moves to desired position. sends to galil controller
	* \@param [in] motorNum the number(int) of the motor to be moved.
	* \@param [in] angle the desired position in angles(radians).
	*/
	bool positionControl(int motorNum, double angle);

	/**
	* \brief sets the current position of the motor. sends to galil controller
	* \@param [in] motorNum the number(int) of the motor to be set.
	* \@param [in] angle the position to be set in angles(radians).
	*/
	bool definePosition(int motorNum, double angle);

	std::vector<double> getLastKnownPos();

	/// \Turns motors on
	bool motorsOn();

	/// \Turns motors off
	bool MotorsOFF();

	
	/// \Sends Jof command for velocity control
	bool sendJog(vector<int> value);


	vector<double> prevPosition;
private:
	galilController controller;
	bool setDefaults(); // set defaults
	bool setBrushedMotors();
	bool setBrushedMode(int motorNum, int mode);
	bool setSmoothing(double value);
	bool initialized ;
	motorControlMode motorMode;
	string ipAddr;
	double enc2Radian[8];
	double radian2Enc[8];
	bool brushedMotors[8];
	double motorVelocity[8];
	double motorAccel[8];
	double motorDecel[8];
	double smoothingVal;
	double readyPosition[8];
	double parkPosition[8];
	string motorLookup[8];
	bool setPID(int motorNum, int P, int I, int D);
	bool isValidMotor(int motorNum);
	
	vector<double> curPosition;
	vector<double> lastKnownPos;
};

#endif;