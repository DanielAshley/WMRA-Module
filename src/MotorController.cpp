
#include "ConfigReader.h"
#include "stringUtility.h"
#include "MotorController.h"
using namespace std;



MotorController::MotorController()
{
	initialized = false;
	motorLookup[0] = "A";
	motorLookup[1] = "B";
	motorLookup[2] = "C";
	motorLookup[3] = "D";
	motorLookup[4] = "E";
	motorLookup[5] = "F";
	motorLookup[6] = "G";
	motorLookup[7] = "H";
}

MotorController::~MotorController()
{
//	controller.~galilController();
}

bool MotorController::initialize(){

	initialized = controller.initialize();
	if(initialized)
		initialized = setDefaults();
	if(initialized)
		initialized = wmraSetup();
	if(initialized)
		return true; //cout << "Motor Controller Initialized" << endl;
	else {
		cout << "Error: Initialization of Motor Controller FAILED" << endl;
		return false;
	}
	return true;
}

bool MotorController::setMotorMode(motorControlMode mode) // 0=Position Tracking, 1=Linear Interpolation, 2=velocity control
{
	/*gripper will always be in position control mode*/
	controller.command("PTH=1");

	/* set mode for 7 arm joints A through G */
	if(mode == MotorController::POS_CONTROL){ //position tracking		
		controller.command("PTA=1");
		controller.command("PTB=1");
		controller.command("PTC=1");
		controller.command("PTD=1");
		controller.command("PTE=1");
		controller.command("PTF=1");
		controller.command("PTG=1");
		motorMode = mode;
	}
	else if(mode == MotorController::LINEAR){ //linear control mode
		/* galil manual pg.88 (pdf pg.98) */
		controller.command("LM ABCDEFG");
		motorMode = mode;
	}
	else if(mode == MotorController::VELOCITY) {
		controller.command("STABCDEFGH");		
		controller.command("JG0.0,0.0,0.0,0.0,0.0,0.0,0.0");
		controller.command("STABCDEFGH");	
		controller.command("BGABCDEFGH");	
		motorMode = mode;
	}
	else
		return 0;
	return 1;
}

bool MotorController::addLinearMotionSegment(vector<double> angles, vector<double> speeds)
{	
	if(controller.isSimulated()){
		if(angles.size() == 8){
			for(int i = 0; i < 8; i++)
				curPosition[i] += angles[i];
		}
		else if(angles.size() == 7){
			for(int i = 0; i < 7; i++)
				curPosition[i] += angles[i];
		}			
		return true;
	}
	if(angles.size() == 7 && speeds.size() == 7)
	{ 
		/*convert angles to position counts
		* also Galil will return an error if all positions sent are 0
		* so check if all positions are zero
		*/
		vector<long> posCount(7);
		double zeroErrCheck = 0;
		for(int i = 0; i < 7 ; i++){
			posCount[i] = (long)angToEnc(i,angles[i]);
			zeroErrCheck += abs(posCount[i]);
		}
		/* If all positions are zero, do not send command to galil controller*/
		if(zeroErrCheck < 1) return false; 

		//calculate vector speed
		vector<double> speedCount(7);
		for(int i = 0; i < 7 ; i++){
			speedCount[i] = angToEnc(i,speeds[i]);
		}
		double vectorSpeed; /// this is for Galil controller
		vectorSpeed = (speedCount[0]*speedCount[0])+(speedCount[1]*speedCount[1])+(speedCount[2]*speedCount[2])+
			(speedCount[3]*speedCount[3])+(speedCount[4]*speedCount[4]) + (speedCount[5]*speedCount[5])
			+ (speedCount[6]*speedCount[6]) ;
		vectorSpeed = std::sqrt(vectorSpeed);

		std::stringstream ss;
		ss << "LI " << posCount[0] << "," << posCount[1] << "," << posCount[2] << "," << posCount[3] << "," << posCount[4] << "," 
			<<  posCount[5] << "," << posCount[6] ;
		//add speed to the command string 
		ss << "<" << (int)vectorSpeed << ">" << (int)vectorSpeed;
		string command = ss.str();
		controller.command(command);
	}
	else{
		cerr << "addLinearMotionSegment takes 7 joint angles and speeds" << endl;
		throw std::out_of_range ("Angles, Speeds incorrect vector size. Should be 7");
	}
	return 1;
}

bool MotorController::beginLI()
{
	//controller.command("LE"); // Linear End, for smooth stopping
	controller.command("BGS"); // Begin Sequence
	return true;
}

bool MotorController::clearLI()
{
	//controller.command("LE"); // Linear End, for smooth stopping
	controller.command("CS"); // Begin Sequence
	return true;
}

bool MotorController::endLIseq()
{
	controller.command("LE"); // Linear End, for smooth stopping
	//controller.command("BGS"); // Begin Sequence
	return true;
}

bool MotorController::isInitialized() // return initialized
{
	return initialized;
}

bool MotorController::isSimulated() // return simulation flag from controller
{
	return controller.isSimulated();
}

bool MotorController::isDebug() // return simulation flag from controller
{
	return controller.isDebug();
}

bool MotorController::wmraSetup() //WMRA setup
{
	/*update current position vector array size*/
	curPosition.resize(8);

	/*set all motors to designated motor type*/
	setBrushedMotors();

	/*set all 7 arm joints to LI mode, and gripper to PT mode*/
	setMotorMode(motorMode);

	/*set all velocities with default values*/
	setMaxVelocity(0, motorVelocity[0]);
	setMaxVelocity(1, motorVelocity[1]);
	setMaxVelocity(2, motorVelocity[2]);
	setMaxVelocity(3, motorVelocity[3]);
	setMaxVelocity(4, motorVelocity[4]);
	setMaxVelocity(5, motorVelocity[5]);
	setMaxVelocity(6, motorVelocity[6]);
	setMaxVelocity(7, motorVelocity[7]);

	/*set accelaration values from defaults*/
	setAccel(0, motorAccel[0]);
	setAccel(1, motorAccel[1]);
	setAccel(2, motorAccel[2]);
	setAccel(3, motorAccel[3]);
	setAccel(4, motorAccel[4]);
	setAccel(5, motorAccel[5]);
	setAccel(6, motorAccel[6]);
	setAccel(7, motorAccel[7]);

	/*set deceleration values from defaults*/
	setDecel(0, motorDecel[0]);
	setDecel(1, motorDecel[1]);
	setDecel(2, motorDecel[2]);
	setDecel(3, motorDecel[3]);
	setDecel(4, motorDecel[4]);
	setDecel(5, motorDecel[5]);
	setDecel(6, motorDecel[6]);
	setDecel(7, motorDecel[7]);

	/*set accelaration smoothing*/
	setSmoothing(smoothingVal);

	/*set default ready position*/
	definePosition(0, readyPosition[0]);
	definePosition(1, readyPosition[1]);
	definePosition(2, readyPosition[2]);
	definePosition(3, readyPosition[3]);
	definePosition(4, readyPosition[4]);
	definePosition(5, readyPosition[5]);
	definePosition(6, readyPosition[6]);
	definePosition(7, readyPosition[7]);

	for(int i = 0 ; i<8; i++)
		curPosition[i] = readyPosition[i];

	/*turn motors on*/
	motorsOn();

	/*set initialization value*/
	initialized = true;

	return 1;
}

bool MotorController::Stop() //emergancy stop
{
	controller.command("ST ABCDEFGH");
	return true; // #debug does this return need to happen after the Arm has fully stopped?
}

bool MotorController::Stop(int motorNum) // emergancy stop a single motor
{
	if(isValidMotor(motorNum)){
		string motor = motorLookup[motorNum];
		controller.command("ST " + motor);
		return true;
	}
	else{
		return false;
	}
}

bool MotorController::motionFinished() {
	string result = controller.command("LM?");
	int availableSpaces = 0;
	sscanf(result.c_str(), "%d", &availableSpaces);
	if (availableSpaces == 511) {
		return true;
	} else {
		return false;
	}
}

double MotorController::readPos(int motorNum) // returns the current motor angle in radians
{
	long encoderVal;	
	string result;
	string motor;
	if ( isValidMotor(motorNum)){
		if(controller.isSimulated())
		{
			return curPosition[motorNum];
		}
		motor = motorLookup[motorNum];
		result = controller.command( "TP" + motor);	
		istringstream stream(result);
		stream >> encoderVal;
		return encToAng(motorNum, encoderVal);        
	}
	else{
		if(motorNum != 8 && motorNum != 9){
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

std::vector<double> MotorController::readPosAll()
{
	if(controller.isSimulated())
	{
			return curPosition;
	}
	vector<int> pos(8);
	vector<double> tgt(8);
	string result = controller.command( "TP");	
	sscanf(result.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d", &pos[0],&pos[1],&pos[2],&pos[3],&pos[4],&pos[5],&pos[6],&pos[7]);
	for(int i = 0; i<8; i++)
	{
		tgt[i] = encToAng(i, pos[i]); 
	}
	lastKnownPos = tgt;
	return tgt;       
}

std::vector<int> MotorController::readPosAll_raw()
{
	vector<int> pos(8);
	string result = controller.command( "TP");	
	sscanf(result.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d", &pos[0],&pos[1],&pos[2],&pos[3],&pos[4],&pos[5],&pos[6],&pos[7]);

	return pos;       
}

std::vector<double> MotorController::getLastKnownPos(){
	return lastKnownPos;
}

double MotorController::readPosErr(int motorNum) // returns the error in  
{

	long encoderVal;	
	string result;
	string motor;
	if (isValidMotor(motorNum)){
		motor = motorLookup[motorNum];
		result = controller.command( "TE" + motor);	
		istringstream stream(result);
		stream >> encoderVal;
		return encToAng(motorNum, encoderVal);        
	}
	else{
		if(motorNum != 8 && motorNum != 9)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::setMaxVelocity(int motorNum, double angularVelocity)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularVelocity));
		string motor = motorLookup[motorNum];
		if ((encVal >= 0) && (encVal < 12000000)){
			string command = "SP" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The velocity is outside the range" << endl;
			throw std::out_of_range ("velocity out_of_range");            
		}
	}
	else{
		if(motorNum != 8 && motorNum != 9)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::setAccel(int motorNum, double angularAccelaration)
{	
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularAccelaration));
		string motor = motorLookup[motorNum];
		if ((encVal >= 1024) && (encVal <= 67107840)){
			string command = "AC" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Accelaration is outside the range" << endl;
			throw std::out_of_range ("Accelaration out_of_range");            
		}
	}
	else{
		if(motorNum != 8 && motorNum != 9)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::setDecel(int motorNum, double angularDecelaration)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularDecelaration));
		string motor = motorLookup[motorNum];
		if((encVal >= 1024) && (encVal <= 67107840)){
			string command = "DC" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Decelaration is outside the range" << endl;
			throw std::out_of_range ("Decelaration out_of_range");            
		}
	}
	else{
		if(motorNum != 8 && motorNum != 9)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::definePosition(int motorNum,double angle)
{
	if(isValidMotor(motorNum)){
		if(controller.isSimulated())
		{
			curPosition[motorNum] = angle;
		}
		long encVal = angToEnc(motorNum,angle);
		string motor = motorLookup[motorNum];
		if (true){ //#debug  check if angle is in range
			string command = "DP" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Position is outside the range" << endl;
			throw std::out_of_range ("Position out_of_range");            
		}
	}
	else{
		if(motorNum != 8 && motorNum != 9)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::positionControl(int motorNum,double angle)
{
	if(isValidMotor(motorNum)){
		if(controller.isSimulated())
		{
			curPosition[motorNum] = angle;
		}

		long encVal = (angToEnc(motorNum,angle));
		string motor;
		motor = motorLookup[motorNum];
		long temp = abs(encVal);
		if (temp <= 2147483648){
			string command = "PA" + motor + "=" + toString(encVal);
			try	{
				controller.command(command);
			}
			catch(string s){
				cout << s << endl;
				cout << endl;
			}
			return true;            
		}
		else{
			cerr << "The Position is outside the range" << endl;
			throw std::out_of_range ("Position out_of_range");            
		}
	}
	else{
		if(motorNum != 8 && motorNum != 9)
		{
			cerr << "The motor specified is not valid" << endl;
			throw std::out_of_range ("MotorNum out_of_range");
		}
	}
}

bool MotorController::MotorsOFF()
{
	controller.command("MO"); //turn off motors
	return 1;
}

bool MotorController::motorsOn()
{
	controller.command("SH"); //turn on motors
	return true;
}

/*------------------------------------------------------

Private Functions

------------------------------------------------------*/

inline bool MotorController::isValidMotor(int motorNum){
	if( motorNum >= 0 && motorNum <= 8) return true;
	else return false;
}

double MotorController::encToAng(int motorNum, long encCount) // #debug needs to be finished, Also need to check initialized
{
	if (isValidMotor(motorNum)){
		return encCount * enc2Radian[motorNum]; 
	}
	else{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

long MotorController::angToEnc(int motorNum, double encCount) // #debug needs to be finished, Also need to check initialized
{
	if (isValidMotor(motorNum)){
		return encCount * radian2Enc[motorNum]; 
	}
	else{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

bool MotorController::setSmoothing(double value)
{
	string command1 = "IT*=";
	std::string motor;
	ostringstream buffer1;
	buffer1 << value;
	motor = buffer1.str();
	command1 = command1 + motor;
	controller.command(command1);

	return 1;
}

bool MotorController::setBrushedMode(int motorNum, int mode)
{
	string command1 = "BR";
	if(isValidMotor(motorNum)){
		std::string motor;
		motor = motorLookup[motorNum];
		command1 = command1 + motor;
		command1 = command1 + "=";
		ostringstream buffer1;
		buffer1 << mode;
		motor = buffer1.str();
		command1 = command1 + motor;

		controller.command(command1);
	}
	else
		return 0;
	return 1;
}

bool MotorController::setBrushedMotors()
{
	// set motor types, as in brushed or brushless DC motor.
	setBrushedMode(0, brushedMotors[0]);
	setBrushedMode(1, brushedMotors[1]);
	setBrushedMode(2, brushedMotors[2]);
	setBrushedMode(3, brushedMotors[3]);
	setBrushedMode(4, brushedMotors[4]);
	setBrushedMode(5, brushedMotors[5]);
	setBrushedMode(6, brushedMotors[6]);
	setBrushedMode(7, brushedMotors[7]);
	return 1;
}

bool MotorController::sendJog(vector<int> value)
{
	ostringstream os;
	os << value[0] << "," << value[1] << "," << value[2] << "," << value[3] << "," << value[4] << "," << value[5] << "," << value[6] << "," << value[7];
	string str="JG"+os.str();
	controller.command(str);
	return 1;
}

bool MotorController::setDefaults()
{
	ConfigReader reader;
	reader.parseFile("settings_controller.conf");
	reader.setSection("MOTOR_CONTROLLER_DEFAULTS");

	if(reader.keyPresent("encoderPerRevolution1")){			
		enc2Radian[0] = 2*M_PI/reader.getInt("encoderPerRevolution1"); //calculate conversion values
		radian2Enc[0] = 1/enc2Radian[0];
	}
	else{
		cout << "'encoderPerRevolution1' default not found" << endl;			
		return 0;
	}
	if(reader.keyPresent("encoderPerRevolution2")){
		enc2Radian[1] = 2*M_PI/reader.getInt("encoderPerRevolution2"); //calculate conversion values
		radian2Enc[1] = 1/enc2Radian[1];
	}
	else{
		cout << "'encoderPerRevolution2' default not found" << endl;			
		return 0;
	}
	if(reader.keyPresent("encoderPerRevolution3"))
	{
		enc2Radian[2] = 2*M_PI/reader.getInt("encoderPerRevolution3"); //calculate conversion values
		radian2Enc[2] = 1/enc2Radian[2];
	}
	else
	{
		cout << "'encoderPerRevolution3' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("encoderPerRevolution4"))
	{
		enc2Radian[3] = 2*M_PI/reader.getInt("encoderPerRevolution4"); //calculate conversion values
		radian2Enc[3] = 1/enc2Radian[3];
	}
	else
	{
		cout << "'encoderPerRevolution4' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("encoderPerRevolution5"))
	{
		enc2Radian[4] = 2*M_PI/reader.getInt("encoderPerRevolution5"); //calculate conversion values
		radian2Enc[4] = 1/enc2Radian[4];
	}
	else
	{
		cout << "'encoderPerRevolution5' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("encoderPerRevolution6"))
	{
		enc2Radian[5] = -2*M_PI/reader.getInt("encoderPerRevolution6"); //calculate conversion values
		radian2Enc[5] = 1/enc2Radian[5];
	}
	else
	{
		cout << "'encoderPerRevolution6' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("encoderPerRevolution7"))
	{
		enc2Radian[6] = 2*M_PI/reader.getInt("encoderPerRevolution7"); //calculate conversion values
		radian2Enc[6] = 1/enc2Radian[6];
	}
	else
	{
		cout << "'encoderPerRevolution7' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("encoderPerRevolution8")) //#Debug: encoderPerRevolution8 value is incorrect
	{
		enc2Radian[7] = 2*M_PI/reader.getInt("encoderPerRevolution8"); //calculate conversion values
		radian2Enc[7] = 1/enc2Radian[7];
	}
	else
	{
		cout << "'encoderPerRevolution8' default not found" << endl;
		return 0;
	}

	// set motor type to brushed motor
	if(reader.keyPresent("brushedMotor1"))
		brushedMotors[0] = reader.getInt("brushedMotor1");
	else
	{
		cout << "'brushedMotor1' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("brushedMotor2"))
		brushedMotors[1] = reader.getInt("brushedMotor2"); 
	else
	{
		cout << "'brushedMotor2' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("brushedMotor3"))
		brushedMotors[2] = reader.getInt("brushedMotor3");
	else
	{
		cout << "'brushedMotor3' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("brushedMotor4"))
		brushedMotors[3] = reader.getInt("brushedMotor4"); 
	else
	{
		cout << "'brushedMotor4' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("brushedMotor5"))
		brushedMotors[4] = reader.getInt("brushedMotor5"); 
	else
	{
		cout << "'brushedMotor5' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("brushedMotor6"))
		brushedMotors[5] = reader.getInt("brushedMotor6"); 
	else
	{
		cout << "'brushedMotor6' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("brushedMotor7"))
		brushedMotors[6] = reader.getInt("brushedMotor7"); 
	else
	{
		cout << "'brushedMotor7' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("brushedMotor8"))
		brushedMotors[7] = reader.getInt("brushedMotor8"); 
	else
	{
		cout << "'brushedMotor8' default not found" << endl;
		return 0;
	}

	//set all motors to position tracking mode
	if(reader.keyPresent("motorMode"))
		motorMode = LINEAR;  // DEBUG - Needs to read from settings file
	//motorMode = reader.getInt("motorMode");
	//motorMode = reader.getString("motorMode");
	else
	{
		cout << "'motorMode' default not found" << endl;
		return 0;
	}

	// Set Max Velocity
	if(reader.keyPresent("maxVelocity1"))
		motorVelocity[0] = reader.getDouble("maxVelocity1"); 
	else
	{
		cout << "'maxVelocity1' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("maxVelocity2"))
		motorVelocity[1] = reader.getDouble("maxVelocity2"); 
	else
	{
		cout << "'maxVelocity2' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("maxVelocity3"))
		motorVelocity[2] = reader.getDouble("maxVelocity3"); 
	else
	{
		cout << "'maxVelocity3' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("maxVelocity4"))
		motorVelocity[3] = reader.getDouble("maxVelocity4"); 
	else
	{
		cout << "'maxVelocity4' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("maxVelocity5"))
		motorVelocity[4] = reader.getDouble("maxVelocity5"); 
	else		
	{
		cout << "'maxVelocity5' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("maxVelocity6"))
		motorVelocity[5] = reader.getDouble("maxVelocity6"); 
	else
	{
		cout << "'maxVelocity6' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("maxVelocity7"))
		motorVelocity[6] = reader.getDouble("maxVelocity7"); 
	else
	{
		cout << "'maxVelocity7' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("maxVelocity8"))
		motorVelocity[7] = reader.getDouble("maxVelocity8"); 
	else
	{
		cout << "'maxVelocity8' default not found" << endl;
		return 0;
	}

	//Set Acceleration
	if(reader.keyPresent("acceleration1"))
		motorAccel[0] = reader.getDouble("acceleration1"); 
	else 
	{
		cout << "'acceleration1' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("acceleration2"))
		motorAccel[1] = reader.getDouble("acceleration2"); 
	else
	{
		cout << "'acceleration2' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("acceleration3"))
		motorAccel[2] = reader.getDouble("acceleration3"); 
	else
	{
		cout << "'acceleration3' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("acceleration4"))
		motorAccel[3] = reader.getDouble("acceleration4"); 
	else
	{
		cout << "'acceleration4' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("acceleration5"))
		motorAccel[4] = reader.getDouble("acceleration5"); 
	else		
	{
		cout << "'acceleration5' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("acceleration6"))
		motorAccel[5] = reader.getDouble("acceleration6"); 
	else
	{
		cout << "'acceleration6' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("acceleration7"))
		motorAccel[6] = reader.getDouble("acceleration7"); 
	else
	{
		cout << "'acceleration7' default not found" << endl;
		return 0;
	}

	if(reader.keyPresent("acceleration8"))
		motorAccel[7] = reader.getDouble("acceleration8"); 
	else
	{
		cout << "'acceleration8' default not found" << endl;
		return 0;
	}

	//Set Decceleration
	if(reader.keyPresent("deceleration1"))
		motorDecel[0] = reader.getDouble("deceleration1"); 
	else
	{
		cout << "'deceleration1' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("deceleration2"))
		motorDecel[1] = reader.getDouble("deceleration2"); 
	else
	{
		cout << "'deceleration2' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("deceleration3"))
		motorDecel[2] = reader.getDouble("deceleration3"); 
	else
	{
		cout << "'deceleration3' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("deceleration4"))
		motorDecel[3] = reader.getDouble("deceleration4"); 
	else
	{
		cout << "'deceleration4' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("deceleration5"))
		motorDecel[4] = reader.getDouble("deceleration5"); 
	else
	{
		cout << "'deceleration5' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("deceleration6"))
		motorDecel[5] = reader.getDouble("deceleration6"); 
	else
	{
		cout << "'deceleration6' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("deceleration7"))
		motorDecel[6] = reader.getDouble("deceleration7"); 
	else
	{
		cout << "'deceleration7' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("deceleration8"))
		motorDecel[7] = reader.getDouble("deceleration8"); 
	else
	{
		cout << "'deceleration8' default not found" << endl;
		return 0;
	}

	/*set accelaration smoothing from default*/ 
	if(reader.keyPresent("smoothing"))
		smoothingVal = reader.getDouble("smoothing");
	else
	{
		cout << "'smoothing' default not found" << endl;
		return 0;
	}

	/*set default ready position*/
	if(reader.keyPresent("readyPosition1"))
		readyPosition[0] = reader.getDouble("readyPosition1"); 
	else
	{
		cout << "'readyPosition1' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("readyPosition2"))
		readyPosition[1] = reader.getDouble("readyPosition2"); 
	else
	{
		cout << "'readyPosition2' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("readyPosition3"))
		readyPosition[2] = reader.getDouble("readyPosition3"); 
	else
	{
		cout << "'readyPosition3' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("readyPosition4"))
		readyPosition[3] = reader.getDouble("readyPosition4"); 
	else
	{
		cout << "'readyPosition4' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("readyPosition5"))
		readyPosition[4] = reader.getDouble("readyPosition5"); 
	else
	{
		cout << "'readyPosition5' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("readyPosition6"))
		readyPosition[5] = reader.getDouble("readyPosition6"); 
	else
	{
		cout << "'readyPosition6' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("readyPosition7"))
		readyPosition[6] = reader.getDouble("readyPosition7"); 
	else
	{
		cout << "'readyPosition7' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("readyPosition8"))
		readyPosition[7] = reader.getDouble("readyPosition8"); 
	else
	{
		cout << "'readyPosition8' default not found" << endl;
		return 0;
	}

	/*set default park position*/
	if(reader.keyPresent("parkPosition1"))
		parkPosition[0] = reader.getDouble("parkPosition1"); 
	else
	{
		cout << "'parkPosition1' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("parkPosition2"))
		parkPosition[1] = reader.getDouble("parkPosition2"); 
	else
	{
		cout << "'parkPosition2' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("parkPosition3"))
		parkPosition[2] = reader.getDouble("parkPosition3"); 
	else
	{
		cout << "'parkPosition3' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("parkPosition4"))
		parkPosition[3] = reader.getDouble("parkPosition4"); 
	else
	{
		cout << "'parkPosition4' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("parkPosition5"))
		parkPosition[4] = reader.getDouble("parkPosition5"); 
	else
	{
		cout << "'parkPosition5' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("parkPosition6"))
		parkPosition[5] = reader.getDouble("parkPosition6"); 
	else
	{
		cout << "'parkPosition6' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("parkPosition7"))
		parkPosition[6] = reader.getDouble("parkPosition7"); 
	else
	{
		cout << "'parkPosition7' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("parkPosition8"))
		parkPosition[7] = reader.getDouble("parkPosition8"); 
	else
	{
		cout << "'parkPosition8' default not found" << endl;
		return 0;
	}

	return 1;
}
