#include "SocketControl.h"
#include "SockStream.h"
#include "Utility.h"
#include "Arm.h"
#include "tinythread.h"

//using namespace tthread;

//Constructor
SocketControl::SocketControl(Arm* robot)
{
	robotArm = robot;
	t = new tthread::thread(socketListenReply,this);
	intermediateWaitingPose = WMRA::Pose(400, -150, 480, 0, 0, 0);
	userPose = WMRA::Pose(-100, -300, 500, 0, 0, 0);
	userPose.yaw = degToRad(-90)
	;waitingPose = WMRA::Pose(50, 200, 600, 0, 0, 0);
	//intermediateWaitingPose = WMRA::Pose(200, 100, 550,-10, 0, 0);
}

//Destructor
SocketControl::~SocketControl(void)
{
}

void SocketControl::socketListenReply(void * aArg) {
	SocketControl* self = (SocketControl*)aArg ;
	receiving_udpsocket socket1( "0.0.0.0:7500" );
	sockstream read_sock( socket1 );
	sending_udpsocket socket2( "localhost:7501" );
	sockstream output_sock( socket2 );

	if(!read_sock.is_open()) {
		cerr << "Could not open read socket for reading." << endl;
	}

	cout << ">> Socket Controller thread started " << endl;
	string temp_str_buf;
	char temp_buf[200];
	while(true) {
		do{
			Sleep(20);
			getline( read_sock, temp_str_buf );
		} while(temp_str_buf.find("COMMAND") == string::npos); // keep reading until message is received
		cout << "\n received command: " << string(temp_str_buf) << endl;
		self->selectAction(temp_str_buf);// selcts the action and calls appropriate function

		//graspObject(graspPose);

		output_sock << "DONE" << endl;
		//output_sock << "POSITION " << curPose.x << " " << curPose.y << " " << curPose.z << endl; 
	}
}

string SocketControl::selectAction(string cmd) {
	//
	cmd.erase(0,8); //erase "COMMAND " from the begining
	if(cmd.find("PICK_UP") != string::npos) {
		pickupObject(cmd);
	} else if(cmd.find("TRASH")!= string::npos) {
		trashObject(cmd);
	} else if(cmd.find("POUR")!= string::npos) {
		pourObject(cmd);
	} else if(cmd.find("BRING_TO_USER")!= string::npos) {
		bringObject(cmd);
	} else if(cmd.find("CAMERA_VIEW_CLOSE")!= string::npos) {
		cameraViewGripper(cmd);
	} else if(cmd.find("MOVE_ARM_TO")!= string::npos) {
		moveArmTo(cmd);
	} else if(cmd.find("GO_TO_WAITING")!= string::npos) {
		goToWaiting(cmd);
	}

	return "DONE";
}

//Move Arm To
bool SocketControl::goToWaiting(string cmd){

	cout << "moving arm to intermediateWaitingPose ...."  << endl;
	robotArm->autonomous(intermediateWaitingPose, WMRA::ARM_FRAME_PILOT_MODE, true); // Move to object location

	cout << "moving arm to waiting pose..." << endl;
	robotArm->autonomous(waitingPose, WMRA::ARM_FRAME_PILOT_MODE, true); // Move to object location

	Sleep(2000);

	return true;
}

//Move Arm To
bool SocketControl::moveArmTo(string cmd){
	double objPos[3] = {0};
	char temp_buf[200];
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf", temp_buf,
		&objPos[0], &objPos[1], &objPos[2]);

	WMRA::Pose objectPose;
	objectPose.clear();
	objectPose.x = objPos[0];
	objectPose.y = objPos[1];
	objectPose.z = objPos[2];

	cout << "moving arm to " << objPos[0] << "," << objPos[1] <<"," << objPos[2] << endl;
	robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE, true); // Move to object location

	return true;
}

//Pickup Object [DONE]
bool SocketControl::pickupObject(string cmd) // This function assumes orientation to be 0,0,0
{
	double objPos[3] = {0};
	char temp_buf[200];
	//cmd.erase(0, 8);
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf", temp_buf,
		&objPos[0], &objPos[1], &objPos[2]);

	if (numRead == 4) {
		//Create Pose for Object Position
		WMRA::Pose objectPose;
		objectPose.clear();
		objectPose.x = objPos[0];
		objectPose.y = objPos[1];
		objectPose.z = objPos[2];

		WMRA::Pose prePose = objectPose;
		prePose.x = prePose.x - 100.0;

		WMRA::Pose liftTable = objectPose;
		liftTable.z = liftTable.z + 100;

/*		cout << "going to intermediate pose" << endl;
		robotArm->autonomous(intermediateWaitingPose, WMRA::ARM_FRAME_PILOT_MODE);*/ // Move to pre-pose
		//Sleep(2000);

		cout << "going to prepose" << endl;
		robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
		//Sleep(2000);

		cout << "Opening gripper " << endl;
		robotArm->openGripper();
		//Sleep(5000);

		cout << "Going to object Pose" << endl;
		robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
		//Sleep(2000);

		cout << "Closing Gripper" << endl;
		robotArm->closeGripper();
		Sleep(3000);

		//objectPose.z = objectPose.z + 100.0; // Raising object
		cout << "Raising Object" << endl;
		robotArm->autonomous(liftTable, WMRA::ARM_FRAME_PILOT_MODE); // Raising object
		//Sleep(2000);

		cout << "Going to object Pose" << endl;
		robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to objectPose
		//Sleep(2000);

		cout << "Opening gripper " << endl;
		robotArm->openGripper();
		Sleep(2000);

		cout << "Going back to pre Pose" << endl;
		robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre Pose
		//Sleep(2000);

		cout << "Going back to waiting Pose" << endl;
		robotArm->autonomous(waitingPose, WMRA::ARM_FRAME_PILOT_MODE); //Return to Waiting Pose
		//Sleep(2000);

		return true;
	} else {
		return false;
	}
}

//Trash Object [DONE]
bool SocketControl::trashObject(string cmd) {
	double objPos[3] = {0};
	double objRot[3] = {0};
	double trashPos[3] = {0};
	char temp_buf[200];
	//cmd.erase(0, 6);
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		temp_buf,
		&objPos[0], &objPos[1], &objPos[2],
		&objRot[0], &objRot[1], &objRot[2], 
		&trashPos[0], &trashPos[1], &trashPos[2]);

	if (numRead == 10) {
		//Create Pose for Object Position
		WMRA::Pose objectPose;
		objectPose.clear();
		objectPose.x = objPos[0];
		objectPose.y = objPos[1];
		objectPose.z = objPos[2];
		objectPose.roll = objRot[0];
		objectPose.pitch = objRot[1];
		objectPose.yaw = objRot[2];

		WMRA::Pose prePose = objectPose;
		prePose.x = prePose.x - 100.0;

		WMRA::Pose liftPose = objectPose;
		liftPose.z = liftPose.z + 100.0;

		//Create Pose for Trash Position
		WMRA::Pose trashPose;
		trashPose.clear();
		trashPose.x = trashPos[0];
		trashPose.y = trashPos[1];
		trashPose.z = trashPos[2];

		//cout << "going to intermediate pose" << endl;
		//robotArm->autonomous(intermediateWaitingPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
		////Sleep(2000);

		cout << "going to prepose" << endl;
		robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
		//Sleep(2000);

		cout << "Opening gripper " << endl;
		robotArm->openGripper();
		//Sleep(5000);

		cout << "Going to object Pose" << endl;
		robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
		//Sleep(2000);

		cout << "Closing Gripper" << endl;
		robotArm->closeGripper();
		Sleep(3000);

		cout << "Going to lift Pose" << endl;
		robotArm->autonomous(liftPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to trash location
		//Sleep(2000);

		cout << "Going to trash Pose" << endl;
		robotArm->autonomous(trashPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to trash location
		//Sleep(2000);

		cout << "Opening gripper " << endl;
		robotArm->openGripper();
		//Sleep(5000);

		cout << "Going back to waiting Pose" << endl;
		robotArm->autonomous(waitingPose, WMRA::ARM_FRAME_PILOT_MODE); //Return to Waiting Pose
		//Sleep(2000);

		return true;
	} else {
		return false;
	}
}

bool SocketControl::pourObject(string cmd) {
	double objPos[3] = {0};
	double objRot[3] = {0};
	double objHeight = 0;
	double destPos[3] = {0};
	double destRot[3] = {0};
	cmd.erase(0, 5);
	int numRead = sscanf(cmd.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
		&objPos[0], &objPos[1], &objPos[2],
		&objRot[0], &objRot[1], &objRot[2], &objHeight, 
		&destPos[0], &destPos[1], &destPos[2],
		&destRot[0], &destRot[1], &destRot[2]);

	return true;
}

bool SocketControl::graspObject(WMRA::Pose objectPose){
	//assume arm is in camera view pose
	WMRA::Pose prePose = objectPose;
	prePose.x = prePose.x - 100.0;

	cout << "going to prepose" << endl;
	robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
	//Sleep(2000);

	cout << "Opening gripper " << endl;
	robotArm->openGripper();
	//Sleep(5000);

	cout << "Going to object Pose" << endl;
	robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
	//Sleep(2000);

	cout << "Closing Gripper" << endl;
	robotArm->closeGripper();


	WMRA::Pose liftPose = objectPose;
	liftPose.z = liftPose.z + 100.0;

	cout << "Going to lift Pose" << endl;
	robotArm->autonomous(liftPose, WMRA::ARM_FRAME_PILOT_MODE); 

	return true;
}

bool SocketControl::releaseGrasp(WMRA::Pose objectPose){

	//
	WMRA::Pose prePose = objectPose;
	prePose.x = prePose.x - 100.0;

	WMRA::Pose liftPose = objectPose;
	liftPose.z = liftPose.z + 100.0;

	cout << "Going to lift Pose" << endl;
	robotArm->autonomous(liftPose, WMRA::ARM_FRAME_PILOT_MODE); 
	cout << "Going to object Pose" << endl;
	robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE);
	cout << "Opening gripper " << endl;
	robotArm->openGripper();
	cout << "going to prepose" << endl;
	robotArm->autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
	return true;
}

bool SocketControl::bringObject(string cmd) {
	double objPos[3] = {0};
	double objRot[3] = {0};
	//cmd.erase(0, 14);
	char temp_buf[200];
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf %lf %lf %lf", 
		temp_buf,
		&objPos[0], &objPos[1], &objPos[2],
		&objRot[0], &objRot[1], &objRot[2]);

	if (numRead == 7) {
		//Create Pose for Object Position
		WMRA::Pose objectPose;
		objectPose.clear();
		objectPose.x = objPos[0];
		objectPose.y = objPos[1];
		objectPose.z = objPos[2];
		objectPose.roll = objRot[0];
		objectPose.pitch = objRot[1];
		objectPose.yaw = objRot[2];


		


		graspObject(objectPose);


	    cout << "going to intermediate pose" << endl;
		robotArm->autonomous(intermediateWaitingPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose

		WMRA::Pose waypoint1(0,-350,500,0,0,0);
		waypoint1.yaw = degToRad(-90);
		cout << "Going to way point 1" << endl;
		robotArm->autonomous(waypoint1, WMRA::ARM_FRAME_PILOT_MODE);

		cout << "Going to user Pose" << endl;
		robotArm->autonomous(userPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to user location

		//tilt gripper
		WMRA::Pose gripper_tilt;
		gripper_tilt.yaw = degToRad(30);
		
		//putting it back
		cout << ">> Putting back.." << endl;
		cout << "Going to way point 1" << endl;
		robotArm->autonomous(waypoint1, WMRA::ARM_FRAME_PILOT_MODE);

		cout << "going to intermediate pose" << endl;
		robotArm->autonomous(intermediateWaitingPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
	    
		releaseGrasp(objectPose); // opposite of grasp object
		

		cout << "Going back to waiting Pose" << endl;
		robotArm->autonomous(waitingPose, WMRA::ARM_FRAME_PILOT_MODE); //Return to Waiting Pose
		//Sleep(2000);

		return true;
	} else {
		return false;
	}
}

bool SocketControl::cameraViewGripper(string cmd) {
	double objPose[6] = {0};
	char temp_buf[200];
	int numRead = sscanf(cmd.c_str(), "%s %lf %lf %lf %lf %lf %lf", temp_buf,
		&objPose[0], &objPose[1], &objPose[2],
		&objPose[3], &objPose[4], &objPose[5]);

	if(numRead == 7){
		//goto intermediate pose
		robotArm->autonomous(intermediateWaitingPose, WMRA::ARM_FRAME_PILOT_MODE, true);
		//calculate the destination position for the arm to get a better look at the object
		// minus x direction and look down towards the object from about 20 cm away.
		WMRA::Pose objectPose;
		objectPose.x = objPose[0] - 120 ; // minus 120mm in x direction ( x is forward dir of the wheelchair)
		objectPose.y = objPose[1]; 
		objectPose.z = objPose[2];
		objectPose.roll = objPose[3];
		objectPose.pitch = objPose[4];
		objectPose.yaw = objPose[5];

		//maybe rotate +10 in pitch
		
		cout << "Going to pose" << endl;
		robotArm->autonomous(objectPose, WMRA::ARM_FRAME_PILOT_MODE, true); // Move to object location
		return true;
	}
	else{
		return false;
	}

}