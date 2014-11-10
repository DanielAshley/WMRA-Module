
#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <limits>
#include "tinythread.h"
#include "matrix.h" 
#include "MotorController.h"
#include "Arm.h"
#include "Utility.h"
#include "SocketControl.h"
#include "SockStream.h"

#define PI 3.14159265
#undef max
using namespace std;
using namespace tthread;

Arm wmraArm;

bool getUserDest(WMRA::Pose& dest){
   //WMRA::Pose dest;
   double temp[6];
   char buf[200];
   string response;
   cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //clear input buffer
   cout << "\n\nEnter destination (x y z r p y): " << endl;
   cin.getline(buf,200);

   int num = sscanf(buf,"%lf %lf %lf %lf %lf %lf", 
	   &temp[0],&temp[1], &temp[2], &temp[3], &temp[4], &temp[5]);
   if(num == 6){
	   dest.x = temp[0];
	   dest.y = temp[1];
	   dest.z = temp[2];
	   dest.roll = degToRad(temp[3]);
	   dest.pitch = degToRad(temp[4]);
	   dest.yaw = degToRad(temp[5]);
	   return true;
   }
   else{
	   cout << "Invalid input" << endl;
	   return false;
   }
   
}


void socketComm(void * aArg){

   receiving_udpsocket socket1( "localhost:7000" );
   sockstream read_sock( socket1 );
   sending_udpsocket socket2( "localhost:7001" );
   sockstream output_sock( socket2 );
   WMRA::Pose curPose;

   if( !read_sock.is_open() ){
      cerr << "Could not open read socket for reading." << endl;
   }

   cout << "hey thread started " << endl;
   string temp_str_buf;
   char temp_buf[200];
   while(true){
      do{
         getline( read_sock, temp_str_buf );
      } while( temp_str_buf.find("GETPOS")== string::npos ); // keep reading until message is received
      //send position
      curPose = wmraArm.getPose();
      output_sock << "POSITION " << curPose.x << " " << curPose.y << " " << curPose.z << endl; 
   }
}



bool graspObject(WMRA::Pose objectPose) // This function assumes orientation to be 0,0,0
{
   WMRA::Pose prePose = objectPose;
   prePose.x = prePose.x-100.0;
   prePose.z = prePose.z+100.0; // Prepose will always be higher than grasping position

   WMRA::Pose graspPose = objectPose;
   graspPose.x = graspPose.x + 50;

   WMRA::Pose liftTable = graspPose;
   liftTable.z = liftTable.z +100;

   cout << "going to prepose" <<endl;
   wmraArm.autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
   Sleep(2000);
   cout << "Opening gripper " << endl;
   wmraArm.openGripper();
   Sleep(5000);
   cout << "Going to grasp pose" << endl;
   wmraArm.autonomous(graspPose, WMRA::ARM_FRAME_PILOT_MODE); // Move to object location
   Sleep(2000);
   cout << "Closing Gripper" <<endl;
   wmraArm.closeGripper();
   Sleep(5000);

   //objectPose.z = objectPose.z + 100.0; // Raising object
   cout << "Raising Object" << endl;
   wmraArm.autonomous(liftTable, WMRA::ARM_FRAME_PILOT_MODE); // Raising object
   Sleep(10000);

   cout << "Going to prepose" << endl;
   wmraArm.autonomous(prePose, WMRA::ARM_FRAME_PILOT_MODE); // Move to pre-pose
   Sleep(10000);

   return true;
}

/*
* Allows to control the arm through a socket connection.
* Parses GOTOPOSE x y z message and grasps a object at that pose
*/
void socketControl(void * aArg){

   receiving_udpsocket socket1( "localhost:7000" );
   sockstream read_sock( socket1 );
   sending_udpsocket socket2( "localhost:7001" );
   sockstream output_sock( socket2 );
   WMRA::Pose curPose = wmraArm.getPose();

   if( !read_sock.is_open() ){
      cerr << "Could not open read socket for reading." << endl;
   }

   cout << "hey thread started " << endl;
   string temp_str_buf;
   char temp_buf[200];
   while(true){
      do{
         getline( read_sock, temp_str_buf );
      } while( temp_str_buf.find("PICKUP")== string::npos ); // keep reading until message is received
      WMRA::Pose graspPose;
      graspPose.clear();
      double pos[3] = {0};
      int numRead = sscanf(temp_str_buf.c_str(), "%s %lf %lf %lf", temp_buf, &pos[0], &pos[1], &pos[2]);

      graspPose.x = pos[0];
      graspPose.y = pos[1];
      graspPose.z = pos[2];
      graspObject(graspPose);


      output_sock << "DONE" << endl;
      output_sock << "POSITION " << curPose.x << " " << curPose.y << " " << curPose.z << endl; 
   }
}

void continuousSquare(WMRA::Pose curPos)
{
   //Request to begin test
   int length;
   cout << "Length? " << endl;
   cin >> length;

   int loopCount;
   cout << "Number of loops? " << endl;
   cin >> loopCount;

   int choice = 0;
   cout << "Begin Square Test? 1=Yes 0=No" << endl;
   cin >> choice;

   WMRA::Pose dest1,dest2,dest3,dest4;
   dest1 = dest2 = dest3 = dest4 = curPos;	

   dest2.x = dest1.x;
   dest2.y = dest1.y+length;

   dest3.x = dest2.x+length;
   dest3.y = dest2.y;

   dest4.x = dest3.x;
   dest4.y = dest3.y-length;

   // Move arm to starting position (dest1)
   wmraArm.autonomous(dest1, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
   Sleep(6000); // wait for motion end

   int delay = (int)(length/50 * 1000) + 20 ;

   while(loopCount > 0 && choice==1)
   {
      cout << "Loop: " << loopCount << endl;
      wmraArm.autonomous(dest2, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
      Sleep(delay); // wait for motion end
      wmraArm.autonomous(dest3, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
      Sleep(delay); // wait for motion end
      wmraArm.autonomous(dest4, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
      Sleep(delay); // wait for motion end
      wmraArm.autonomous(dest1, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
      Sleep(delay); // wait for motion end
      loopCount--;
   }
}

bool moveJoint()
{
   int choice, jointNum;
   double angle, angleRadians;

   cin.clear();
   cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //clear input buffer
   cout << "Which Joint? ";
   cin >> jointNum;
   jointNum--;

   cin.clear();
   cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //clear input buffer
   cout << "abs=0, rel=1? ";
   cin >> choice;

   cin.clear();
   cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //clear input buffer
   cout << "Angle? ";
   cin >> angle;
   angleRadians = degToRad(angle);

   wmraArm.moveJoint(jointNum, angleRadians, choice);
   return 1;
}

bool saveJointAngles(WMRA::JointValueSet &jAng){
   ofstream jointFile;
   jointFile.open ("lastJointPositions.txt");
   if (!jointFile.is_open()){
      cerr << "Error opening input file" << endl;
      return false;
   }
   //get joint angles
   //WMRA::JointValueSet jAng=  wmraArm.getJointAngles();
   //write to stream
   jointFile << jAng[0] << "," << jAng[1] << "," << jAng[2] << "," << jAng[3] << "," ;
   jointFile << jAng[4] << "," << jAng[5] << "," << jAng[6] << endl;
   jointFile.close();
   return true;
}

bool readJointAnglesFromFiles(WMRA::JointValueSet &angles){
   ifstream jointFile;
   jointFile.open ("lastJointPositions.txt");
   if (!jointFile.is_open()){
      cerr << "Error opening input file" << endl;
      return false;
   }
   double t[7] = {0};//temp 
   std::string temp_in;
   getline(jointFile, temp_in);
   int num = sscanf(temp_in.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
      &t[0],&t[1],&t[2],&t[3],&t[4],&t[5],&t[6] );

   jointFile.close(); // close file
   if(num ==7){ //if all 7 joints angles are parsed correctly
      for(int i=0; i < 7 ;++i){
         angles[i] = t[i];  //write to JointValueSet
      }
      return true;
   }
   else return false;
}

bool gotoReadyPosition(Arm& arm, WMRA::Pose& ready){
	arm.autonomous(ready, WMRA::ARM_FRAME_PILOT_MODE);
	cout << "Would you like to set joint angles to 90 90 0 90 90 60 0 ? ( yes = 1 no = 2) :";
	int temp = -1;
	cin >> temp;
	if(temp == 1){
		arm.toReady(true);
	}
	return true;	
}

int main()
{
   int velocity;				// Max velocity of the gripper in movement
   bool endFlag = false;
   if(wmraArm.initialize()){
      try{

         //thread t(socketControl, 0); // start the communication thread
         //cout << "Controller intialized in main" << endl;
        
		 SocketControl BCIrobotControler(&wmraArm);

         int cordframe;
         double temp;
         int option;

         //test save and read joint angle functions.
       /*  WMRA::JointValueSet j = wmraArm.getJointAngles();
         cout << "Current joint angles are: " << endl;;
         cout  << j.toString() << endl; 
         saveJointAngles(j);*/

		 WMRA::Pose readyPose = wmraArm.getPose();

		 
         //cin.clear();
		 //cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //clear input buffer
		 cout << "would you like to load Joint angles from file ? yes=1 no=0 :" ;
         cin >> option;
         WMRA::JointValueSet initialJointAngles;
         if(option ==1){         
            if( readJointAnglesFromFiles(initialJointAngles)){
               cout << ">> Joint angles read from file are : ";
               cout << initialJointAngles.toString() << endl;
               cout << ">> Seting joint angles on Arm... " << endl;
			   wmraArm.setInitialJointAngles(initialJointAngles);
            }
            else{
               cout << ">> Read Joint angle failed" << endl;
            }
         }          


         while(!endFlag){
            cout << "Current Position is :" << endl;
            WMRA::Pose pose = wmraArm.getPose();
            cout << " x = " << pose.x << " y = " << pose.y << " z = " << pose.z  << endl; 
            cout << " roll = " << radToDeg(pose.roll) << " pitch = " << radToDeg(pose.pitch) << " yaw = " << radToDeg(pose.yaw)  <<endl;

			cin.clear();
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //clear input buffer
            cout << "\nAvaialble options: \n 0 = Exit \n 1 = Move Arm \n 2 = Go to Ready \n 3 = ready to park ";
			cout << "\n 4 = park to ready \n 5 = save joint position \n 6 = move joint \n 8 = Grasp Object \n 9 = close gripper \n 10 = open gripper \n"; 
			cout << "Enter option : "; 
			cout.flush();
            cin >> option;

			if(option==1){
				WMRA::Pose dest;
				if(getUserDest(dest)){ 
					//cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
					cout << "Which cordinate frame? 1=ABS, 2=REL, 3=Gripper, 7=skip: "; 					
					cin >> cordframe; 
					if(cordframe==1){
						try{
							cout <<">> moving arm..." << endl;
							wmraArm.autonomous(dest, WMRA::ARM_FRAME_PILOT_MODE); // Moves arm
						}
						catch(...){
							cout << "haha" << endl;
						}
					} else if(cordframe==2){
						wmraArm.autonomous(dest, WMRA::ARM_FRAME_REL); // Moves arm
					} else if(cordframe==3){
						wmraArm.autonomous(dest, WMRA::GRIPPER_FRAME_REL); // Moves arm
					} else{
						cout << "skipping motion..." << endl;
						continue;
					}
				}
				//Sleep(10000); // wait for motion end
			}
            else if(option==2){
			   gotoReadyPosition(wmraArm, readyPose);
               
               //Sleep(10000);
            }
            else if(option==3){
               wmraArm.ready2Park();
            }
            else if(option==4){
               wmraArm.park2Ready();
            }
            else if(option==5){

				saveJointAngles(wmraArm.getLastKnownJointPosition());
				cout << ">> Joint values saved" << endl;
               //continuousSquare(wmraArm.getPose());
            }
            else if(option==6){
               moveJoint();
            }
            else if(option==8){
               WMRA::Pose dest;
			   if (getUserDest(dest) ){
					graspObject(dest);
			   }
            }
			 else if(option==9){
				 wmraArm.closeGripper(true); //block until done

            }
			else if(option==10){
				  wmraArm.openGripper(true); //block until done
            }
            else if(option== 0){
               wmraArm.closeDebug();
               endFlag = true;
            }
			else{
				cout <<  "Invalid option entered." << endl;
			}
            // save position at the end of every loop
			cout << "Saving Joint angles to file for recovery" << endl;
            saveJointAngles(wmraArm.getLastKnownJointPosition());

         } //end of while loop

         cout << "About to exit program. Would you like to go to ready position? 1=Yes 0=No : " ;
         cin >> option;
         if(option ==1){
            gotoReadyPosition(wmraArm, readyPose);
            Sleep(1000);
         }
         //saveJointAngles();
      }
      catch (exception& e){
         cerr << e.what() << endl;
         //try to save wmra joint angles
		 WMRA::JointValueSet j = wmraArm.getLastKnownJointPosition();
         cout << "Current joint angles are: " << endl;;
         cout  << j.toString() << endl; 
         saveJointAngles(j);
		 cin.get();
      }
   }
   else{
      cout << "Controller initalizaition failed in main" << endl;
   }

   cout << "Exiting Program..... Press any key to close this window." << endl;
   cin.get();
}