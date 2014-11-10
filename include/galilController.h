#ifndef GALILCONTROLLER_H
#define GALILCONTROLLER_H

//#pragma once

#include <iostream>
#include <string>
#include <fstream>



class client_tcpsocket;

class galilController {
public:
	galilController(); // sets initialized to false
	//~galilController(); // closes socket communication
	bool initialize(); // Initializes socket communication
	bool isInitialized(); // return initialized
	bool isSimulated(); // return simulation
	bool isDebug(); // return debug
	std::string command(std::string Command); // user command structure, used by MotorController

private:
	bool debug;
	std::string IP; // IP address of the Galil Controller
	bool initialized; // Flag that is set when socket communication has been achieved
	bool simulation; // Flag that is set when movement data needs to be output to visualization program
	bool setDefaults(); // set defaults
	bool initializeSocket(std::string IP); // sets up socket communication, sets initialized
	int commandGalil(char* Command, char* Response, int ResponseSize); // Galil Controller command structure, used by command()
	static client_tcpsocket sock; // The socket class used to communicate with galil controller
	std::ofstream debugFile;
};

#endif