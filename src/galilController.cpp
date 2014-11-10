#include "galilController.h"
#include "SockStream.h"
#include "utility.h"
#include "ConfigReader.h"

using namespace std;

/************************
//
// Galil Controller Class
//
************************/

client_tcpsocket galilController::sock;

galilController::galilController(){

	debugFile.open("data/GalilDebug.log");
	initialized = false;
}

/*
galilController::~galilController(){
	debugFile.close();
	sock.~client_tcpsocket();
}*/

bool galilController::initialize() // return initialized
{
	//cout << "Galil: set defaults" << endl;
	initialized = setDefaults();
	if(initialized)
	{
		if(simulation)
		{
			cout << "Simulation Mode" << endl;
			initialized = true;
		}
		else
		{
			//cout << "Galil: initialize connection" << endl;
			initialized = initializeSocket(galilController::IP);
		}
	}
	if(!initialized)
	{
		cout << "Error: Failed to initialize Galil Controller" << endl;
		return 0;
	}
	else
	{
		if(simulation)
		{
			if(isDebug()) cout << "Simulation Initialized" << endl;
			return 1;
		}
		else
		{
			if(isDebug()) cout << "Galil: Initialized" << endl;
			return 1;
		}
	}
}

bool galilController::isInitialized() // return initialized
{
	return initialized;
}

bool galilController::isSimulated() // return simulation
{
	return simulation;
}
bool galilController::isDebug() // return simulation
{
	return debug;
}

std::string galilController::command(std::string Command)
{
	if(simulation)
	{
		//cout << "Sim Mode: " << Command <<  endl;
		return "0";
	}
	else
	{
		if(debug)
			debugFile << Command << endl;
		char com[300];
		char ret[300];
		std::string c = Command + "\r";
		strcpy(com, c.c_str());
		commandGalil(com, ret, sizeof(ret));
		std::string ret_str(ret);
		return ret_str;
	}
}

// PRIVATE FUNCTIONS
bool galilController::setDefaults()
{
	ConfigReader reader;
	reader.parseFile("settings_controller.conf");
	reader.setSection("GALIL_DEFAULTS");

	if(reader.keyPresent("IP"))
	{
		galilController::IP = reader.getString("IP");
		//cout << "IP: " << galilController::IP << endl;
	}
	else
	{
		cout << "'IP' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("simulation"))
	{
		simulation = reader.getInt("simulation");
	}
	else
	{
		cout << "'simulation' default not found" << endl;
		return 0;
	}
	if(reader.keyPresent("debug"))
	{
		debug = reader.getInt("debug");
	}
	else
	{
		cout << "'debug' default not found" << endl;
		return 0;
	}

	return 1;
}	

bool galilController::initializeSocket(std::string IP)
{
	//cout << "Initializing Socket" << endl;
	const char * c = IP.c_str();
	sock.open(c,23);
	//cout << "After Opening Socket" << endl;
	return galilController::sock.connected(); // #DEBUG - breaks if trying to setup sock while WMRA is off
	//return sock.is_open();
}

int galilController::commandGalil(char* Command, char* Response, int ResponseSize) //returns the number of bytes read
{
	//command() sends an ASCII Command (e.g. "TPX") to the controller and retrieves a Response (e.g. "123\r\n:").
	//The size of Response should be supplied as ResponseSize so that unallocated memory is not overwritten.
	//If you statically allocate your response buffer (e.g. char buff[100]) use sizeof(buff).
	char acPartialResponse[512] = {0}; //buffer to contain partial responses (which will be concatenated together to form the final response)
	int iPartialBytesRead = 0; //number of bytes read each time through the loop
	int iTotalBytesRead = 0;   //the total number of bytes read.  Can't exceed ResponseSize.  
	Response[0] = 0; //set response to null string    
	sock.write(Command, strlen(Command));

	//keep reading until we (a) get a colon (b) get a question mark (c) fill up the callers Response buffer
	while(1)
	{
		iPartialBytesRead = sock.read(acPartialResponse, sizeof(acPartialResponse)); //read some characters

		if(iPartialBytesRead <= 0)   //nothing read, keep reading until :
			continue;
		else if(iTotalBytesRead + iPartialBytesRead > ResponseSize) //get out of the loop if we will fill up the caller's buffer, iPartialBytesRead >= 1
			break;
		else {
			strncat(Response, acPartialResponse, iPartialBytesRead); //add the partial response to the full response.  Response is null terminated
			iTotalBytesRead += iPartialBytesRead; //tally up the total number of bytes read
			if (acPartialResponse[iPartialBytesRead - 1] == ':' || acPartialResponse[iPartialBytesRead - 1] == '?') //got a colon, iPartialBytesRead >= 1
				break;
		}
	}
	return(iTotalBytesRead);
}
