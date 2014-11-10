#pragma once
#include <string>
#include <map>
#include<vector>
#include <exception>

using namespace std;

class ConfigReader
{
public:
	ConfigReader(void);
	~ConfigReader(void);
	/* Read a config file (in windows INI format) into the internal data structure*/
	bool parseFile(string fileName);
	/* section has to be set before reading any key value pairs*/
	bool setSection(string s);
	/* checks if a key is present in the currently selected section*/
	bool keyPresent(string key);
	/* */
	int getInt(string key);
	double getDouble(string key);
	string getString(string key);
	bool clear();
	

private:
	typedef map<string,string> keyValMap;
	typedef map<string,keyValMap*> sectionMap;
	bool dataValid;
	string fileName;
	sectionMap dataMap;
	keyValMap* selectedSection;

	
	bool readFile(string fileName);
	string toLowerCase(string s);
	vector<string> tokenize(const  string  & inString, const  string  & theDelimiter= " " );
	string removeLeadTrailWhtspc(string s);

	/*struct section{
		string name;
		map<string,string>* keyMap;
		section(string s, map<string,string>* k): name(s){
			keyMap = k;
		}
	};*/
};

