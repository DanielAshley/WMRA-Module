#include "ConfigReader.h"
#include <vector>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include <assert.h>
#


ConfigReader::ConfigReader(void)
{
	dataValid = false;
}

ConfigReader::~ConfigReader(void)
{
	clear();
}

bool ConfigReader::keyPresent(string key ){
	string k = toLowerCase(key);
	if(selectedSection->find(k) != selectedSection->end()){
		return true;
	}
	else{
		return false;
	}
}

bool ConfigReader::setSection(string str){
	string sect = toLowerCase(str);
	if(dataMap.find(sect)!= dataMap.end()){
		selectedSection = dataMap[sect];
		return true;
	}
	else{
		return false;
	}
}

string ConfigReader::getString(string key){
	int result;
	string key_lower = toLowerCase(key);
	if(selectedSection->find(key_lower) != selectedSection->end()){
		string val = (*selectedSection)[key_lower];
		return val;
	}
	else {
		throw std::out_of_range ("Key :" + key + " not found");
	}
}

int ConfigReader::getInt(string key){
	int result;
	string key_lower = toLowerCase(key);
	if(selectedSection->find(key_lower) != selectedSection->end()){
		string val = (*selectedSection)[key_lower];
		istringstream stream(val);
		stream >> result;
		return result;
	}
	else {
		throw std::out_of_range ("Key :" + key + " not found");
	}
}

double ConfigReader::getDouble(string key){
	double result;
	string key_lower = toLowerCase(key);
	if(selectedSection->find(key_lower) != selectedSection->end()){
		string val = (*selectedSection)[key_lower];
		istringstream stream(val);
		stream >> result;
		return result;
	}
	else {
		throw std::out_of_range ("Key :" + key + " not found");
	}
}

bool ConfigReader::parseFile(string fn){
	
	if(readFile(fn)) {
		fileName = fn;
		dataValid = true;
		return true;
	}
	else {
		return false;
	}
}

bool ConfigReader::clear(){
	// relese all all memory for 
	sectionMap::iterator it ;
	for( it = dataMap.begin(); it != dataMap.end(); ++ it){
		if(it->second != NULL) delete it->second;
	}
	dataMap.clear();
	dataValid = false;
	fileName = "";
	selectedSection = NULL;
	return true;
}

vector<string> ConfigReader::tokenize(const  string & inString, const  string& theDelimiter )
{
   assert( theDelimiter.size() > 0 ); 

   vector<string>  stringTokens;
   string whiteSpc(" \t\f\v\n\r");
   size_t  start = 0, end = 0;

   while ( end != string::npos ){
      // skip white space
      start = inString.find_first_not_of(whiteSpc, start);
      if(start != string::npos){
         end = inString.find( theDelimiter, start );
         // If at end, use length=maxLength.  Else use length=end-start.
         stringTokens.push_back( inString.substr( start,
            (end == string::npos) ? string::npos : end - start ) );

         // If at end, use start=maxSize.  Else use start=end+delimiter.
         start = (   ( end > (string::npos - theDelimiter.size()) )
            ?  string::npos  :  end + theDelimiter.size()    );
      }
      else break;
   }
   return stringTokens;
}


bool ConfigReader::readFile(string fileName)
{
   bool readSuccess = true;
   bool sectionSelected = false;
   ifstream inputFile;
   if(fileName == "") fileName = "settings.txt";
   inputFile.open (fileName.c_str());
   if (!inputFile.is_open()) 
   {
      //cout << "Error opening input file" << endl;
      return false;
   }

   string temp_in;
   size_t start;
   vector<string> tokens;
   string whiteSpc(" \t\f\v\n\r");
   bool eofReached = false;

   while (!eofReached)
   {        
      getline(inputFile, temp_in); 
      if(inputFile.eof()){
         eofReached = true;
         //continue;
      }
      //skip blank lines
      start = 0;
      start = temp_in.find_first_not_of(whiteSpc, start);
      if(start == string::npos) continue;
      //check i=if its a comment line and skip
      if(temp_in[start] == '#') continue;
      //remove trailing comments
      size_t commentPos = temp_in.find_first_of('#', start);
      if(commentPos != string::npos){
         // erase from position of '#' to end of string
         temp_in.erase(temp_in.begin()+commentPos, temp_in.end());
      }

      //check if its the start of a section
      if(temp_in[start] == '['){
         //extract section name
         size_t end = temp_in.find_first_of(']', start);
         if(end == string::npos){
            readSuccess = false;
            break;
         }
         string secName = temp_in.substr( start+1, end-start-1);
         secName = removeLeadTrailWhtspc(secName);
         secName = toLowerCase(secName);
         // add to SectionMap if it doesnt already exist
         if(dataMap.find(secName) == dataMap.end()){
            dataMap[secName] = new keyValMap();
            selectedSection = dataMap[secName];
            sectionSelected = true;
         }
         else{ // double declaration of section name, so break
            readSuccess = false;
            break;
         }
         continue; // read the next line from the file. next iteration of the loop
      }

      // read values 
      // tokenizethe string with "=" as delimitters
      tokens = tokenize(temp_in, "=");
      if(tokens.size()== 0)continue;

      if(tokens.size() >= 2 && sectionSelected){
         string key = removeLeadTrailWhtspc(tokens[0]);
         key = toLowerCase(key);
         string val = removeLeadTrailWhtspc(tokens[1]);
         if(selectedSection->find(key) == selectedSection->end()){
            // assign the name value pair to the keyValMap				
            (*selectedSection)[key] = val;
         }
      }
   }
   inputFile.close();
   return readSuccess;
}


string ConfigReader::removeLeadTrailWhtspc(string s){
	string whtSpc(" \t\f\v\n\r");
	size_t start;
	size_t end;
	start = s.find_first_not_of(whtSpc);
	end= s.find_last_not_of(whtSpc);
	if (start != string::npos && end != string::npos){
		return s.substr(start, end-start+1);
	}
	else {
		return s;
	}
}


string ConfigReader::toLowerCase(string str) {
    for (int i=0; i < (int)str.length(); i++) 
        if (str[i] >= 0x41 && str[i] <= 0x5A) 
            str[i] = str[i] + 0x20;
    return str;
}