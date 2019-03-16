/*
 * ProgramConfig.h
 *
 *  Created on: 12.09.2017
 *      Author: mateusz
 */

#ifndef CONFIG_PROGRAMCONFIG_H_
#define CONFIG_PROGRAMCONFIG_H_

#include <string>
#include <vector>

//#include "UmbDevice.h"

using namespace std;

class ProgramConfig {
private:
	static float lat, lon;
	static bool latns, lonwe;

	static string comport;
	static string outputAprxFile;

	static unsigned char devicesNumber;
	static int masterId;

//	static vector<UmbDevice> devices;


public:
	ProgramConfig();
	virtual ~ProgramConfig();

	static int readConfigFromFile();
	static int manualConfig();

	static string getLat();
	static string getLon();
	static string getLatns();
	static string getLonwe();
	static const string& getOutputAprxFile();
	static void setOutputAprxFile(const string& outputAprxFile);
	static unsigned char getDevicesNumber();
	static void setDevicesNumber(unsigned char devicesNumber);
	static int getMasterId();
	static void setMasterId( int masterId);

//	static vector<UmbDevice>* getDevices();
//	void setDevices(const vector<UmbDevice>& devices);
};

#endif /* CONFIG_PROGRAMCONFIG_H_ */
