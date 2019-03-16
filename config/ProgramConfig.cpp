/*
 * ProgramConfig.cpp
 *
 *  Created on: 12.09.2017
 *      Author: mateusz
 */

#include "ProgramConfig.h"

#include <sstream>
#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <libconfig.h++>

float ProgramConfig::lat = 4946.09f;
float ProgramConfig::lon = 1902.33f;
bool ProgramConfig::latns = false;
bool ProgramConfig::lonwe = true;

string ProgramConfig::comport;
string ProgramConfig::outputAprxFile;
unsigned char ProgramConfig::devicesNumber;
int ProgramConfig::masterId;
//vector<UmbDevice> ProgramConfig::devices;

using namespace libconfig;

ProgramConfig::ProgramConfig() {
	// TODO Auto-generated constructor stub

}

ProgramConfig::~ProgramConfig() {
	// TODO Auto-generated destructor stub
}

string ProgramConfig::getLat() {
	ostringstream out;
	out << ProgramConfig::lat;

	return out.str();
}

string ProgramConfig::getLon() {
	ostringstream out;
	
	char oout[10];
	memset(oout, 0, 10);
	sprintf(oout,"%08.2f" , ProgramConfig::lon);

//	out << ProgramConfig::lon;

	return oout;
}

string ProgramConfig::getLatns() {
	ostringstream out;

	if (ProgramConfig::latns)
		out << "S";
	else
		out << "N";

	return out.str();
}

const string& ProgramConfig::getOutputAprxFile() {
	return outputAprxFile;
}

unsigned char ProgramConfig::getDevicesNumber() {
	return devicesNumber;
}

int ProgramConfig::getMasterId() {
	return masterId;
}

//vector<UmbDevice>* ProgramConfig::getDevices() {
//	return &devices;
//}

//void ProgramConfig::setDevices(const vector<UmbDevice>& devices) {
//	this->devices = devices;
//}

void ProgramConfig::setMasterId(int masterId) {
	ProgramConfig::masterId = masterId;
}

void ProgramConfig::setDevicesNumber(unsigned char devicesNumber) {
	ProgramConfig::devicesNumber = devicesNumber;
}

void ProgramConfig::setOutputAprxFile(const string& outputAprxFile) {
	ProgramConfig::outputAprxFile = outputAprxFile;
}

string ProgramConfig::getLonwe() {
	ostringstream out;

	if (ProgramConfig::lonwe)
		out << "E";
	else
		out << "W";

	return out.str();
}

int ProgramConfig::manualConfig() {
	comport = "/dev/ttyUSB99";
	outputAprxFile = "/etc/aprxwx.txt";
	masterId = 10;

	devicesNumber = 2;
/*
	UmbDevice* d1 = new UmbDevice();
	UmbDevice* d2 = new UmbDevice();

	d1->setDeviceId(2);
	d1->setDeviceClass(12);

	d2->setDeviceId(1);
	d2->setDeviceClass(8);


				DeviceChannel* dc = new DeviceChannel();
				string s;

				dc->setChannelNumber(100);
				dc->setChannelName("temperatura");
				dc->setChannelUsage(DeviceChannel::CastUsageStringToEnum("TEMPERATURE"));
				dc->setMeasurementUnit(DeviceChannel::CastUnitStringToEnum("degC"));
				d1->getChannels()->push_back(*dc);

				delete dc;
				dc = new DeviceChannel();

				dc->setChannelNumber(440);
				dc->setChannelName("porywy");
				dc->setChannelUsage(DeviceChannel::CastUsageStringToEnum("WINDGUSTS"));
				dc->setMeasurementUnit(DeviceChannel::CastUnitStringToEnum("m/s"));
				d2->getChannels()->push_back(*dc);

				delete dc;
				dc = new DeviceChannel();

				dc->setChannelNumber(460);
				dc->setChannelName("predkosc");
				dc->setChannelUsage(DeviceChannel::CastUsageStringToEnum("WINDSPEED"));
				dc->setMeasurementUnit(DeviceChannel::CastUnitStringToEnum("m/s"));
				d2->getChannels()->push_back(*dc);

				delete dc;
				dc = new DeviceChannel();

				dc->setChannelNumber(580);
				dc->setChannelName("kierunek");
				dc->setChannelUsage(DeviceChannel::CastUsageStringToEnum("WINDDIR"));
				dc->setMeasurementUnit(DeviceChannel::CastUnitStringToEnum("m/s"));
				d2->getChannels()->push_back(*dc);

				delete dc;
				dc = new DeviceChannel();

				dc->setChannelNumber(365);
				dc->setChannelName("cisnienie");
				dc->setChannelUsage(DeviceChannel::CastUsageStringToEnum("PRESSURE"));
				dc->setMeasurementUnit(DeviceChannel::CastUnitStringToEnum("hPa"));
				d2->getChannels()->push_back(*dc);

				delete dc;


				devices.push_back(*d1);
				devices.push_back(*d2);
*/
	return 0;



}

int ProgramConfig::readConfigFromFile() {
#ifndef _ONLY_MANUAL_CFG
	Config cLibConfig;
/*
	try {
		cLibConfig.readFile("/etc/linuxmeteo.conf");
		cout << "--- libconfig++: Otwieram plik konfiguracyjny..." << endl;
	}

	catch(const FileIOException &ex)
	{
		printf("--- libconfig++: Nie mozna otworzyc pliku!!!\r\n");
		return -1;
	}
	catch(const ParseException &ex) {
		printf("--- libconfig++: Blad parsowania pliku wejsciowego!!!");
		return -2;
	}

	Setting &rRoot = cLibConfig.getRoot();

	cLibConfig.lookupValue("ComPort", comport);
	cLibConfig.lookupValue("OutputAprxFile", outputAprxFile);
	cLibConfig.lookupValue("MasterId", masterId);

	Setting &Devs = rRoot["Devices"];

	devicesNumber = Devs.getLength();

	for (int ii = 0; ii < devicesNumber; ii++)
	{
			UmbDevice* d = new UmbDevice();
			int temp;

			Devs[ii].lookupValue("deviceId", temp);
			d->setDeviceId((unsigned short)temp);

			Devs[ii].lookupValue("deviceClass", temp);
			d->setDeviceClass((unsigned short)temp);

			Setting &Channels = Devs[ii]["Channels"];

			for (int iii = 0; iii < Channels.getLength(); iii++)
			{
				DeviceChannel* dc = new DeviceChannel();
				string s;

				Channels[iii].lookupValue("Number", temp);
				dc->setChannelNumber((unsigned int) temp);

				Channels[iii].lookupValue("Name", s);
				dc->setChannelName(s);

				Channels[iii].lookupValue("Usage", s);
				dc->setChannelUsage(DeviceChannel::CastUsageStringToEnum(s));

				Channels[iii].lookupValue("Unit", s);
				dc->setMeasurementUnit(DeviceChannel::CastUnitStringToEnum(s));


				d->getChannels()->push_back(*dc);

			}

			devices.push_back(*d);

			delete d;
	}
*/
	return 0;
#endif
}
