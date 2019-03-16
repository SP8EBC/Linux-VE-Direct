/*
 * Float.cpp
 *
 *  Created on: 20.09.2017
 *      Author: mateusz
 */

#include "Float.h"
#include "../aux/UnitsConversionMatrix.h"

#include <sstream>
#include <cstdio>
#include <cmath>
#include <string>

using namespace std;


const string Float::type = "float";

Float::Float() {
	// TODO Auto-generated constructor stub
	value = &this->store;

}

Float::~Float() {
	// TODO Auto-generated destructor stub
}

string Float::getType() {
	return Float::type;
}

string Float::toAprsConvertedString(MeasurementUnit from, MeasurementUnit to) {
	float output = this->store * UnitsConversionMatrix::conversionMatrix[from][to];
	if (to == DEGF)
	{
		output += 32.0f;
	}
	if (to == HPA)
	{
		output *= 10.0f;
	}

	char s[6] = {0, 0, 0, 0, 0, 0};

	if (to != HPA)
		sprintf(s, "%03d", (int)round(output));
	else
		sprintf(s, "%05d", (int)round(output));


	string o(s);

	return o;
}

float Float::getValue() const {
	return store;
}

Float::Float(float init) {
	store = init;
	value = &this->store;

}

void Float::setValue(float store) {
	this->store = store;
}

string Float::toString() {
	stringstream out;
	out << this->store;

	return out.str();

}
