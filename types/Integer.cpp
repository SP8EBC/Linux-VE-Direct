/*
 * Integer.cpp
 *
 *  Created on: 16.09.2017
 *      Author: mateusz
 */

#include "Integer.h"
#include "../aux/UnitsConversionMatrix.h"

#include <sstream>
#include <cstdio>
#include <cmath>

using namespace std;

const string Integer::type = "int";

Integer::Integer(){
	// TODO Auto-generated constructor stub
	value = &this->store;
}

Integer::~Integer() {
	// TODO Auto-generated destructor stub
}

string Integer::getType() {
	return Integer::type;
}

string Integer::toAprsConvertedString(MeasurementUnit from, MeasurementUnit to) {
	float output = (float)this->store * UnitsConversionMatrix::conversionMatrix[from][to];
	if (to == DEGF)
	{
		output += 32.0f;
	}

	char s[4] = {0, 0, 0, 0};

	sprintf(s, "%03d", (int)round(output));

	string o(s);

	return o;
}

int Integer::getValue() const {
	return store;
}

Integer::Integer(int init) {
	store = init;
	value = &this->store;

}

void Integer::setValue(int store) {
	this->store = store;
}

string Integer::toString() {
	stringstream out;
	out << this->store;

	return out.str();

}

