/*
 * Integer.h
 *
 *  Created on: 16.09.2017
 *      Author: mateusz
 */

#ifndef TYPES_INTEGER_H_
#define TYPES_INTEGER_H_

#include "ChannelValueFoundation.h"

#include <string>

using namespace std;

class Integer: public ChannelValueFoundation {

	int store;

	static const string type;

public:
	Integer();
	Integer(int init);
	virtual ~Integer();

	string getType();
	string toAprsConvertedString(MeasurementUnit from, MeasurementUnit to);
	string toString();

	int getValue() const;
	void setValue(int store);
};

#endif /* TYPES_INTEGER_H_ */
