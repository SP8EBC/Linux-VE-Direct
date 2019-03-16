/*
 * Float.h
 *
 *  Created on: 20.09.2017
 *      Author: mateusz
 */

#ifndef TYPES_FLOAT_H_
#define TYPES_FLOAT_H_

#include "ChannelValueFoundation.h"
#include "../aux/UnitsConversionMatrix.h"

#include <sstream>
#include <cstdio>

class Float: public ChannelValueFoundation {

	float store;

	static const string type;

public:
	Float();
	Float(float init);
	virtual ~Float();

	string getType();
	string toAprsConvertedString(MeasurementUnit from, MeasurementUnit to);
	string toString();

	float getValue() const;
	void setValue(float store);
};

#endif /* TYPES_FLOAT_H_ */
