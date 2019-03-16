/*
 * ChannelValue.h
 *
 *  Created on: 16.09.2017
 *      Author: mateusz
 */

#ifndef TYPES_CHANNELVALUEFOUNDATION_H_
#define TYPES_CHANNELVALUEFOUNDATION_H_

#include <string>

#include "MeasurementUnit.h"

using namespace std;

class ChannelValueFoundation {
protected:

	void* value;

public:

	virtual string toString() = 0;
	virtual string toAprsConvertedString(MeasurementUnit from, MeasurementUnit to) = 0;
	virtual string getType() = 0;

	ChannelValueFoundation();
	virtual ~ChannelValueFoundation();
};

#endif /* TYPES_CHANNELVALUEFOUNDATION_H_ */
