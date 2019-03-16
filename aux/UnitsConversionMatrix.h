/*
 * ConversionMatrix.h
 *
 *  Created on: 19.09.2017
 *      Author: mateusz
 */

#ifndef AUX_UNITSCONVERSIONMATRIX_H_
#define AUX_UNITSCONVERSIONMATRIX_H_

class UnitsConversionMatrix {
public:
	UnitsConversionMatrix();
	virtual ~UnitsConversionMatrix();

	static const float conversionMatrix[8][8];
};

#endif /* AUX_UNITSCONVERSIONMATRIX_H_ */
