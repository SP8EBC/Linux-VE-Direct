/*
 * UnitsConversionMatrix_test.cpp
 *
 *  Created on: 19.09.2017
 *      Author: mateusz
 */

#include "UnitsConversionMatrix.h"
#include "../types/MeasurementUnit.h"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE UNITCONVERSION
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(a)
{
	float ms = 1.0f * UnitsConversionMatrix::conversionMatrix[MS][KMH];
	BOOST_CHECK_CLOSE(ms, 0.27f, 0.01);

	ms = 3.0f * UnitsConversionMatrix::conversionMatrix[MS][KMH];
	BOOST_CHECK_CLOSE(ms, 0.81f, 0.01);

	float kmh = 1.0f * UnitsConversionMatrix::conversionMatrix[KMH][MS];
	BOOST_CHECK_CLOSE(kmh, 3.6f, 0.01);

	kmh = 5.2f * UnitsConversionMatrix::conversionMatrix[KMH][MS];
	BOOST_CHECK_CLOSE(kmh, 18.72f, 0.01);

	float degf = 0 * UnitsConversionMatrix::conversionMatrix[DEGF][DEGC] + 32;
	BOOST_CHECK_CLOSE(degf, 32.0f, 0.01);


}
