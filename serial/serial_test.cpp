/*
 * serial_test.cpp
 *
 *  Created on: 22.09.2017
 *      Author: mateusz
 */

#include "serial.h"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE SERIAL
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(a)
{
	serial s;
	BOOST_REQUIRE_NO_THROW(s.init("/dev/ttyUSB0"));

	BOOST_REQUIRE_NO_THROW(s.test_transmit());
}
