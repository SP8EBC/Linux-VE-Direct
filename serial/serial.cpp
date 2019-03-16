/*
 * serial.cpp
 *
 *  Created on: 22.09.2017
 *      Author: mateusz
 */

#include "serial.h"
#include <iostream>
#include <stdint.h>
#include <cstring>
#include <vector>
#include <string>
#include <sys/time.h>

#include "../exceptions/NullPointerE.h"
#include "../exceptions/TimeoutE.h"

#include "../config/ProgramConfig.h"

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/thread/thread.hpp>

#define SOH 0x01
#define STX 0x02
#define ETX 0x03
#define EOT 0x04

#define V10 0x10

#define MASTER_ID 0x01

using namespace std;

serial::serial() {
	// TODO Auto-generated constructor stubes
}

int serial::transmit(std::vector<uint8_t> &in) {


	write(handle, in.data(), in.size());
}

int serial::receive(std::vector<uint8_t> &out, uint16_t bytes_to_rx, uint16_t timeout_in_msec){
	uint8_t ln_rcv = 0;
	uint8_t rx_buf = 0;
	uint16_t n = 0;
	uint16_t pos = 0;

	boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::universal_time();
	boost::posix_time::ptime current_time;
//	vector<uint8_t> *rx = new vector<uint8_t>();


	try {

		do {
			current_time = boost::posix_time::microsec_clock::universal_time();

			if (current_time - start_time > boost::posix_time::microseconds(timeout_in_msec * 1000)) {
				throw TimeoutE();
			}

			n = read(handle, &rx_buf, 1);

			if (n == 65535) {
				boost::this_thread::sleep_for(boost::chrono::microseconds(500));
				continue;
			}
			else {
				out.push_back(rx_buf);
				pos++;
			}

		}while( pos <= bytes_to_rx );

	}
	catch(TimeoutE &ex) {
		return -1;
	}

	return 0;


}

serial::~serial() {
	// TODO Auto-generated destructor stub
}

void serial::init(string port)
{
	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);

	handle = open( port.c_str(), O_RDWR| O_NOCTTY );


	/* Error Handling */
	if ( tcgetattr ( handle, &tty ) != 0 ) {
	   std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
	}

	/* Save old tty parameters */
	tty_old = tty;

	/* Set Baud Rate */

	tty.c_iflag &= ~(IMAXBEL|IXOFF|INPCK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON|IGNPAR);
	tty.c_iflag |= IGNBRK;

	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~CRTSCTS;

	tty.c_lflag &= ~(ECHO|ECHOE|ECHOK|ECHONL|ICANON|ISIG|IEXTEN|NOFLSH|TOSTOP|PENDIN);
	tty.c_cflag &= ~(CSIZE|PARENB);
	tty.c_cflag |= CS8|CREAD;
	tty.c_cc[VMIN] = 0;		// bylo 80
	tty.c_cc[VTIME] = 3;		// byo 3

	cfsetospeed (&tty, (speed_t)B19200);
	cfsetispeed (&tty, (speed_t)B19200);

	/* Make raw */
//	cfmakeraw(&tty);

	/* Flush Port, then applies attributes */
	tcflush( handle, TCIFLUSH );
	if ( tcsetattr ( handle, TCSANOW, &tty ) != 0) {
	   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
	}

	std::cout << "Port serial skonfigurowany " << std::endl;
}

void serial::test_transmit()
{
	for (uint8_t i = 0; i < 255; i++)
		write( handle, &i, 1 );
}

short serial::checkCRC(char* pInputData) {
	char ii,i = 0;
	unsigned short crc = 0xFFFF;
	ii = pInputData[6] + 12;
	for (i = 0; i < ii - 3; i++)
		crc = calc_crc(crc, pInputData[i]);
	if ( (pInputData[ii - 2] == ( (crc & 0xFF00) >> 8) ) && ( pInputData[ii - 3] == (crc & 0xFF) ) )
		return 0;
	else
		return -1;
}

unsigned short serial::calc_crc(unsigned short crc_buff, unsigned char input) {
	unsigned char i;
	unsigned short x16;
	for	(i=0; i<8; i++)
	{
		// XOR current D0 and next input bit to determine x16 value
		if		( (crc_buff & 0x0001) ^ (input & 0x01) )
			x16 = 0x8408;
		else
			x16 = 0x0000;
		// shift crc buffer
		crc_buff = crc_buff >> 1;
		// XOR in the x16 value
		crc_buff ^= x16;
		// shift input for next iteration
		input = input >> 1;
	}
	return (crc_buff);
}

