/*
 * UnitsConversionMatrix.cpp
 *
 *  Created on: 19.09.2017
 *      Author: mateusz
 */

#include "UnitsConversionMatrix.h"

/*
	MS = 1,
	KMH = 2,
	KTS = 3,
	HPA = 4,
	DEGC = 5,
	DEGF = 6
*/

const float UnitsConversionMatrix::conversionMatrix[8][8] = {
		//				FROM
		//	  m/s	km/h   kts   hPa  degC  degF    mph
/* TO*/	{.0f, .0f  , .0f , .0f  , .0f , .0f , .0f , .0f  },
/* ms*/ {.0f, 1.0f , .27f, .51f , .0f , .0f , .0f , 2.24f},
/* kmh*/{.0f, 3.6f , 1.0f, 1.85f, .0f , .0f , .0f , .621f},
/* kts*/{.0f, 1.94f, .53f, 1.0f , .0f , .0f , .0f , 1.15f},
/* hpa*/{.0f, .0f  , .0f , .0f  , 1.0f, .0f , .0f , .0f  },
/*degc*/{.0f, .0f  , .0f , .0f  , .0f , 1.0f, 1.8f, .0f  },
/*degf*/{.0f, .0f  , .0f , .0f  , .0f , .55f, 1.0f, .0f  },
/* mph*/{.0f, .447f, 1.6f, .868f, .0f , 1.8f, 1.0f, .0f  }
};

UnitsConversionMatrix::UnitsConversionMatrix() {
	// TODO Auto-generated constructor stub

}

UnitsConversionMatrix::~UnitsConversionMatrix() {
	// TODO Auto-generated destructor stub
}

