// TargetComs.h

#ifndef _TARGETCOMS_h
#define _TARGETCOMS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct targetData {
	long packNum;
	byte ledColor[3];
	bool turnTargetOn;
	bool targetHit;
	byte batVolts;
	byte impactSensitivity;
};

void initData(targetData*);
void printPacket(targetData*);

#endif

