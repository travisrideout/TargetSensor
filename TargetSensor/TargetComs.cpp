// 
// 
// 

#include "TargetComs.h"

//Initialize targetData values
void initData(targetData *data) {
	data->packNum = 0;
	data->ledColor[0] = 125;
	data->ledColor[1] = 125;
	data->ledColor[2] = 125;
	data->turnTargetOn = false;
	data->targetHit = false;
	data->batVolts = 200;
	data->impactSensitivity = 50;
}

//Print all targetData values to serial
void printPacket(targetData *data) {
	Serial.println("");
	Serial.print("Packet #");
	Serial.println(data->packNum);

	Serial.print("LED color: R = ");
	Serial.print(data->ledColor[0]);
	Serial.print(", G = ");
	Serial.print(data->ledColor[1]);
	Serial.print(", B = ");
	Serial.println(data->ledColor[2]);

	Serial.print("Turn target on command = ");
	Serial.println(data->turnTargetOn);

	Serial.print("Target hit = ");
	Serial.println(data->targetHit);

	Serial.print("Battery volts = ");
	Serial.println(data->batVolts);

	Serial.print("Impact sensitivity = ");
	Serial.println(data->impactSensitivity);
	Serial.println("");
}


