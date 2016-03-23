#include <BatteryMgt.h>
#include <TargetComs.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>

const int ledRed = 3;
const int ledGreen = 5;
const int ledBlue = 6;
const int impactSensor = A0;

unsigned long debouceSetTime = 0;
unsigned short debounce = 100;	//debounce delay in milliseconds
   
int sensorValue = 0;
float sensorVolts = 0;
float threshold = 1.5;	//threshold sensor voltage

TargetComs tc;
BatteryMgt batt;

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

/**
* User Configuration: nodeID - A unique identifier for each radio. Allows addressing
* to change dynamically with physical changes to the mesh.
*
* Configuration takes place below, prior to uploading the sketch to the device
* A unique value from 1-255 must be configured for each node.
* This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.
*
**/
#define nodeID 1

void setup() {
	pinMode(ledRed, OUTPUT);
	pinMode(ledGreen, OUTPUT);
	pinMode(ledBlue, OUTPUT);

	Serial.begin(115200);	
	mesh.setNodeID(nodeID);	// Set the nodeID manually	
	Serial.println("Connecting to the mesh...");
	mesh.begin();	// Connect to the mesh	

	tc.transmitData.nodeId = nodeID;
	tc.resetWatchdog();
}

void loop() {
	mesh.update();

	if (tc.checkWatchdog()) {
		Serial.println("No heartbeat detected!");
		meshConnectionTest();
	}

	sensorValue = analogRead(impactSensor);	//read analog pin, connected to piezo impact sensor circuit
	sensorVolts = sensorValue * (3.3 / 1023.0);	//convert read value to volts
	//if sensor reading greater than threshold and not debouncing tell gateway target hit
	threshold = (tc.transmitData.batVolts / 1000) * (tc.receiveData.impactThresh / 100);
	if (sensorVolts > threshold && (millis() - debouceSetTime > debounce)) {
		ledOff();
		tc.transmitData.targetHit = true;
		transmit();
		debouceSetTime = millis();
	}

	receive();

	batt.scheduledCheckBatteryVolts(&tc.transmitData.batVolts);
	if (tc.transmitData.batVolts < 2900) {
		batt.lowBattWarningLED(ledRed);
		sendError(tc.errorCode.nodeBattLow);
	}
}	//END loop

void ledOn() {
	analogWrite(ledRed, tc.receiveData.ledColor[0]);
	analogWrite(ledGreen, tc.receiveData.ledColor[1]);
	analogWrite(ledBlue, tc.receiveData.ledColor[2]);
}

void ledOff() {
	analogWrite(ledRed, 0);
	analogWrite(ledGreen, 0);
	analogWrite(ledBlue, 0);
}

void meshConnectionTest() {
	if (!mesh.checkConnection()) {
		Serial.println("Mesh Connection Test: Fail");		
	} else {
		Serial.println("Mesh Connection Test: Pass");
	}
	Serial.println("Renewing Address");	//refresh the network address
	mesh.renewAddress();
	Serial.println(mesh.mesh_address);
}

//TX - message transmit section
void transmit() {
	if (!mesh.write(&tc.transmitData, nodeData, sizeof(tc.transmitData))) {
		Serial.println("Message write failed");
		meshConnectionTest();
	} else {
		Serial.print("Transmitted message:");
		tc.printPacket(&tc.transmitData);
		tc.transmitData.targetHit = false;	//Gateway got hit message so turn it off, else keep sending message		
	}
}

//RX - message read section
void receive() {	
	while (network.available()) {
		RF24NetworkHeader header;
		network.read(header, &tc.receiveData, sizeof(tc.receiveData));
		Serial.print("Received message:");
		tc.printPacket(&tc.receiveData);
		parseReceiveData();
	}
}

void parseReceiveData() {
	tc.transmitData.ledColor[0] = tc.receiveData.ledColor[0];
	tc.transmitData.ledColor[1] = tc.receiveData.ledColor[1];
	tc.transmitData.ledColor[2] = tc.receiveData.ledColor[2];	
	if (tc.receiveData.turnTargetOn) {
		ledOn();
	}
	tc.transmitData.impactThresh = tc.receiveData.impactThresh;
	if (tc.receiveData.heartbeat) {
		tc.heartbeat = tc.receiveData.heartbeat;
	}	
}

void sendError(byte errorCode) {
	tc.transmitData.error = errorCode;
	transmit();
	tc.transmitData.error = tc.errorCode.noError;
}