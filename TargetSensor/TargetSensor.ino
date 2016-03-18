#include <TargetComs.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>

const int ledRed = 3;
const int ledGreen = 5;
const int ledBlue = 6;
const int impactSensor = A0;

unsigned short debounce = 100;	//debounce delay in milliseconds
   
int sensorValue = 0;
float sensorVolts = 0;
float threshold = 1.5;	//threshold sensor voltage

TargetComs tc;

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

/**
* User Configuration: nodeID - A unique identifier for each radio. Allows addressing
* to change dynamically with physical changes to the mesh.
*
* In this example, configuration takes place below, prior to uploading the sketch to the device
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
	if (sensorVolts > threshold) {	//if sensor reading greater than threshold tell gateway target hit
		analogWrite(ledGreen, 0);
		tc.transmitData.targetHit = true;
	}

	transmit();
	receive();
}	//END loop

void meshConnectionTest() {
	if (!mesh.checkConnection()) {
		Serial.println("Mesh Connection Test: Fail");
		Serial.println("Renewing Address");	//refresh the network address
		mesh.renewAddress(5000);
		Serial.println(mesh.mesh_address);
	} else {
		Serial.println("Mesh Connection Test: Pass");
	}
}

//TX - message transmit section
void transmit() {
	//if target hit send message
	if (tc.transmitData.targetHit) {
		if (!mesh.write(&tc.transmitData, 'D', sizeof(tc.transmitData))) {
			Serial.println("Message write failed");
			meshConnectionTest();
		} else {
			Serial.print("Transmitted message:");
			tc.printPacket(&tc.transmitData);
			tc.transmitData.packNum++;
			delay(debounce);	//allow impact sensor signal to settle
			tc.transmitData.targetHit = false;	//Gateway got hit message so turn it off, else keep sending message		
		}
	}
}

//RX - message read section
void receive() {	
	while (network.available()) {
		RF24NetworkHeader header;
		network.peek(header);
		switch (header.type) {
			case 'D':
				network.read(header, &tc.receiveData, sizeof(tc.receiveData));
				Serial.print("Received message:");
				tc.printPacket(&tc.receiveData);
				if (tc.receiveData.turnTargetOn) {
					analogWrite(ledGreen, 255);
				}
				break;
			case 'H':
				network.read(header, &tc.heartbeatCount, sizeof(tc.heartbeatCount));
				Serial.print("Received heartbeat: ");
				Serial.println(tc.heartbeatCount);
				tc.heartbeat = true;
				break;
			default:
				break;
		}
	}
}