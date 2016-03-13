#include <TargetComsLib.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>

const int ledRed = 3;
const int ledGreen = 5;
const int ledBlue = 6;
const int impactSensor = A0;

unsigned long heartbeatTimer = 0;	//time counter for heartbeat message check
unsigned int heartbeatDelay = 10000;	//interval at which to check gateway heartbeat, ms
unsigned short debounce = 100;	//debounce delay in milliseconds
   
int sensorValue = 0;
float sensorVolts = 0;
float threshold = 1.5;	//threshold sensor voltage

bool heartbeat = false;
unsigned long heartbeatCount;

targetData td;	//transmit data
targetData rd;	//recieve data

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

	initData(&td);	//Initialize transmit data packet
	initData(&rd);	//Initialize recieve data packet

	Serial.begin(115200);	
	mesh.setNodeID(nodeID);	// Set the nodeID manually	
	Serial.println("Connecting to the mesh...");
	mesh.begin();	// Connect to the mesh	
}

void loop() {
	mesh.update();

	//check Gateway heartbeat
	if (millis() - heartbeatTimer > heartbeatDelay) {
		heartbeatTimer = millis();
		if (!heartbeat) {
			Serial.println("No heartbeat detected!");
			meshConnectionTest();
		}
		heartbeat = false;
	}

	sensorValue = analogRead(impactSensor);	//read analog pin, connected to piezo impact sensor circuit
	sensorVolts = sensorValue * (3.3 / 1023.0);	//convert read value to volts
	if (sensorVolts > threshold) {	//if sensor reading greater than threshold tell gateway target hit
		analogWrite(ledGreen, 0);
		td.targetHit = true;
	}

	//--------------------------------------------------------------
	//TX - message transmit section
	//--------------------------------------------------------------

	//if target hit send message
	if(td.targetHit){		
		if (!mesh.write(&td, 'D', sizeof(td))) {	
			Serial.println("Message write failed");			
			meshConnectionTest();
		} else {
			Serial.print("Transmitted message:");
			printPacket(&td);
			td.packNum++;		
			delay(debounce);	//allow impact sensor signal to settle
			td.targetHit = false;	//Gateway got hit message so turn it off, else keep sending message		
		}		
	} 	
	

	// End - TX - message transmit section
	//--------------------------------------------------------------
	

	//--------------------------------------------------------------
	//RX - message read section
	//--------------------------------------------------------------
	while (network.available()) {
		RF24NetworkHeader header;
		network.peek(header);
		switch (header.type) {
			case 'D':
				network.read(header, &rd, sizeof(rd));
				Serial.print("Recieved message:");
				printPacket(&rd);
				if (rd.turnTargetOn) {
					analogWrite(ledGreen, 255);
				}
				break;
			case 'H':
				network.read(header, &heartbeatCount, sizeof(heartbeatCount));
				Serial.print("Recieved heartbeat: ");
				Serial.println(heartbeatCount);
				heartbeat = true;
				break;
			default:
				break;
		}		
	}	
	//END - RX - message read section
	//--------------------------------------------------------------
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