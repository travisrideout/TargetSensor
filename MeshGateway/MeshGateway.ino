#include <TargetComsLib.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>

const int ledRed = 3;
const int ledGreen = 5;
const int ledBlue = 6;

unsigned long heartbeatTimer = 0;
unsigned int heartbeatDelay = 9000;	//frequency at which heart beat signals are sent
unsigned long targetRefreshTimer = 0;

unsigned long heartbeatCount = 0;

/***** Configure the chosen CE,CS pins *****/
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

targetData rd;	//recieved data
targetData td;	//transmit data

void setup() {
	pinMode(ledRed, OUTPUT);
	pinMode(ledGreen, OUTPUT);
	pinMode(ledBlue, OUTPUT);

	initData(&td);
	initData(&rd);

	Serial.begin(115200);	
	mesh.setNodeID(0);	// Set the nodeID to 0 for the master node	
	Serial.println("Connecting to the mesh...");
	mesh.begin();	// Connect to the mesh
}

void loop() {
	mesh.update();
	mesh.DHCP();	// Assign addresses to the sensor nodes if requested

	serialUI();		//allow user input from serial for debug

	//--------------------------------------------------------------
	//TX - message transmit section
	//--------------------------------------------------------------
	//Heartbeat message
	if (millis() - heartbeatTimer > heartbeatDelay) {
		heartbeatTimer = millis();
		if (mesh.addrListTop > 0) {
			for (int i = 0; i < mesh.addrListTop; i++) {
				if (!mesh.write(mesh.addrList[i].address, &heartbeatCount, 'H', sizeof(heartbeatCount))) {					
					Serial.print("NodeID ");
					Serial.print(mesh.addrList[i].nodeID); 
					Serial.print(" Failed to recieve heartbeat # ");
					Serial.print(heartbeatCount);
					Serial.println("Message write failed");
				} else {
					Serial.print("Sent heartbeat # ");
					Serial.print(heartbeatCount);
					Serial.print(" to NodeID ");
					Serial.println(mesh.addrList[i].nodeID);
				}
			}
			heartbeatCount++;
		} else {
			Serial.println("No Nodes connected");
		}
	}
	//Reset target - for debug
	if (td.targetHit) {
		if (millis() - targetRefreshTimer > 1000) {
			td.turnTargetOn = true;
			td.targetHit = false;			
			analogWrite(ledRed, 255);
			Serial.print("Sending data: ");
			printPacket(&td);
			if (!mesh.write(mesh.getAddress(1), &td, 'D', sizeof(td))) {
				Serial.println("Message write failed");
			} else {
				td.packNum++;
			}
		}
	} else {
		targetRefreshTimer = millis();
	}

	//END - TX - message transmit section
	//--------------------------------------------------------------


	//--------------------------------------------------------------
	//RX - message read section
	//--------------------------------------------------------------
	// Check for incoming data from the sensors
	if (network.available()) {
		RF24NetworkHeader header;
		network.peek(header);		
		switch (header.type) {
			case 'D': 
				network.read(header, &rd, sizeof(rd));
				Serial.print("Recieved data: ");
				printPacket(&rd);
				if (rd.targetHit) {
					td.targetHit = rd.targetHit;
					analogWrite(ledRed, 0);
				} 
				break;
			default:
				Serial.print("Undefined data type");
				break;
		}
	}	
	//END - RX - message read section
	//--------------------------------------------------------------
}	//END loop()

void serialUI() {
	if (Serial.available()) {
		char c = toupper(Serial.read());
		if (c == 'S') {
			Serial.print(F("Turning on LED"));
			td.turnTargetOn = true;
			analogWrite(ledRed, 255);
			if (!mesh.write(&td, 'D', sizeof(td))) {
				Serial.println("Message write failed");
			} else {
				td.packNum++;
			}
		}
		if (c == 'N') {
			Serial.println(" ");
			Serial.println(F("********Assigned Addresses********"));
			for (int i = 0; i < mesh.addrListTop; i++) {
				Serial.print("NodeID: ");
				Serial.print(mesh.addrList[i].nodeID);
				Serial.print(" RF24Network Address: 0");
				Serial.println(mesh.addrList[i].address, OCT);
			}
			Serial.println(F("**********************************"));
		}
	}
}