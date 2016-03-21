#include <BatteryMgt.h>
#include <TargetComs.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>

const int ledRed = 3;
const int ledGreen = 5;
const int ledBlue = 6;

unsigned long targetRefreshTimer = 0;

TargetComs tc;
BatteryMgt batt;

/***** Configure the chosen CE,CS pins *****/
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

void setup() {
	pinMode(ledRed, OUTPUT);
	pinMode(ledGreen, OUTPUT);
	pinMode(ledBlue, OUTPUT);

	pinMode(0, INPUT_PULLUP); // just a precaution for BLE module

	Serial.begin(115200);	
	mesh.setNodeID(0);	// Set the nodeID to 0 for the master node	
	Serial.println("Connecting to the mesh...");
	mesh.begin();	// Connect to the mesh

	tc.watchdogTimeout = 9000;	//set watchdog to 9sec, less than the default 10sec the sensors time on
	tc.resetWatchdog();
}

void loop() {
	mesh.update();
	mesh.DHCP();	// Assign addresses to the sensor nodes if requested

	//Check if there are nodes connected
	//TODO: send error code, check on delay
	if (mesh.addrListTop > 0) {
		tc.nodesAvailable = true;
	} else {
		tc.nodesAvailable = false;
	}

	//send heartbeat message to each node
	//TODO: implement heartbeat on android side
	if (tc.checkWatchdog() && tc.nodesAvailable) {
		tc.transmitData.heartbeat = true;
		sendAllNodes();
		tc.heartbeatCount++;
	}

	//Reset all targets - for debugging
	if (tc.receiveData.targetHit && (millis() - targetRefreshTimer > 1000)) {
		tc.transmitData.turnTargetOn = true;
		tc.receiveData.targetHit = false;
		analogWrite(ledRed, 255);		//gateway LED - for debugging
		sendAllNodes();
	} else {
		targetRefreshTimer = millis();
	}

	//serialUI();		//allow user input from serial for debug
	meshReceive();	
	bleReceive();

	//Check battery voltage
	batt.scheduledCheckBatteryVolts(&tc.transmitData.batVolts);
	if (tc.transmitData.batVolts < 2900) {
		batt.lowBattWarningLED(ledRed);
	}
}	//END loop()

//send message of type header to all nodes connected to mesh
void sendAllNodes() {
	if (tc.nodesAvailable) {
		for (int i = 0; i < mesh.addrListTop; i++) {
			tc.transmitData.nodeId = mesh.getNodeID(mesh.addrList[i].address);			
			meshTransmit();
		}
	}	
}

//TODO: wrap this into a debug message
//void serialUI() {
//	if (Serial.available()) {
//		char c = toupper(Serial.read());
//		if (c == 'S') {
//			Serial.print(F("Turning on LED"));
//			tc.transmitData.turnTargetOn = true;
//			analogWrite(ledRed, 255);
//			if (!mesh.write(&tc.transmitData, 'D', sizeof(tc.transmitData))) {
//				Serial.println("Message write failed");
//			} 
//		}
//		if (c == 'T') {
//			Serial.print(F("Turning off LED"));
//			tc.transmitData.turnTargetOn = false;
//			analogWrite(ledRed, 0);
//			if (!mesh.write(&tc.transmitData, 'D', sizeof(tc.transmitData))) {
//				Serial.println("Message write failed");
//			} 
//		}
//		if (c == 'N') {
//			Serial.println(" ");
//			Serial.println(F("********Assigned Addresses********"));
//			for (int i = 0; i < mesh.addrListTop; i++) {
//				Serial.print("NodeID: ");
//				Serial.print(mesh.addrList[i].nodeID);
//				Serial.print(" RF24Network Address: 0");
//				Serial.println(mesh.addrList[i].address, OCT);
//			}
//			Serial.println(F("**********************************"));
//		}
//	}
//}

//TX - message transmit section
void meshTransmit() {
	if (!mesh.write(mesh.getAddress(tc.transmitData.nodeId), &tc.transmitData, nodeData, sizeof(tc.transmitData))) {
		tc.transmitData.error = tc.errorCode.nodeUnresponsive;
		tc.blePacketTransmit(&tc.transmitData);
	} 
}

//RX - message read section
void meshReceive() {
	// Check for incoming data from the sensors
	if (network.available()) {
		RF24NetworkHeader header;
		network.peek(header);
		network.read(header, &tc.receiveData, sizeof(tc.receiveData));
		tc.blePacketTransmit(&tc.receiveData);

		//turn off gateway LED if a target was hit - for debugging
		if (tc.receiveData.targetHit) {
			analogWrite(ledRed, 0);
		}
	}
}

void bleReceive() {
	while (Serial.available()) {
		//if next byte isn't a startByte skip it
		if (Serial.peek() != tc.receiveData.startByte) {
			Serial.read();
		} else {
			Serial.readBytesUntil(tc.transmitData.terminator, tc.blePacket, nodePacketSize);
			tc.blePacketReceive();
		}	
	}
}