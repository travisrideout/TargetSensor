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

targetData gwReceiveData;
targetData gwTransmitData;

/***** Configure the chosen CE,CS pins *****/
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

#define nodeID 0	//nodeID 0 for the master node

void setup() {
	pinMode(ledRed, OUTPUT);
	pinMode(ledGreen, OUTPUT);
	pinMode(ledBlue, OUTPUT);

	pinMode(0, INPUT_PULLUP); // just a precaution for BLE module	

	Serial.begin(115200);	
	mesh.setNodeID(nodeID);		
	//Serial.println("Connecting to the mesh...");
	mesh.begin();	// Connect to the mesh

	gwTransmitData.nodeId = nodeID;
	tc.watchdogTimeout = 9000;	//set watchdog to 9sec, less than the default 10sec the sensors time on
	tc.resetWatchdog();
}

void loop() {
	mesh.update();
	mesh.DHCP();	// Assign addresses to the sensor nodes if requested

	//Check if there are nodes connected
	//TODO: send error code on delay
	if (mesh.addrListTop > 0) {
		tc.nodesAvailable = true;
	} else {
		tc.nodesAvailable = false;
		sendError(&gwTransmitData, tc.errorCode.noNodesConnected);
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

	meshReceive();	
	bleReceive();

	//Check battery voltage
	batt.scheduledCheckBatteryVolts(&gwTransmitData.batVolts);
	if (gwTransmitData.batVolts < 2900) {
		batt.lowBattWarningLED(ledRed);
		sendError(&gwTransmitData, tc.errorCode.nodeBattLow);
	}
}	//END loop()

//send message to all nodes connected to mesh
void sendAllNodes() {
	if (tc.nodesAvailable) {
		for (int i = 0; i < mesh.addrListTop; i++) {
			tc.transmitData.nodeId = mesh.getNodeID(mesh.addrList[i].address);			
			meshTransmit(&tc.transmitData);
		}
	}	
}

//TX - message transmit section
void meshTransmit(targetData* data) {
	if (!mesh.write(mesh.getAddress(data->nodeId), &data, nodeData, sizeof(data))) {
		sendError(data, tc.errorCode.nodeUnresponsive);
	} 
}

//RX - message read section
void meshReceive() {
	// Check for incoming data from the sensors
	while (network.available()) {
		RF24NetworkHeader header;
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
			tc.blePacketReceive(&gwReceiveData);
			if (gwReceiveData.nodeId != 0) {
				meshTransmit(&gwReceiveData);
			} else {
				//TODO: handle gateway specific code here
			}							
		}	
	}
}

void sendError(targetData *data, byte errorCode) {
	data->error = errorCode;
	tc.blePacketTransmit(data);
}