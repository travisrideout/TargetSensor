#include <BatteryMgt.h>
#include <TargetComs.h>
#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <avr/sleep.h>

const int ledRed = 5;
const int ledGreen = 6;
const int ledBlue = 9;
const int bleInterruptPin = 2;
const int radioInteruptPin = 3;

volatile bool message = false;
volatile bool bleMessage = false;
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

	Serial.begin(9600);	
	mesh.setNodeID(nodeID);		
	//Serial.println("Connecting to the mesh...");
	mesh.begin();	// Connect to the mesh

	gwTransmitData.nodeId = nodeID;
	tc.watchdogTimeout = 9000;	//set watchdog to 9sec, less than the default 10sec the sensors time on
	tc.resetWatchdog();

	radio.maskIRQ(1, 1, 0);	//mask all mesh radio interupts except for the recieve
	attachInterrupt(digitalPinToInterrupt(radioInteruptPin), nrfISR, LOW);
	attachInterrupt(digitalPinToInterrupt(bleInterruptPin), bleISR, LOW);
}

void loop() {
	mesh.update();
	mesh.DHCP();	// Assign addresses to the sensor nodes if requested

	//Check if there are nodes connected
	//TODO: send error code on delay
	if (mesh.addrListTop > 0) {
		tc.nodesAvailable = true;
	} else {
		if (tc.nodesAvailable) {
			sendError(&gwTransmitData, tc.errorCode.noNodesConnected);
		}
		tc.nodesAvailable = false;		
	}

	//send heartbeat message to each node
	//TODO: implement heartbeat on android side
	if (tc.checkWatchdog() && tc.nodesAvailable) {
		tc.transmitData.heartbeat = true;
		sendAllNodes();
		//Serial.println("sending heartbeat");
		tc.heartbeatCount++;
	}

	//Reset all targets - for debugging
	if (tc.receiveData.targetHit && (millis() - targetRefreshTimer > 2000)) {
		tc.transmitData.turnTargetOn = true;
		tc.receiveData.targetHit = false;
		analogWrite(ledRed, 255);		//gateway LED - for debugging
		sendAllNodes();
		tc.transmitData.turnTargetOn = false;
		//Serial.println("turn LED on message");
	} else {		
	}

	if (message) {
		delay(50);
		meshReceive();
		message = false;
	}

	if (bleMessage) {
		delay(50);
		Serial.println("BLE Message received");
		bleReceive();
		attachInterrupt(digitalPinToInterrupt(bleInterruptPin), bleISR, LOW);
		bleMessage = false;
	}	

	//Check battery voltage
	if (batt.scheduledCheckBatteryVolts(&gwTransmitData.batVolts)) {
		if (gwTransmitData.batVolts < 2900) {
			batt.lowBattWarningLED(ledRed);
			sendError(&gwTransmitData, tc.errorCode.nodeBattLow);
		}
	}	
}	//END loop()

void nrfISR() {
	message = true;
}

void bleISR() {
	detachInterrupt(digitalPinToInterrupt(bleInterruptPin));
	bleMessage = true;
}

void do_sleep(void)
{
	Serial.println("Going to sleep");
	attachInterrupt(digitalPinToInterrupt(bleInterruptPin), bleISR, LOW);
	delay(100);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	sleep_enable();
	sleep_mode();                        // System sleeps here

	sleep_disable();                     // System continues execution here on interrupt 
}

//send message to all nodes connected to mesh
void sendAllNodes() {
	if (tc.nodesAvailable) {
		for (int i = 0; i < mesh.addrListTop; i++) {
			tc.transmitData.nodeId = (byte)mesh.getNodeID(mesh.addrList[i].address);			
			meshTransmit(&tc.transmitData);
		}
	}	
}

//TX - message transmit section
void meshTransmit(targetData* data) {
	if (!mesh.write(mesh.getAddress((int)data->nodeId), &tc.transmitData, nodeData, sizeof(tc.transmitData))) { //
		gwTransmitData.nodeId = data->nodeId;
		sendError(&gwTransmitData, tc.errorCode.nodeUnresponsive);
	} 
	//tc.printPacket(&tc.transmitData);
	//tc.blePacketTransmit(&tc.transmitData);
}

//RX - message read section
void meshReceive() {
	// Check for incoming data from the sensors
	while (network.available()) {
		RF24NetworkHeader header;
		network.read(header, &tc.receiveData, sizeof(tc.receiveData));
		tc.blePacketTransmit(&tc.receiveData);
		tc.packetPrintHex(&tc.receiveData);

		//turn off gateway LED if a target was hit - for debugging
		if (tc.receiveData.targetHit) {
			analogWrite(ledRed, 0);
			targetRefreshTimer = millis();
		}
	}
}

void bleReceive() {
	while (Serial.available()) {
		//if next byte isn't a startByte skip it
		if (Serial.peek() != tc.receiveData.startByte) {
			Serial.read();
			//Serial.println("pushing BLE");
		} else {
			for (int i = 0; i < nodePacketSize; i++) {
				tc.blePacket[i] = Serial.read();
			}
			//Serial.readBytesUntil(tc.transmitData.terminator, tc.blePacket, nodePacketSize+1);
			Serial.println("Recieved BLE Packet");
			blePacketPrint(); 
			tc.blePacketReceive(&gwReceiveData);			
			if (gwReceiveData.nodeId != 0) {
				meshTransmit(&gwReceiveData);
				//Serial.println("sending BLE message to mesh");
				//tc.blePacketTransmit(&gwReceiveData);
			} else {
				//TODO: handle gateway specific code here
			}							
		}	
	}
}

void blePacketPrint() {
	Serial.println("Raw BLE message:");
	for (int i = 0; i < nodePacketSize; i++) {
		Serial.print(tc.blePacket[i]);
		Serial.print(" ");
	}
	Serial.println(" ");
}

void sendError(targetData *data, byte errorCode) {
	data->error = errorCode;
	tc.blePacketTransmit(data);
	tc.packetPrintHex(data);
}