#include <BatteryMgt.h>
#include <TargetComs.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

//Pin assignments
const int ledRed = 5;
const int ledGreen = 6;
const int ledBlue = 9;
const int bleInterruptPin = 2;
const int radioInteruptPin = 3;

//Interrupt flags
volatile bool message = false;
volatile bool bleMessage = false;
volatile bool wdHeartbeat = true;

//BLE Timer variables
unsigned long bleReceiveMessageStartTime = 0;
int bleStayAwake = 3000;

bool allowBattCheck = true;
unsigned long targetRefreshTimer = 0;

//Function declaration, needed to use function inside setup()
void ledFlash(int pin = 13, int numFlashes = 3, int msPeriod = 500, int duty = 50);

//class objects
TargetComs tc;
BatteryMgt batt;

//Data structs for gateway specific data
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
	if (mesh.begin()) {	// Connect to the mesh
		ledFlash(ledGreen);
	} else {
		ledFlash(ledRed);
	}

	gwTransmitData.nodeId = nodeID;

	cli(); // disable system interrupts during watchdog configuration
	wdt_reset(); // reset the watchdog timer
	WDTCSR |= (1 << WDCE) | (1 << WDE); // follow unlocking procedure at the bottom of page 51 on the datasheet
	WDTCSR = 1 << WDP0 | 1 << WDP3; // 8 seconds - Page 55 of the datasheet
	WDTCSR |= _BV(WDIE); // Enable the WD interrupt (note no reset)
	sei(); // enable interrupts again, it's cool now

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
	if (wdHeartbeat) {		
		tc.heartbeatCount++;
		wdHeartbeat = false;
		ledFlash(ledGreen, 2, 250, 50);
		if (tc.nodesAvailable) {
			//Serial.println("sending heartbeat");
			tc.transmitData.heartbeat = true;
			sendAllNodes();
		}		
	}

	//Reset all targets - for debugging
	//TODO: remove dependance on millis() to allow sleep
	if (tc.receiveData.targetHit && (millis() - targetRefreshTimer > 2000)) {
		tc.transmitData.turnTargetOn = true;
		tc.receiveData.targetHit = false;
		analogWrite(ledRed, 125);		//gateway LED - for debugging
		sendAllNodes();
		tc.transmitData.turnTargetOn = false;
		//Serial.println("turn LED on message");
	} 

	if (message) {
		meshReceive();
		message = false;
		attachInterrupt(digitalPinToInterrupt(radioInteruptPin), nrfISR, LOW);
	}

	//TODO: get message without delay
	if (bleMessage) {
		bleReceiveMessageStartTime = millis();
		delay(50);
		Serial.println("BLE Message received");
		bleReceive();
		ledFlash(ledBlue, 2, 250, 50);
		attachInterrupt(digitalPinToInterrupt(bleInterruptPin), bleISR, LOW);
		bleMessage = false;
	}		
		
	//Every 6 heartbeats check battery voltage
	if (tc.heartbeatCount % 6 == 0 && allowBattCheck) {
		tc.transmitData.batVolts = batt.checkBatteryVolts();
		allowBattCheck = false;
		//Serial.print("Checked Batt volts: ");
		//Serial.println(batt.voltsByteToFloat(tc.transmitData.batVolts));
		if (tc.transmitData.batVolts < batt.voltsFloatToByte(3.0)) {
			batt.lowBattWarningLED(ledRed);
			sendError(&gwTransmitData, tc.errorCode.nodeBattLow);
		}
	}

	if (millis() - bleReceiveMessageStartTime > bleStayAwake) {		
		do_sleep();
	}

}	//END loop()

//ISR for the mesh network, set flag to be handled in loop
void nrfISR() {
	detachInterrupt(digitalPinToInterrupt(radioInteruptPin));
	message = true;	
}

//ISR for the BLE module. Serial not available during sleep so wake signals are sent
//Sleep is disabled for a period of time to allow reading of BLE message
void bleISR() {
	detachInterrupt(digitalPinToInterrupt(bleInterruptPin));
	bleMessage = true;
}

//ISR for the watchdog, used to send heartbeat message
ISR (WDT_vect){
	wdHeartbeat = true;
	wdt_reset();	//reset timer 
}

void do_sleep(void) {
	Serial.println("Going to sleep");
	delay(10);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	sleep_enable();
	sleep_mode();	// System sleeps here

	sleep_disable();	// System continues execution here on interrupt 
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

//Flash LED
void ledFlash(int pin, int numFlashes, int msPeriod, int duty) {
	int state = digitalRead(pin);
	for (int i = 0; i < numFlashes; i++) {
		digitalWrite(pin, HIGH);
		delay(msPeriod*(duty / 100.0));
		digitalWrite(pin, LOW);
		delay(msPeriod*((100 - duty) / 100.0));
	}
	digitalWrite(pin, state);
}