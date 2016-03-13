#include "TargetComs.h"
#include <RF24.h>
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>

const int ledRed = 3;
const int ledGreen = 5;
const int ledBlue = 6;
const int impactSensor = A0;

uint32_t displayTimer = 0;
int delayTime = 10;
int debounce = 1000;	//debounce delay in milliseconds
   
int sensorValue = 0;
float sensorVolts = 0;
float threshold = 1.5;	//threshold sensor voltage

bool connectingLED = false;

targetData td;

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
	// Set the nodeID manually
	mesh.setNodeID(nodeID);
	// Connect to the mesh
	Serial.println(F("Connecting to the mesh..."));
	mesh.begin();
	Serial.println(F("Initializing data"));
	initData(&td);	//get some values in data
}

void loop() {
	Serial.println(F("1"));
	//mesh.update();
	Serial.println(F("2"));

	// Send to the master node every second
	if (millis() - displayTimer >= 1000) {
		displayTimer = millis();
		connectingLedFlash();

		// Send an 'M' type message containing the current millis()
		if (!mesh.write(&displayTimer, 'M', sizeof(displayTimer))) {

			// If a write fails, check connectivity to the mesh network
			if (!mesh.checkConnection()) {
				//refresh the network address
				Serial.println("Renewing Address");
				mesh.renewAddress();
			}
			else {
				Serial.println("Send fail, Test OK");
			}
		}
		else {
			Serial.print("Send OK: "); Serial.println(displayTimer);
		}
	}
	//analogWrite(ledRed, 0);

	while (network.available()) {
		RF24NetworkHeader header;
		network.read(header, &td, sizeof(td));
		printPacket(&td);

		sensorValue = analogRead(impactSensor);	//read analog pin, connected to piezo impact sensor circuit
		sensorVolts = sensorValue * (3.3 / 1023.0);	//convert read value to volts

		//if sensor reading greater than threshhold turn off external LED, on internal LED, wait debounce, 
		if (sensorVolts > threshold) {
			digitalWrite(13, HIGH);
			digitalWrite(2, LOW);
			delay(debounce);
		}
	}	
}

void connectingLedFlash() {
	connectingLED = !connectingLED;
	if (connectingLED) {
		analogWrite(ledRed, 255);
	} else {
		analogWrite(ledRed, 0);
	}
}