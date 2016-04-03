#include <BatteryMgt.h>
#include <TargetComs.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

const int ledRed = 5;
const int ledGreen = 6;
const int ledBlue = 9;
const int sensorInteruptPin = 2;
const int radioInteruptPin = 3;

volatile bool message = false;
volatile byte wdCount = 0;

bool ledState = false;
bool allowBattCheck = true;

void ledFlash(int pin = 13, int numFlashes = 3, int msPeriod = 500, int duty = 50);

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
	if (mesh.begin()) {	// Connect to the mesh
		ledFlash(ledGreen);
	} else {
		ledFlash(ledRed);
	}

	tc.transmitData.nodeId = nodeID;

	attachInterrupt(digitalPinToInterrupt(sensorInteruptPin), hitISR, RISING);	//ISR for the impact sensor
	radio.maskIRQ(1, 1, 0);	//mask all mesh radio interupts except for the recieve
	attachInterrupt(digitalPinToInterrupt(radioInteruptPin), nrfISR, LOW);
}

void loop() {
	mesh.update();

	if (tc.transmitData.targetHit) {
		ledOff();
		Serial.println("Target hit!");
		transmit();
		tc.transmitData.targetHit = false;
	}

	if (message) {
		receive();
		message = false;
		attachInterrupt(digitalPinToInterrupt(radioInteruptPin), nrfISR, LOW);
	}	

	//Every 6 heartbeats check battery voltage
	if (tc.heartbeatCount % 6 == 0 && allowBattCheck) {
		tc.transmitData.batVolts = batt.checkBatteryVolts();
		allowBattCheck = false;
		Serial.print("Checked Batt volts: ");
		Serial.println(batt.voltsByteToFloat(tc.transmitData.batVolts));
		if (tc.transmitData.batVolts < batt.voltsFloatToByte(3.0)) {
			batt.lowBattWarningLED(ledRed);
			sendError(tc.errorCode.nodeBattLow);
		}
	}
		
	if (!ledState) {	//LED is off so we can sleep now
		do_sleep();
	}
	
	if (wdCount >= 15) {	//No mesh connection in 15*8sec=2min, so go to deep sleep
		Serial.println("No Mesh Detected");
		deepSleep();
	} else if (wdCount >= 2) {	//No mesh connection in 2*8sec, check connection, try to renew address
		Serial.println("No heartbeat detected!");
		meshConnectionTest();
	} 	
}	//END loop

void hitISR() {
	tc.transmitData.targetHit = true;	
}

void nrfISR() {
	message = true;
	detachInterrupt(digitalPinToInterrupt(radioInteruptPin));
}

ISR(WDT_vect) {	
	wdCount++;	
}

void do_sleep(void) {
	Serial.println("Going to sleep");	
	cli(); // disable system interrupts during watchdog configuration
	wdt_reset(); // reset the watchdog timer
	WDTCSR |= (1 << WDCE) | (1 << WDE); // follow unlocking procedure at the bottom of page 51 on the datasheet
	WDTCSR = 1 << WDP0 | 1 << WDP3; // 8 seconds - Page 55 of the datasheet
	WDTCSR |= _BV(WDIE); // Enable the WD interrupt (note no reset)
	sei(); // enable interrupts again, it's cool now	
	delay(10);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	sleep_enable();
	sleep_mode();	// System sleeps here
										
	sleep_disable();	// System continues execution here on interrupt 
	wdt_disable();
}

void deepSleep(void) {
	Serial.println("Going to Deep Sleep");
	delay(10);
	radio.powerDown();	// shut off radio to save lots of power
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);// sleep mode is set here
	sleep_enable();
	sleep_mode();	// System sleeps here

	sleep_disable();	// System continues execution here on interrupt 
	radio.powerUp();	//wake radio to see if mesh is there now
	wdCount = 0;		//reset watchdog, user must want to use device
}

void ledOn() {
	ledState = true;
	analogWrite(ledRed, tc.receiveData.ledColor[0]);
	analogWrite(ledGreen, tc.receiveData.ledColor[1]);
	analogWrite(ledBlue, tc.receiveData.ledColor[2]);
}

void ledOff() {
	ledState = false;
	analogWrite(ledRed, 0);
	analogWrite(ledGreen, 0);
	analogWrite(ledBlue, 0);
}

//Flash LED
void ledFlash(int pin, int numFlashes, int msPeriod, int duty) {
	int state = digitalRead(pin);
	for (int i = 0; i < numFlashes; i++) {
		digitalWrite(pin, HIGH);
		delay(msPeriod*(duty/100.0));               
		digitalWrite(pin, LOW);
		delay(msPeriod*((100-duty) / 100.0));
	}
	digitalWrite(pin, state);
}

void meshConnectionTest() {
	if (!mesh.checkConnection()) {
		ledFlash(ledRed);
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
		tc.packetPrintHex(&tc.transmitData);
		tc.transmitData.targetHit = false;	//Gateway got hit message so turn it off, else keep sending message		
	}
}

//RX - message read section
void receive() {
	while (network.available()) {
		RF24NetworkHeader header;
		network.peek(header);
		wdCount = 0;	//Received from mesh so connection is good
		if (header.type == nodeData) {
			network.read(header, &tc.receiveData, sizeof(tc.receiveData));
			Serial.print("Received message: ");
			Serial.println(header.id);
			//tc.printPacket(&tc.receiveData);
			parseReceiveData();						
		} else {
			Serial.println("Not a known data type!");
		}
	}
}

void parseReceiveData() {
	tc.transmitData.ledColor[0] = tc.receiveData.ledColor[0];
	tc.transmitData.ledColor[1] = tc.receiveData.ledColor[1];
	tc.transmitData.ledColor[2] = tc.receiveData.ledColor[2];	
	if (tc.receiveData.turnTargetOn) {
		ledOn();
	}
	tc.transmitData.time = tc.receiveData.time;
	if (tc.receiveData.heartbeat) {
		Serial.println("Heartbeat detected");
		tc.heartbeat = tc.receiveData.heartbeat;
		tc.heartbeatCount++;
		allowBattCheck = true;
		wdCount = 0;
	}	
}

void sendError(byte errorCode) {
	tc.transmitData.error = errorCode;
	transmit();
	tc.transmitData.error = tc.errorCode.noError;
}