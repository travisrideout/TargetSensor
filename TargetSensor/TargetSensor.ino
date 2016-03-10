
int sensorPin = A0;    
int sensorValue = 0;
float sensorVolts = 0;
float threshold = 1.5;	//threshold sensor voltage
int debounce = 1000;	//debounce delay in milliseconds

void setup() {
  
  pinMode(13, OUTPUT);	// pin 13, on board LED, as an output for debug
  pinMode(2, OUTPUT);	// pin 2 output for external LED
}

void loop() {
	digitalWrite(2, HIGH);	//turn on external LED
	digitalWrite(13, LOW);	//turn off internal LED
	sensorValue = analogRead(sensorPin);	//read analog pin, connected to piezo impact sensor circuit
	sensorVolts = sensorValue * (3.3 / 1023.0);	//convert read value to volts

	//if sensor reading greater than threshhold turn off external LED, on internal LED, wait debounce, 
	if (sensorVolts > threshold) {
		digitalWrite(13, HIGH);
		digitalWrite(2, LOW);
		delay(debounce);
	}
}
