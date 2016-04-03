////For configuring the BLE module
////Comment out all of MeshGateway the build and run once
////Run with USB/Serial Disconnected!
////LED will come on when programing complete
////1 set to 1200bps
////2 set to 2400bps
////3 set to 4800bps
////4 set to 9600bps(Default)
////5 set to 19200bps
////6 set to 38400bps
////7 set to 57600bps
////8 set to 115200bps
//void setup() { 
//	pinMode(3, OUTPUT);
//	pinMode(0, INPUT_PULLUP); // just a precaution for BLE module
//	// It seems internally HC-06 only responds to 9600
//	//Even if you set the baud rate to 115200 you can "reprogram"  the sketch again with 9600
//	Serial.begin(9600);		
//	Serial.print("AT");                 //PING 
//	delay(2000);
//	Serial.print("AT+NAMETarget_Mesh");        //CHANGE NAME
//	delay(2000);
//	Serial.print("AT+PIN1234");        //CHANGE PASSWORD
//	delay(2000);
//	Serial.print("AT+BAUD4");             //CHANGE SPEED TO 115K
//	delay(2000);
//}
//void loop() {
//	analogWrite(3, 255);
//	delay(10000);
//}
