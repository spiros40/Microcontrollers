#include<avr/wdt.h> /* Header for watchdog timers in AVR */
//#include <Ethernet.h>
/*
 * Industruino INDIO RS485 example communication with Arduino over RS485
 * this sketch is for the Arduino: it sends the value of a potentiometer as a byte over RS485
 * more info on RS485 for Arduino at https://arduino-info.wikispaces.com/SoftwareSerialRS485Example
 */
/*
#define SSerialRX        10  //Serial Receive pin
#define SSerialTX        11  //Serial Transmit pin

#define SSerialTxControl 3   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX

int byteSend;

void setup()  {
  pinMode(SSerialTxControl, OUTPUT);    
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  RS485Serial.begin(9600);   // set the data rate 
}

void loop() {
    digitalWrite(SSerialTxControl, RS485Transmit);   
    RS485Serial.write(analogRead(A0)/4); // Send pot reading
    delay(10);   
    digitalWrite(SSerialTxControl, RS485Receive);  // Disable RS485 Transmit      
    delay(100);
}*/
#define  ChipID 10
#define  LightSensor A0
#define  Temp1 A1
#define  Temp2 A2
#define  Humid A3
#define  Wind  A4
int incomingByte = 0; // for incoming serial data

void setup() {
  wdt_disable();  /* Disable the watchdog and wait for more than 2 seconds */
  pinMode(10, OUTPUT);
  wdt_enable(WDTO_2S);  /* Enable the watchdog with a timeout of 2 seconds */
  Serial.begin(9600);
  /*
  pinMode(ledPin, OUTPUT);  // sets the digital pin 13 as output
  pinMode(inPin, INPUT);    // sets the digital pin 7 as input
   digitalWrite(13, HIGH); // sets the digital pin 13 on
  digitalWrite(13, LOW);  // sets the digital pin 13 off
  delay(1000);            // waits for a second
  analogRead(A0)
  */
}


void sendJson(float temp1,float temp2,float temp3,float temp4, int IO){
  wdt_reset();
  String message;
  message = String("{id:" );
  message=message + ChipID +","+"temp1:"+ temp1 +","+"temp2:"+ temp2 +","+"temp3:"+ temp3 +","+"temp4:"+ temp4 +","+"Input_status:"+ IO +"}";
  Serial.println(message);
}

int readAnalogInput(byte Analog, byte Type){
  wdt_reset();
  int Value=0;
  //NTC 10k temperature
  if(Type=1){      
      float R1 = 10000;
      float logR2, R2, T;
      float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
      Value = analogRead(Analog);
      R2 = R1 * (1023.0 / (float)Value - 1.0);
      logR2 = log(R2);
      T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
      T = T - 273.15;
      Value= (T * 9.0)/ 5.0 + 32.0;
  }
  //Humidity
  if(Type=2){
      Value = analogRead(Analog);
  }
  //Light sensor
   if(Type=3){
      Value = analogRead(Analog);
  }
  return Value;
}

void loop() {
  // put your main code here, to run repeatedly:
  wdt_reset();/* Reset the watchdog */
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    wdt_reset();
    sendJson(readAnalogInput(Temp1, 1), readAnalogInput(Humid, 1),readAnalogInput(Humid, 1),readAnalogInput(Humid, 1),1);
  }
}
