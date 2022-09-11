# Welcome to the code Documentation for the 2 week Arduino Course. 

So i have listed the code documentation for all the components we will learn or have learned in the course.


## LED / Buzzer

```c++
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(13, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
```


# Ultrasonic Sensor

### Simple distance calcutor 

```c++
const unsigned int TRIG_PIN=3;//trigger pin attached to digital pin 13
const unsigned int ECHO_PIN=2;//echo pin attached to digital pin 12
const unsigned int BAUD_RATE=9600;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(BAUD_RATE);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  

 const unsigned long duration= pulseIn(ECHO_PIN, HIGH);
 int distance= duration/29/2;
 if(duration==0){
   Serial.println("Warning: no pulse from sensor");
   } 
  else{
      Serial.print("distance to nearest object:");
      Serial.println(distance);
      Serial.println(" cm");
  }
 delay(1000);
 }
```

### Ultrasonic with LED (Wave)

```c++
const unsigned int TRIG_PIN=3;//trigger pin attached to digital pin 13
const unsigned int ECHO_PIN=2;//echo pin attached to digital pin 12
const unsigned int BAUD_RATE=9600;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(13, OUTPUT);
  Serial.begin(BAUD_RATE);
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  

 const unsigned long duration= pulseIn(ECHO_PIN, HIGH);
 int distance= duration/29/2;
 if(distance<50){
   digitalWrite(13, HIGH);    
   } 
  else{
      digitalWrite(13, LOW); 
  }
 }
 ``` 


# Light Sensor

### Simple value indicator

```c++

void setup() {
  Serial.begin(9600);
}


void loop() {
  int value = analogRead(A0);
  Serial.println("Analog value : ");
  Serial.println(value);
  delay(250);
}
```

### Light Sensor with LED/Buzzer

```c++
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}


void loop() {
  int value = analogRead(A0);
  Serial.println("Analog value : ");
  Serial.println(value);
  delay(250);
  
  if(value<80){
   digitalWrite(13, HIGH);    
   } 
  else{
      digitalWrite(13, LOW); 
  }
}
```

## RFID with TAG

Make sure you install MFRC522 library from Arduino IDE.

### Reading data from an RFID tag

After having the circuit ready, go to File > Examples > MFRC522 > DumpInfo and upload the code. This code will be available in Arduino IDE (after installing the RFID library). Run the code from the examples to find that the number of the RFID.

```c++
#include <SPI.h>
#include <MFRC522.h>
 
#define SS_PIN 10
#define RST_PIN 9
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
 
void setup() 
{
  Serial.begin(9600);   // Initiate a serial communication
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  Serial.println("Approximate your card to the reader...");
  Serial.println();
  pinMode(5, OUTPUT);

}
void loop() 
{
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }
  //Show UID on serial monitor
  Serial.print("UID tag :");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
  if (content.substring(1) == "4A 92 5F 3F") //change here the UID of the card/cards that you want to give access
  {
    Serial.println("Authorized access");
    Serial.println();
    digitalWrite(5, HIGH);
    delay(10000);
    digitalWrite(5, LOW);
  }
 
 else   {
    Serial.println(" Access denied");
    delay(3000);
  }
} 
```

## PIR Sensor.

```c++
int ledPin = 13;                // LED 
int pirPin = 4;                 // PIR Out pin 
void setup() {
 pinMode(ledPin, OUTPUT);     
 pinMode(pirPin, INPUT);     
 Serial.begin(9600);
}
void loop(){
 pirStat = digitalRead(pirPin); 
 if (pirStat == HIGH) {            // if motion detected
   digitalWrite(ledPin, HIGH);  // turn LED ON
   Serial.println("Hey I got you!!!");
 } 
 else {
   digitalWrite(ledPin, LOW); // turn LED OFF if we have no motion
 }
} 
```

## Humidity Sensor

Make sure you install the Library Of DHT11 temperature sensor library from Arduino IDE.

```c++

#include "DHT.h"        // including the library of DHT11 temperature and humidity sensor
#define DHTTYPE DHT11   // DHT 11
#define dht_dpin 8	//data pin of DHT11 sensor attached to digital pin 8 of arduino
DHT dht(dht_dpin, DHTTYPE); 
void setup(){
  Serial.begin(9600);
  dht.begin();
  
} 
void loop(){
  float h = dht.readHumidity();
  float t = dht.readTemperature(); 
  Serial.println("Humidity and temperature\n\n");
  Serial.print("Current humidity = ");
  Serial.print(h);
  Serial.print("%  ");
  Serial.print("temperature = ");
  Serial.print(t);
  delay(1000);
}
```
## IR Sensor

```c++

int IRSensor = 2; // connect ir sensor to arduino pin 2
int LED = 13; // conect Led to arduino pin 13



void setup() 
{
  pinMode (IRSensor, INPUT); // sensor pin INPUT
  pinMode (LED, OUTPUT); // Led pin OUTPUT
}

void loop()
{
  int statusSensor = digitalRead (IRSensor);
  
  if (statusSensor == 1){
    digitalWrite(LED, LOW); // LED LOW
  }
  
  else
  {
    digitalWrite(LED, HIGH); // LED High
  }
  
}
```


## Servo Motor

Make sure you install the Servo Library from Arduino IDE.

```c++

#include <Servo.h>
int servoPin = 3; 	//servo motor data pin attached to digital pin 3 of arduino 
// Create a servo object 
Servo Servo1; 
void setup() { 
   // We need to attach the servo to the used pin number 
   Servo1.attach(servoPin); 
}
void loop(){ 
   // Make servo go to 0 degrees 
   Servo1.write(0); 
   delay(1000); 
   // Make servo go to 90 degrees 
   Servo1.write(90); 
   delay(1000); 
   // Make servo go to 180 degrees 
   Servo1.write(180); 
   delay(1000); 
}
```


## Relay 

```c++

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(13, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
```


## Motor Driver

This is used as any other led which just inputs an digital output from the Arduino.


## Bluetooth Module 

```c++

#include<SoftwareSerial.h>
SoftwareSerial BT(10,11); //(Tx,Rx)
String readData;

void setup()
{
  BT.begin(9600);
  Serial.begin(9600);
 pinMode(A0, OUTPUT);
}

void loop()
{
  while(BT.available())
  {
  delay(10);
  char c = BT.read();
  readData+=c;
}

if(readData.length()>0)
{
  Serial.println(readData);

if(readData == "ON")
{
  analogWrite(A0, 180);
}

  
if(readData=="OFF")
  {
    analogWrite(A0, 0);
  }
  
  else
  {
  readData="";
}
}
}
```

## Final Code of the Bluetooth Controlled Car.(Android Version)


```c++
#include<SoftwareSerial.h>
SoftwareSerial BT(10,11); //(Tx,Rx)
String readData;
void setup()
{
  BT.begin(9600);
  Serial.begin(9600);
 pinMode(A0, OUTPUT);
 pinMode(A1, OUTPUT);
 pinMode(A3, OUTPUT);
 pinMode(A4, OUTPUT);
}
void loop()
{
  while(BT.available())
  {
  delay(10);
  char c = BT.read();
  readData+=c;
}

if(readData.length()>0)
{
  Serial.println(readData);
  if(readData=="Front")
  {
    analogWrite(A0, 180);
    analogWrite(A1, 0);
    analogWrite(A3, 0);
    analogWrite(A4, 180);
  }

  if(readData=="Back")
  {
    analogWrite(A0, 0);
    analogWrite(A1, 180);
    analogWrite(A3, 180);
    analogWrite(A4, 0);
  }
  
  if(readData=="Right")
  {
    analogWrite(A0, 0);
    analogWrite(A1, 180);
    analogWrite(A3, 0);
    analogWrite(A4, 180);
  }
  if(readData=="Left")
  {
    analogWrite(A0, 180);
    analogWrite(A1, 0);
    analogWrite(A3, 180);
    analogWrite(A4, 0);
  }
   
  if(readData=="OFF")
  {
    analogWrite(A0, 0);
    analogWrite(A1, 0);
    analogWrite(A3, 0);
    analogWrite(A4, 0);
  }
    else
  {
    digitalWrite(13 ,LOW);
  readData="";
}
}
}
```

## Final Code of the Bluetooth Controlled Car.(MAC Version)


```c++
char val ;  // Please remember to connect the TX pin of the bluetooth module with the RX pin of the arduino and vice-visa.

void setup(){
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  Serial.begin(9600);

}

void loop(){
  if ( Serial.available())
  {
    val = Serial.read();
   }

   if(val=='s')
  {
    analogWrite(A0, 180);
    analogWrite(A1, 0);
    analogWrite(A3, 0);
    analogWrite(A4, 180);
  }

  if(val=='w')
  {
    analogWrite(A0, 0);
    analogWrite(A1, 180);
    analogWrite(A3, 180);
    analogWrite(A4, 0);
  }
  
  if(val=='a')
  {
    analogWrite(A0, 0);
    analogWrite(A1, 180);
    analogWrite(A3, 0);
    analogWrite(A4, 180);
  }
  if(val=='d')
  {
    analogWrite(A0, 180);
    analogWrite(A1, 0);
    analogWrite(A3, 180);
    analogWrite(A4, 0);
  }
   
  if(val=='q')
  {
    analogWrite(A0, 0);
    analogWrite(A1, 0);
    analogWrite(A3, 0);
    analogWrite(A4, 0);
  }

  else{
  }
  }
  ```





