/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>

int value = 0; // set values you need to zero

Servo firstESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

  firstESC.attach(9);    // attached to pin 9 I just do this with 1 Servo
  Serial.begin(115200);    // start serial at 9600 baud
  Serial.println("Start");
  
  //firstESC.writeMicroseconds(2000);
  firstESC.write(50);
  delay(4000);
}

void loop() {
//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions

  firstESC.write(127
 );  //127º MAximum Thrust->2ms impulse 
  /*if(Serial.available()) {
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.print ("CMD: "); Serial.println(value); 
    //firstESC.writeMicroseconds(value);
    firstESC.write(value);
    Serial.flush();
    delay(5000);
  }*/
  
/*  if(Serial.available()) {
    value = Serial.read();    // Parse an Integer from Serial
    Serial.print ("CMD: "); Serial.println(value); 
    //firstESC.writeMicroseconds(value);
    //firstESC.write(value);
    //Serial.flush();
    //delay(500);
  }
  */
}

/* Este program alterou o ESC. A partir deste momento depois de ser alimentado, deixou de emitir son.
 
 #include <Servo.h>

Servo m1;

void setup(){
  m1.attach(3);
  delay(1);
  m1.write(180); 
  delay(100);
}

void loop(){
  //m1.write(50); 
  //delay(1000);
}*/
