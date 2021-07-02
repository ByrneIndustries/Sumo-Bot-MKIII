/*
This code controls my sumo bot MKIII.
*/
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
//#include <ESP32Servo.h>
/*
Declaring Variables for the pins that control the 2 motors and 
servos. The left pins control the DC motor on the left, and the 
right pins control the DC motor on the right. The leftPinS controls 
the servo on the left, and the rightPinS controls the servo on the 
right.
*/
//Left motor
const int leftPin1 = 18;
const int leftPin2 = 5;
const int enablePinL = 25;
int motorSpeedA = 0;
//Right motor
const int rightPin1 = 17;
const int rightPin2 = 16;
const int enablePinR = 26;
int motorSpeedB = 0;
//Ultrasonic sensor
const int pingPin = 33;
const int echoPin = 39;
//IR sensor
const int IRPin = 36;
int IRRead = 0;
//LEDc setup
const int ledfreq = 30000;
const int ledresolution = 8;
const int ledChannelL = 1;
const int ledChannelR = 2;
const int servoL = 3;
const int servoR = 4;
//servo
//Servo leftServo;
const int servoPinL = 32;
//Servo rightServo;
const int servoPinR = 27;

void setup() {
  Serial.begin(115200);
  Dabble.begin("MarkIII");
  pinMode(leftPin1,OUTPUT);
  pinMode(leftPin2,OUTPUT);
  pinMode(enablePinL,OUTPUT);
  pinMode(rightPin1,OUTPUT);
  pinMode(rightPin2,OUTPUT);
  pinMode(enablePinR,OUTPUT);
  pinMode(pingPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(IRPin,INPUT);
  ledcSetup(ledChannelL, ledfreq, ledresolution);
  ledcSetup(ledChannelR, ledfreq, ledresolution);
  ledcAttachPin(enablePinL, ledChannelL);
  ledcAttachPin(enablePinR, ledChannelR);
  //servo
  ledcSetup(servoL, ledfreq, ledresolution);
  ledcSetup(servoR, ledfreq, ledresolution);
  ledcAttachPin(servoPinL, servoL);
  ledcAttachPin(servoPinR, servoR);
  /*leftServo.setPeriodHertz(50);
  leftServo.attach(servoPinL);
  rightServo.setPeriodHertz(50);
  rightServo.attach(servoPinR);*/
  Serial.println("setup");
}

void loop () {
  //detectGround();
  //detectBot();
  Dabble.processInput();
  //servoArms(); 
  if (GamePad.isDownPressed())
  {
    ledcWrite(ledChannelL,255); 
    ledcWrite(ledChannelR,255);
    digitalWrite(leftPin1,LOW);
    digitalWrite(leftPin2,HIGH);
    digitalWrite(rightPin1,LOW);
    digitalWrite(rightPin2,HIGH);
    delay(200);
    ledcWrite(ledChannelL,0); 
    ledcWrite(ledChannelR,0);
  }
  if (GamePad.isUpPressed())
  {
    ledcWrite(ledChannelL,255); 
    ledcWrite(ledChannelR,255);
    digitalWrite(leftPin1,HIGH);
    digitalWrite(leftPin2,LOW);
    digitalWrite(rightPin1,HIGH);
    digitalWrite(rightPin2,LOW);
    delay(200);
    ledcWrite(ledChannelL,0); 
    ledcWrite(ledChannelR,0);
  }
  if (GamePad.isRightPressed())
  {
    ledcWrite(ledChannelL,255); 
    ledcWrite(ledChannelR,255);
    digitalWrite(leftPin1,LOW);
    digitalWrite(leftPin2,HIGH);
    digitalWrite(rightPin1,HIGH);
    digitalWrite(rightPin2,LOW);
    delay(100);
    ledcWrite(ledChannelL,0); 
    ledcWrite(ledChannelR,0);
  }
  if (GamePad.isLeftPressed())
  {
    ledcWrite(ledChannelL,255); 
    ledcWrite(ledChannelR,255);
    digitalWrite(leftPin1,HIGH);
    digitalWrite(leftPin2,LOW);
    digitalWrite(rightPin1,LOW);
    digitalWrite(rightPin2,HIGH);
    delay(100);
    ledcWrite(ledChannelL,0); 
    ledcWrite(ledChannelR,0);
  }
}

/*void loop () {
  for(;;){ // infinite loop
    Dabble.processInput();
    //servoArms();
    int xAxis = GamePad.getXaxisData();
    Serial.print("xAxis: ");
    Serial.print(xAxis);
    int yAxis = GamePad.getYaxisData();
    Serial.print("  yAxis: ");
    Serial.print(yAxis);
  
    if (yAxis < -1) {
      digitalWrite(rightPin1, HIGH);
      digitalWrite(rightPin2, LOW);
      digitalWrite(leftPin1, HIGH);
      digitalWrite(leftPin2, LOW);
      motorSpeedA = map(yAxis, 0, -6, 0, 255);
      motorSpeedB = map(yAxis, 0, -6, 0, 255);
    }
    
    else if (yAxis > 1) {
      digitalWrite(rightPin1, LOW);
      digitalWrite(rightPin2, HIGH);
      digitalWrite(leftPin1, LOW);
      digitalWrite(leftPin2, HIGH);
      motorSpeedA = map(yAxis, 0, 7, 0, 255);
      motorSpeedB = map(yAxis, 0, 7, 0, 255);
    }
    
    else {
      motorSpeedA = 0;
      motorSpeedB = 0;
    }
  
     //X-axis used for left and right control
    if (xAxis < -1) {
      int xMapped = map(xAxis, 0, -7, 0, 255);
      // Move to left - decrease left motor speed, increase right motor speed
      motorSpeedB = motorSpeedB - xMapped;
      motorSpeedA = motorSpeedA + xMapped;
      // Confine the range from 0 to 255
      if (motorSpeedB < 0) {
        motorSpeedB = 0;}
      if (motorSpeedB > 255) {
        motorSpeedB = 255;}
      if (motorSpeedA < 0) {
        motorSpeedA = 0;}
      if (motorSpeedA > 255) {
        motorSpeedA = 255;}
    }
    
    if (xAxis > 1) {
      int xMapped = map(xAxis, 0, 6, 0, 255);
      // Move right - decrease right motor speed, increase left motor speed
      motorSpeedB = motorSpeedB + xMapped;
      motorSpeedA = motorSpeedA - xMapped;
      // Confine the range from 0 to 255
      if (motorSpeedB < 0) {
        motorSpeedB = 0;}
      if (motorSpeedB > 255) {
        motorSpeedB = 255;}
      if (motorSpeedA < 0) {
        motorSpeedA = 0;}
      if (motorSpeedA > 255) {
        motorSpeedA = 255;}
    }
    Serial.print("  motorSpeedA = "); 
    Serial.print(motorSpeedA);
    Serial.print("  motorSpeedB = "); 
    Serial.println(motorSpeedB);
    ledcWrite(ledChannelL, motorSpeedA); // Send PWM signal to motor A
    ledcWrite(ledChannelR, motorSpeedB); // Send PWM signal to motor B
  }
}*/

/*void loop() {
  Dabble.processInput();
  //Serial.print("KeyPressed: ");
  if (GamePad.isUpPressed())
  {
    Serial.print("Up");
  }

  if (GamePad.isDownPressed())
  {
    Serial.print("Down");
  }

  if (GamePad.isLeftPressed())
  {
    Serial.print("Left");
  }

  if (GamePad.isRightPressed())
  {
    Serial.print("Right");
  }

  if (GamePad.isSquarePressed())
  {
    Serial.print("Square");
  }

  if (GamePad.isCirclePressed())
  {
    Serial.print("Circle");
  }

  if (GamePad.isCrossPressed())
  {
    Serial.print("Cross");
  }

  if (GamePad.isTrianglePressed())
  {
    Serial.print("Triangle");
  }

  if (GamePad.isStartPressed())
  {
    Serial.print("Start");
  }

  if (GamePad.isSelectPressed())
  {
    Serial.print("Select");
  }
  Serial.print('\t');

  int a = GamePad.getAngle();
  //Serial.print("Angle: ");
  //Serial.print(a);
  //Serial.print('\t');
  int r = GamePad.getRadius();
  //Serial.print("Radius: ");
  //Serial.print(r);
  //Serial.print('\t');
  int x = GamePad.getXaxisData();
  //Serial.print("x_axis: ");
  //Serial.print(x);
  //Serial.print('\t');
  int y = GamePad.getYaxisData();
  //Serial.print("y_axis: ");
  //Serial.println(y);
  //Serial.println();

  if (y > 0) 
  {
    ledcWrite(ledChannelL,int(y*36));
    digitalWrite(leftPin1,LOW);
    digitalWrite(leftPin2,HIGH);  
    ledcWrite(ledChannelR,int(y*36));
    digitalWrite(rightPin1,LOW);  
    digitalWrite(rightPin2,HIGH);
    Serial.println("forward");
  }
  if (y <= 0) 
  {
    ledcWrite(ledChannelL,int(y*-36));
    digitalWrite(leftPin1,HIGH);
    digitalWrite(leftPin2,LOW);  
    ledcWrite(ledChannelR,int(y*-36));
    digitalWrite(rightPin1,HIGH);  
    digitalWrite(rightPin2,LOW);
    Serial.println("backward");
  }
}*/


/*void forward(int speedL, int speedR) {
  ledcWrite(ledChannelL,speedL);
  digitalWrite(leftPin1,LOW);
  digitalWrite(leftPin2,HIGH);  
  ledcWrite(ledChannelR,speedR);
  digitalWrite(rightPin1,LOW);  
  digitalWrite(rightPin2,HIGH); 
  Serial.println("forward");   
}*/

/*void detectBot() {
   long duration, inches, cm;
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   duration = pulseIn(echoPin, HIGH);
   inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
   Serial.print(inches);
   Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();
   delay(100);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void detectGround() {
  int IRRead = digitalRead (IRPin);
  Serial.println(IRRead);
}*/

/*void servoArms() {
  if (GamePad.isTrianglePressed())
  {
    int servoVarL = 80;
    int servoVarR = 0;
    leftServo.write(servoVarL);
    rightServo.write(servoVarR);
  }
  if (GamePad.isCrossPressed())
  {
    int servoVarL = 10;
    int servoVarR = 70;
    leftServo.write(servoVarL);
    rightServo.write(servoVarR);
  }
  delay(200);
}*/
