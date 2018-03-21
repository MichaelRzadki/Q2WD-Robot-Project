//Final Project Class//
//Group 38//
//Group Members:
//Shashank Nayak: 10184852                                                                                                                             
//Michael Rzadki: 10197068                                                                                  
//Justin Bowen: 10177770

//Inclusions
#include <Servo.h> //allows servo motors to be utilized
#include "QSerial.h" //used for serial communication

//Pin Definitions
#define rxpin 3 //IR Receiver Pin
#define txpin -1 //IR Transmitter Pin
#define pinM2 4 //Left Motor, direction, Black
#define pinE2 5 //Left Motor, speed, Red
#define pinE1 6 //Right Motor, speed, Red
#define pinM1 7 //Right Motor, direction, Black
#define LEDPin 8 //LED pin
#define forcePin 9 //force sensor pin
#define leftIR A3 //Left IR Colour Sensor
#define middleIR A4 //Middle IR Colour Sensor
#define rightIR A5 //Right IR Colour Sensor

//Included Variable Definitions
QSerial myIRserial; //set a QSerial object
Servo myServo1, myServo2, myServo3; //set up the three servo motors

//Standard Variable Definitions
int forceSensed = 0; //Used to determine the current state of the grip. 0 means that the grip is open, 1 means that the grip is closed
int servoAngle = 30; //degree that the third servo motor is open. This is the gripper motor
int gripAngleMax = 175; //maximum degree for the third servo motor to open to
int gripAngleMin = 30; //minimum degree for the third servo motor to open to
int tiltAngle = 80; //degree for the tilt angle
int right_speed = 107; //speed of the right motor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
int left_speed = 107; //speed of the left motor



//setup: Initialized the program//
void setup() {
  //Setup IRserial attachment
  myIRserial.attach(rxpin, txpin);
  
  //Pin Declaration: see Pin Definition for an overview of what each pin is utilized for
  pinMode(forcePin, INPUT);
  pinMode(leftIR, INPUT);
  pinMode(middleIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(rxpin, INPUT);
  pinMode(pinM2, OUTPUT);
  pinMode(pinE2, OUTPUT);
  pinMode(pinE1, OUTPUT);
  pinMode(pinM1, OUTPUT);

  //Servo Attachments: assign pins to each servo motor
  myServo1.attach(10);
  myServo2.attach(11);
  myServo3.attach(12);

  //Servo Initialization: 
  myServo1.write(90); //center the pan servo
  myServo2.write(tiltAngle); //write the tilt servo to 80 degrees
  myServo3.write(servoAngle); //write the grip servo to 30 degrees, fully open
  delay(3000); //wait, allows for the servo motors to spasm before the main program starts
  
  Serial.begin(9600); //allows for serial data to be collected
  
  while(Serial.available() != true){ //if anything is received from the bluetooth connection, then start from loop
  }//end while
}//end setup



//loop: Main loop of the Program//
void loop() {

  char beaconVal = detectBeacon(); //Sets the beaconVal char value to the return from the detectBeacon method
                                   //The return value is a number, either '0', '1', or '2', in char format
  if (beaconVal == '0'){ //if beacon at position '0' is online
    turn(1); //turns 90 degrees to the left (by turning 90 degrees to the right 3 times)
    forward(); //goes forward
    grabRelease(1); //picks up the ball
  }
  else if (beaconVal == '1'){ //beacon '1' is online, which is the one directly to the front of the starting position
    forward(); //goes forward
    grabRelease(1); //picks up the ball
  }
  else{//'2'
    turn(2); //turns 90 degrees to the right
    forward(); //goes forward
    grabRelease(1); //picks up the ball
  }//end if

  backup(); //backs up slightly
  turn(0); //turns 180 degrees (by turning 90 degrees to the right 2 times)
  forward(); //goes forward to the next solid line

  //resets to 
  if (beaconVal == '0'){
    turn(2); //turn 90 degrees to the right
  }
  else if (beaconVal == '1'){ //doesn't need to rotate, as it is facing towards the net
  }
  else{//'2'
    turn(1); //turns 90 degrees to the left (by turning 90 degrees to the right 3 times)
  }//end if
  
  forward(); //goes forward until the next solid line
  grabRelease(0); //releases the ball

  backup(); //backs up 
  turn(0); //turn 180 degrees
  forward(); //goes forward until the center
}//end loop



//grabRelease: Method utilized to grab and release the ball//
void grabRelease(int grabVal){
  if (grabVal == 1){  //grabs the ball
   while((servoAngle < gripAngleMax)&&(forceSensed == 0)){ //while the servo can still close and nothing has been picked up
      servoAngle++; //increase the servo angle
      myServo3.write(servoAngle); //writes the angle to the servo motor, closing the grip
      if (digitalRead(forcePin)== HIGH) //if the force sensor returns high voltage, then something has been picked up
      {
        forceSensed = 1;//set the force sensed to one, meaning something has been picked up
      }//end if
      delay(25); //waits 25ms
    }//end while
    
    while(tiltAngle < 120){ //increase the angle of the tilt to 120 degrees, to allow for dropping into the net
      tiltAngle++; //increase the tilt angle
      myServo2.write(tiltAngle); //write to the servo motor
      delay(25); //wait 25ms
    }//adjust upwards
  }
  else{ //grabVal == 0, releases the ball
    while(servoAngle > gripAngleMin){
      servoAngle--; //decrease servo angle
      myServo3.write(servoAngle); //writes to the servo motor, opening the grip
      if (digitalRead(forcePin)== LOW)
      {
        forceSensed = 0; //the grip is empty
      }//end if 
      delay(25); //wait 25ms
    }//end while
    
   while(tiltAngle > 80){
      tiltAngle--; //decrease the tilt angle
      myServo2.write(tiltAngle); //write to the servo motor
      delay(25); //wait 25ms
   }//adjust downwards
  }//end if
}//end grab


//forward: method to go forward until a solid black line is detected//
void forward(){
  //Initialize the readers for the left, middle and right IR colour sensors
  int leftIRVal = analogRead(leftIR);
  int middleIRVal = analogRead(middleIR);
  int rightIRVal = analogRead(rightIR);

  //Go forward, with the path correcting, until a solid black line is detected
  do{
    //both motors forward
    digitalWrite(pinM2, HIGH); 
    digitalWrite(pinM1, HIGH);
    //both motors moving straight forward
    analogWrite(pinE2, left_speed+3);
    analogWrite(pinE1, right_speed);  

    delay(300); //wait 300ms

    //detect IR values
    leftIRVal = analogRead(leftIR);
    middleIRVal = analogRead(middleIR);
    rightIRVal = analogRead(rightIR);

    //correct the course of the vehicle
    if(leftIRVal > middleIRVal){
      analogWrite(pinE2, 120);//drift right
      analogWrite(pinE1, 107);
    }
    else if (rightIRVal > middleIRVal){
      analogWrite(pinE2, 120);//drift right
      analogWrite(pinE1, 107);
    }//end if
    delay(400);
  } while(((middleIRVal+100)>rightIRVal>(middleIRVal-100))&&(((middleIRVal+100)>leftIRVal>(middleIRVal-100)))&&(middleIRVal > 800));

  //stop the robot
  analogWrite(pinE2, 0);
  analogWrite(pinE1, 0);
  delay(300);//wait 300ms
}//end forward



//backup: backs the robot up a short distance//
void backup(){   
  //sets both motors to reverse
  digitalWrite(pinM2, LOW); 
  digitalWrite(pinM1, LOW);
  analogWrite(pinE2, left_speed + 3);
  analogWrite(pinE1, right_speed);  

  delay(1500); //waits 1.5s

  //stop the robot
  analogWrite(pinE2, 0);
  analogWrite(pinE1, 0);
  delay(300);//wait 300ms
}//end backup



//turn: Turns the robot//
void turn(int side){
  //Local variables
  int repeatNum = 0; //number of times the 90 degree right turn will run
  int countTurn = 0; //number of times the 90 degree right turn has run

   //Sets number of times the turn program needs to repeat
  if (side == 2){ //90 degrees to the right
    repeatNum = 1; //repeats once
  }
  else if (side == 0){ //180 degree turn
    repeatNum = 2; //repeats twice
  }
  else { //90 degrees to the left, turns 270 degrees to the right
    repeatNum = 3; //repeats three times
  }//end if

  //turns 90 degrees for each repeat
  for (countTurn = 0; countTurn < repeatNum; countTurn++){
    //turns 90 degrees right
    digitalWrite(pinM2, HIGH);
    digitalWrite(pinM1, LOW);
    analogWrite(pinE2, left_speed);
    analogWrite(pinE1, right_speed);
    delay(900);
    
    //stop the robot
    analogWrite(pinE2, 0);
    analogWrite(pinE1, 0);
    delay(300);//wait 300ms
  }//end for
}//end turn



//Detect Beacon Section//
//detectBeacon Method: determines the beacon number, utilizing 90 degree turns to ensure that the beacon is detected regardless of location. After the beacon is detected, returns to initial facing, then returns the char value of the beacon//
char detectBeacon(){ 
  int detectCounter = 0; //used to determine the facing of the robot
  char receivedVal = receive(); //the value received from the IR receiver
  while (true){
    receivedVal = receive(); //receive the IR value
    if ((receivedVal == '0')||(receivedVal == '1')||(receivedVal == '2')){ //if the received value is one of the three possible beacons, then break. Otherwise, turn and try again 
      break; //leave the while loop
    }//end if
    
    detectCounter++; //increments detectCounter
    turn(2); //turns 90 degrees right

    if (detectCounter >= 4){ //if equal to or greater than four, it has made a full circle, therefore currently at original position. Greater than is just there to prevent any potential errors
      detectCounter = 0; //resets to zero, original position
    }//end if

    delay(1000); //waits for 1 second, before repeating
  }//end while


  //resets to original position
  if (detectCounter != 0){ //if the detection counter is equal to zero, it did not increment, and therefore the robot is currently at the original position
    while(detectCounter < 4){ 
      turn(2); //do one right turn
      detectCounter++; //increase the counter by one
    }//end while
  }//end if
  
  return receivedVal; //return the char value of the beacon
}//end detectBeacon


//receive Method: receives a char value from the IR receiver, and returns it//
char receive(){
  char value = myIRserial.receive(200); //detects the serial value from the IR receiver
  return value; //return char value
}//end recieve()


