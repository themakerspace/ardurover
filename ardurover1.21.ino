/*  CODE FOR AN ULTRASONIC SENSING, DC MOTOR DRIVEN ROVING ROBOT

This program controls an ultrasonic sensor rover using L298N H Bridge Chip and SG04 ultrasonic 'ping' sensor.

The code is sometimes the easy part! the comments are the gold, they let you remember why you did what you did, and explain exactly how the code works.
Comments take a lot of time and commitment so read them! You will save hours of bumping you head. 
If explainations of code are clear then it allows you code to be easily adapted, which saves millions of man hours in the long run!

Made with the help of http://themakerspace.co.za

The logic of this code is to read the distance at regular intervals from an ultrasonic sensor, and then change the speed of each wheel to try and avoid and objects that have been sensed.
The motors are driven with PWM (pulse width modulation), which breaks down the sending of current to the motors in to lots of tiny pulses. The more speed you want, the more pulses you send.
You can send an amount of pulses from 1 to 255, with 1 pulse not being enough to do anything to the motor, and 255 being a continous current sent to the motor achieving maximum speed.
The other way to control speed of a DC motor is to change the voltage, which is a more "costly" approach. Since arduino allows PWM out the box, its an easy, cheap way to control motor speed.
DC motor's draw a certain amount of current depending on their size, arduino boards can't really supply enough current to drive them, so we need a motor driver.
We are using L298N Dual H-Bridge for this project. This device allows up to 2A per channel (motor) which is plenty. 
The H-Bridge works like a relay switch, the enable signal (low current) activates the high current supply to the motor, the signal can control polarity sent to the motor (which controls direction of the motor).

This is code draws heavily on code from Iain Portalupi L298N_Dual_H_Bridge

          *THE MOTOR SUBROUTINES WORK AS FOLLOWS*

motorA(mode, speed)
% replace A with B to control motor B %

mode is a number 0 -> 3 that determines what the motor 
will do.
0 = coast/disable the H bridge
1 = turn motor clockwise
2 = turn motor counter clockwise
3 = brake motor

speed is a number 0 -> 100 that represents percentage of
motor speed.
0 = off
50 = 50% of full motor speed
100 = 100% of full motor speed

EXAMPLE
Say you need to have motor A turn clockwise at 33% of its
full speed.  The subroutine call would be the following...

motorA(1, 33);

Created by 
Iain Portalupi http://www.youtube.com/iainportalupi
1/2/2014

This code is in the public domain.
*/

//this is where you can tell your program to include a prepackaged library for your code, instead of re-inventing the wheel for various functions your may require
#include <NewPing.h> //NewPing.h is a library for ultrasonic sensors which includes commonly used functions. You need to download this library from here if it isn't included in the IDE:http://playground.arduino.cc/Code/NewPing
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t)) //i'm not really sure what this does


#define TRIGGER_PIN  A0  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PIN     A1  // Arduino pin tied to echo pin on ping sensor.
#define MAX_DISTANCE 160 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define ENA 6  //enable A on the motor controller connects to arduino pin 6 (needs to be a pwm pin)
#define ENB 5  //enable B on the motor controller connects to arduino pin 5 (needs to be a pwm pin)
#define IN1 10  //IN1 on the motor controller connects to arduino pin 10, and conrtols one side of bridge A
#define IN2 11  //IN2 on the motor controller connects to arduino pin 11, and controls other side of A
#define IN3 8  //IN3 on the motor controller connects to arduino pin 8, and conrtols one side of bridge B
#define IN4 9  //IN4 on the motor controller connects to arduino pin 9, and controls other side of B
//#define SenA A3 //current sensing from H-Bridge to pin 12 of arduino (hope it doesnt blow it)
//#define SenB A2 //current sensing from H-Bridge

NewPing Roboeyes(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // distance sensor function called Roboeyes
int SR04; //integer called SR04 after the type of sensor
int cm2crash; //integer which measures the distance away from an object

int currentz; //integer for current sensing (not currently functional)

void setup() //this is the code that runs once when the arduino is turned on.
{

  //set all of the outputs as per the "define" schema above
 {
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
  delay(3000); // 3 seconds before the code (loop) starts
}
}


void loop()  // this code runs over and over forever, or until the arduino is turned off.
{
   Distance2crash(); //this function calculates the distance from the sensor to an object directly in it's path
   
 //This is a function using if-else-else logic, 
 //*(we don't use 0cm in our measurement function because the sensor often returns 0cm when there is running, you can check this on the serial monitor)
 
    if (1 < cm2crash && cm2crash <=15) //if the sensor senses something between 1cm and 15cm ... very close to object - reverse 
    {
      motorA(2, 75);  //motor A reverse speed 75%, you can change this if you want
      motorB(2, 50);  //motor B reverse speed 50%, you can change this if you want
 delay(1000);
    }
   
   else if (15 < cm2crash && cm2crash <30) //moderately close to object - tight turn left
{
 
  motorA(1, 50);  //have motor A turn clockwise at 50% speed, you can change this if you want
  motorB(2, 50);  //have motor B turn anticlockwise at 50% speed, you can change this if you want
    delay(500);
  
  }
  
  else if (cm2crash >= 30 && cm2crash < 50) //far from object - slow turn left
  
  {
  motorA(1, 66);  //have motor A turn clockwise at 66% speed, you can change this if you want
  motorB(2, 50);  //have motor B turn clockwise at 50% speed, you can change this if you want
  delay(500);
  }
  
 
  else // cruise
  {
  
  motorA(1, 66);  //have motor A turn clockwise at 66% speed
  motorB(1, 66);  //have motor B turn clockwise at 66% speed
 
  
  }
  

}

//******************   Current sense from H-bridge   **
/*
void currentsense()

   
{
 runEvery(100)

 {
   currentz = SenA;
 }
  Serial.println(currentz); 
}
*/

//******************   Sonar ultrasonic   *******************
void Distance2crash()
{
  runEvery(100)   //loop for ultrasonic measurement
  {
    SR04 = Roboeyes.ping();
    cm2crash = SR04 / US_ROUNDTRIP_CM;
    if (SR04 == NO_ECHO) // if the sensor did not get a ping        
    {
      cm2crash = MAX_DISTANCE;      //so the distance must be bigger then the max vaulue of the sensor
    }
    // Serial.print("Ping: "); //to check distance on the serial monitor
    // Serial.println(cm2crash); 

  }
}


//******************   Motor A control   *******************
void motorA(int mode, int percent)
{
  
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENA, LOW);  //set enable low to disable A
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      
      break;
  }
}
//**********************************************************


//******************   Motor B control   *******************
  void motorB(int mode, int percent)
{
  
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENB, LOW);  //set enable low to disable B
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      
      break;
  }
}



