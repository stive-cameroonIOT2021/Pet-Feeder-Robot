#include <AccelStepper.h>
#include <NewPing.h>
#include <Servo.h>
////////////////////////////////////////////////////////////
// Define step constants
#define FULLSTEP 8
#define HALFSTEP 4

// Define end switch pins
#define END_SWITCH1 38
#define END_SWITCH2 39

int ft_Speed = 700;

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper stepper1(HALFSTEP, 34, 36, 35, 37);
const int relay_stepper = 40;

// Variable to track the last debounce time
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay = 50;

// Variables to store the switch states
bool lastSwitchState1 = LOW;
bool lastSwitchState2 = LOW;
bool switchState1 = LOW;
bool switchState2 = LOW;

// Variable to track direction
bool directionForward = true;
// Variable to track whether motors should be moving
bool motorsRunning = false;
///////////////////////////////////////////////////////////
Servo myservo;  // create servo object to control a servo
int posServo = 0;    // variable to store the servo position

//Motor pump
const int WaterPumpPin = 28;
//#include"Ultrasonic.h"
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.


NewPing L_ultra(22, 23, MAX_DISTANCE); // Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing R_ultra (25, 24, MAX_DISTANCE);
NewPing F_ultra(27, 26, MAX_DISTANCE);
//ulstrasonic pinsPet
/*distSensor L_ultra(22, 23);
distSensor R_ultra(25, 24);
distSensor F_ultra(27, 26);*/
//read distances
int distL, distF, distR;
int obstacle = 20;//in cm

bool Auto = false;

// Step motor pins assignment
#define FL_step 2
#define FR_step 3
#define BL_step 9
#define BR_step 4

#define FL_dir 5
#define FR_dir 6
#define BL_dir 10
#define BR_dir 7

// Define the stepper motors and the pins they will use
AccelStepper MFL(AccelStepper::DRIVER, FL_step, FL_dir); // motor front left
AccelStepper MFR(AccelStepper::DRIVER, FR_step, FR_dir); // motor front right
AccelStepper MBL(AccelStepper::DRIVER, BL_step, BL_dir); // motor back left
AccelStepper MBR(AccelStepper::DRIVER, BR_step, BR_dir); // motor back right

const int stepsPerRevolution = 3200; // 16x microstepping
const int Speed = 900;

//water sensor pin
#define waterSens_pin  A7
float water_percent = 0;

// Function prototype
void Move(int v1, int v2, int v3, int v4);
void Obstacle_avoid();

unsigned long currentMillis ;
unsigned long previousMillis = 0,  previousMillis1 = 0;
const long interval = 2000; // interval to wait (milliseconds)
const long stopInterval = 1000, stopInterval1 = 1000; // interval to stop between movements (milliseconds)
int moveState = 0;
bool isStopped = false;

bool drop_food = false;
bool pump_water = false;
bool move_drawer = false;
bool prev_move_drawer = true;

const int water_percent_tresh = 40;//treshlhod to stop or open water pump

void setup() {
  Serial.begin(57600);
///////////Foot train component setup//////////////////
  // Set the maximum speed, acceleration factor,
  // initial speed for motor 1
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(50.0);
  stepper1.setSpeed(0);

  // Set pin modes for limit switches
  pinMode(END_SWITCH1, INPUT);
  pinMode(END_SWITCH2, INPUT);

  pinMode(relay_stepper, OUTPUT);
  digitalWrite(relay_stepper, HIGH);
////////////////////////////////////////////////////
myservo.attach(29);  // attaches the servo on pin 29 to the servo object
myservo.write(0);

pinMode(WaterPumpPin, OUTPUT);
digitalWrite(WaterPumpPin, LOW);//Make water pump not running at beginning

  MFL.setMaxSpeed(1000.0);
  MFL.setAcceleration(500.0);

  MFR.setMaxSpeed(1000.0);
  MFR.setAcceleration(500.0);

  MBL.setMaxSpeed(1000.0);
  MBL.setAcceleration(500.0);

  MBR.setMaxSpeed(1000.0);
  MBR.setAcceleration(500.0);

  moveState = 4;  // stop
}

void loop() {
currentMillis = millis();

//Read water percent
water_percent = (analogRead(waterSens_pin)* 100.0)/ 1023.0;
//Serial.println(water_percent);
//delay(50);

  if (Serial.available() > 0) {
    previousMillis1 = millis();
    char command = Serial.read();  // Read the incoming command
    // Execute the command
    switch (command) {
      case 'f':
        moveState = 0;  // Move forward
        if(Auto == true)Auto = false;
        break;
      case 'b':
        moveState = 1;  // Move backward
        if(Auto == true)Auto = false;
        break;
      case 'l':
        moveState = 3;  // Turn left
        if(Auto == true)Auto = false;
        break;
      case 'r':
        moveState = 2;  // Turn right
        if(Auto == true)Auto = false;
        break;
      case 'a':
        if(Auto == false)Auto = true; // Auto mode
        break;
      case 'x':
        if(drop_food == false && move_drawer != true){
          myservo.write(60);
          drop_food = true;
          }
        else{
          myservo.write(0);
          drop_food = false;
          }
          break;
        case 'w':
          if(pump_water == false && move_drawer != true){
            if(water_percent < water_percent_tresh){
            digitalWrite(WaterPumpPin, HIGH);
            pump_water = true;}
          }
          else{
            digitalWrite(WaterPumpPin, LOW);
            pump_water = false;
          }
          break;
        case 't':
          if(pump_water == false && drop_food == false){
            move_drawer = !move_drawer;
            //ft_Speed = - ft_Speed;
          }
          break;
      default:
        moveState = 4;  // stop
        if(Auto == true)Auto = false;
        break;
    }
  }



/////////////////Stop pump if water enough in drawer////////////////////////////////////
if(water_percent > water_percent_tresh){
digitalWrite(WaterPumpPin, LOW);
pump_water = false;}
/*----------------------------------------------------------------------------------------*/

if( prev_move_drawer != move_drawer && moveState == 4){
    // Read the state of the limit switches
    bool reading1 = digitalRead(END_SWITCH1);
    bool reading2 = digitalRead(END_SWITCH2);

    // Handle debounce for limit switch 1
    if (reading1 != lastSwitchState1) {
        lastDebounceTime1 = millis();
    }
    if ((millis() - lastDebounceTime1) > debounceDelay) {
        if (reading1 != switchState1) {
            switchState1 = reading1;
            if (switchState1 == HIGH) {
                stepper1.setSpeed(0);
                motorsRunning = false;
                prev_move_drawer = move_drawer;
                ft_Speed = - ft_Speed;
                //Serial.println("Limit switch 1 triggered, stopping motors");
            }
        }
    }
    lastSwitchState1 = reading1;

    // Handle debounce for limit switch 2
    if (reading2 != lastSwitchState2) {
        lastDebounceTime2 = millis();
    }
    if ((millis() - lastDebounceTime2) > debounceDelay) {
        if (reading2 != switchState2) {
            switchState2 = reading2;
            if (switchState2 == HIGH) {
                stepper1.setSpeed(0);
                motorsRunning = false;
                prev_move_drawer = move_drawer;
                ft_Speed = - ft_Speed;
                //Serial.println("Limit switch 2 triggered, stopping motors");
            }
        }
    }
    lastSwitchState2 = reading2;

    if(prev_move_drawer != move_drawer){
    stepper1.setSpeed(ft_Speed);
    motorsRunning = true;
    digitalWrite(relay_stepper, LOW);
    }
    else{
      //ft_Speed = - ft_Speed;
      digitalWrite(relay_stepper, HIGH);
    }
}
/*---------------------------------------------------------------------------------------*/

/*Serial.print("DL: ");
  Serial.print(distL);
  Serial.print(" ");
Serial.print("DF: ");
  Serial.print(distF); 
  Serial.print(" ");
Serial.print("DR: ");
  Serial.println(distR);*/
//delay(1000);
  //Serial.print(distL);Serial.print(" "); Serial.print(distF); Serial.print(" ");Serial.println(distR);
  if (Auto == true){
    Obstacle_avoid();
  }
  //if (isStopped) {
    if (currentMillis - previousMillis >= stopInterval) {
      previousMillis = currentMillis;
      //isStopped = false;
      //moveState = (moveState + 1) % 4;
      switch (moveState) {
        case 0:
          Move(Speed, Speed, Speed, Speed); // Move forward
          break;
        case 1:
          Move(-Speed, -Speed, -Speed, -Speed); // Move backward
          break;
        case 2:
          Move(Speed, -Speed, Speed, -Speed); // Move right
          break;
        case 3:
          Move(-Speed, Speed, -Speed, Speed); // Move left
          break;
        case 4:
          Move(0, 0, 0, 0);// stop motor
          break;
      }
    }
  //} 
 /* else {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      Move(0, 0, 0, 0);
      isStopped = true;
    }
  }*/
    // Move the motors one step, ensuring they run simultaneously if they should be running
    if (motorsRunning) {
        stepper1.runSpeed();
    }
  // Run the steppers
  MFL.runSpeed();
  MFR.runSpeed();
  MBL.runSpeed();
  MBR.runSpeed();
}

void Obstacle_avoid(){
if(millis() - previousMillis1 > 100)   {moveState = 4;}

if (currentMillis - previousMillis1 >= stopInterval1) {
  previousMillis1 = currentMillis;  
  distL = L_ultra.ping_cm();
  distR = R_ultra.ping_cm();
  distF = F_ultra.ping_cm();
  }
  if(distF > obstacle){
    moveState = 0;//forward
  }
  else{
    int cmp = distL - distR;

    if(cmp > 0){
      if(distL > obstacle){
        //Turn left
        moveState = 3;
      }
      else{
        moveState = 4;
      }
    }
    else{
      if(distR > obstacle){
        //Turn right
        moveState = 2;
      }
      else{
        moveState = 4;
      }
    }
    
  }
}
void Move(int v1, int v2, int v3, int v4) {
  // Set speed for each motor
  MFL.setSpeed(v1);
  MFR.setSpeed(v2);
  MBL.setSpeed(v3);
  MBR.setSpeed(v4);
}
