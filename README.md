#include <QTRSensors.h>
  #define left_motor_positive 4
  #define left_motor_negative 3
  #define right_motor_positive 6
  #define right_motor_negative 5
  #define en1 10
  #define en2 11

#include "digitalWriteFast.h"
/**
 * Hardware pin defines
 */
// #define BOARD UKMARSBOT_V1

// const int MOTOR_LEFT_DIR = 7;
// const int MOTOR_RIGHT_DIR = 8;
// const int MOTOR_LEFT_PWM = 9;
// const int MOTOR_RIGHT_PWM = 10;
// const int LED_RIGHT = 6;
// const int LED_LEFT = 11;
// const int EMITTER = 12;
// const int SENSOR_RIGHT_MARK = A0;
// const int SENSOR_1 = A1;
// const int SENSOR_2 = A2;
// const int SENSOR_3 = A3;
// const int SENSOR_4 = A4;
// const int SENSOR_LEFT_MARK = A5;
// const int FUNCTION_PIN = A6;
// const int BATTERY_VOLTS = A7;
// /**/

/*
 * Global variables
 */
const int COUNTS_PER_ROTATION = 28;
const float GEAR_RATIO = 30;
const float WHEEL_DIAMETER = 47.5f;
const float WHEEL_SEPARATION = 150.2;

const float MM_PER_COUNT = (PI * WHEEL_DIAMETER) / (2 * COUNTS_PER_ROTATION * GEAR_RATIO);
const float DEG_PER_COUNT = (360.0 * MM_PER_COUNT) / (PI * WHEEL_SEPARATION);
/*
 * Global variables
 */
volatile int32_t encoderLeftCount;
volatile int32_t encoderRightCount;
uint32_t updateTime;
uint32_t updateInterval = 100;  // in milliseconds

  

int check = 0;
int attempt = 0;
int rightState =0;

int w = 250;
int ww = 500;
int b = 800;
int bb = 1400;
int c = 500;

#define led 13


QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];




int initial_motor_speed = 190;//100
int rotating_speed = 100;//60
int forward_speed = 200;//100
int brake= 60;
int right_motor_speed = 0; //for the speed after PID control
int left_motor_speed = 0;


int error; 
float kp =3;//proportional constant 0.05
float ki = 0;
float kd = 50; //5
float P,I,D,previousError=0;
int pid_value;

char mode;
int Status = 0;
int buttom_reading;

int ObstaclePin=10;
int ObstacleRead;
void led_signal(int times);

void calculatePID();
void PIDmotor_control();
   uint16_t position;

void readIRvalue();     //to read sensor value and calculate error as well mode
void Set_motion();

void dryrun();
void actualrun();

void recIntersection(char);
char path[100] = "";
unsigned char pathLength = 0; // the length of the path
int pathIndex = 0;
// void setmotionactual();
// void mazeTurn (char dir);

void forward(int spd1, int spd2);
void left(int spd);
void right(int spd);
void stop_motor();
void goAndTurnLeft();
void maze_end();
void move_inch();
void backward(int spd1, int spd2);


void setup() 



{ 
 Serial.begin(9600);
  // put your setup code here, to run once:
  DDRD = B01111000;
  DDRB = B0101100;
  // DDRD &= !B0100001;
 PCICR |= B00000101;
  PCMSK0 |= B00000010;
  PCMSK2 |= B00000100;
    encoderLeftCount = 0;
    encoderRightCount = 0;
    
  delay(500);
  pinMode(13, OUTPUT);

   qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5}, SensorCount);
  qtr.setSamplesPerSensor(12);
  // qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  PORTB |= B00100000;  // turn on Arduino's LED to indicate we are in calibration mode
 left(80);
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  PORTB &= !B00100000;
  // print the calibration minimum values measured when emitters were on
 stop_motor();
    delay(3000);
}


ISR(PCINT0_vect) {
  // static bool oldB = 0;
  // bool newB = bool(digitalReadFast(ENCODER_LEFT_B));
  // bool newA = bool(digitalReadFast(ENCODER_LEFT_CLK)) ^ newB;
  // if (newA == oldB) {
  //   encoderLeftCount--;
  // } else {
    encoderLeftCount++;
  // }
  // oldB = newB;
}

ISR(PCINT2_vect) {
//   static bool oldB = 0;
//   bool newB = bool(digitalReadFast(ENCODER_RIGHT_B));
//   bool newA = bool(digitalReadFast(ENCODER_RIGHT_CLK)) ^ newB;
//   if (newA == oldB) {
    
// encoderRightCount--;
//   } else {
    encoderRightCount++;
  // }
  // oldB = newB;
}



void loop() 
{   
//   forward(100,100);
// }
  dryrun();
  // move_inch();
  // // stop_motor();
  // delay(1000);  
    // encoderSum = encoderRightCount + encoderLeftCount;
    // encoderDifference = encoderRightCount - encoderLeftCount;
    // float distance = MM_PER_COUNT * encoderSum;
    // float angle = DEG_PER_COUNT * encoderDifference;

    // Serial.print(F("EncoderSum: "));
    // Serial.print(encoderSum);
    // Serial.print(F(" = "));
    // Serial.print(distance);
    // Serial.print(F(" mm    "));

    // Serial.print(F("EncoderDifference: "));
    // Serial.print(encoderDifference);
    // Serial.print(F(" = "));
    // Serial.print(angle);
    //  Serial.print(encoderRightCount);
    //  Serial.print(F("    "));
    // Serial.print(encoderLeftCount);
    




    // Serial.println();
      // actualrun();
//    if(Status==0){
//   dryrun();
// }
//     else{
//       actualrun();
//     } 
  }


void led_signal(int times)
{
  for (int i = 0; i <= times; i=i+1)
  {
    PORTB |= B00100000;
    delay(100);
    PORTB &= !B00100000;
    delay(100);
  }
}

//dryrun begins------------------------------------------------------------------------------------------------------------------
void dryrun()
{
     readIRvalue();
     set_motion();
  
}
//ReadIRvalue begins----------------------------------------------------------------------------------------------------------------------------------
void readIRvalue()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
    
    error = 2500 - position;
  //   for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  // mode = 'F';

    

//--------------------------------------------------------------------------------------
  if (sensorValues[0] > b && sensorValues[1] > b && sensorValues[2] > b && sensorValues[3] > b && sensorValues[4] > b && sensorValues[5] > b)
  {
    mode = 'N';  //NO LINE
    // Serial.println("b");
    error = 0;
  }

  //--------------------------------------------------------------------------------------
  else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w)
  {
    mode = 'S';//Stop Condition
    error = 0;
  }
  //   else if ( sensorValues[0] < w+100 && sensorValues[1] < w+100 && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w+100 && sensorValues[5] < w+100)
  // {
  //   mode = 'S';//Stop Condition
  //   error = 0;
  // }
  
  //--------------------------------------------------------------------------------------

  else if (sensorValues[0] > b  && sensorValues[2] < w && sensorValues[3] < w  && sensorValues[5] < w )
  {
    mode = 'R';//90 degree turn
    error = 0;
    // Serial.println("R");
  }
  //   else if (sensorValues[0] > b && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w )
  // {
  //   mode = 'R';//90 degree turn
  //   error = 0;
  //   // Serial.println("R");
  // }
  //   else if (sensorValues[0] > b && sensorValues[1] > w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] < w && sensorValues[5] < w )
  // {
  //   mode = 'R';//90 degree turn
  //   error = 0;
    // Serial.println("R");
  // }
  // else if ( sensorValues[0] > b && sensorValues[1] > b && sensorValues[4] < w && sensorValues[5] < w)
  // {
  //   mode = 'R';//90 degree turn
  //   Serial.println("R");
  //   error = 0;
  // }


  

  else if ( sensorValues[0] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[5] > b)
  {
    mode = 'L'; //90 degree turn
    // Serial.println("l");
    error = 0;
  }
  //   else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[2] < w && sensorValues[3] < w && sensorValues[4] > b && sensorValues[5] > b)
  // {
  //   mode = 'L'; //90 degree turn
  //   // Serial.println("l");
  //   error = 0;
  // }
  // else if ( sensorValues[0] < w && sensorValues[1] < w && sensorValues[4] > b && sensorValues[5] > b)
  // {
  //   mode = 'L'; //90 degree turn
  //   error = 0;
  //   Serial.println("l");
  // }
  else{
    mode = 'F';
  }
  
// if((sensorValues[2]  + sensorValues[3]) < ww){

//   if((sensorValues[0] + sensorValues[1]) < ww){
//     if((sensorValues[4] + sensorValues[5]) < ww){
//         mode = 'S';//Stop Condition
//         error = 0;
//     }
//     else if((sensorValues[4] + sensorValues[5]) > bb){
//         mode = 'L';//Stop Condition
//         error = 0;
//     }
//     else{
//         mode = 'F';//Stop Condition
//     }   
//   }
  
//   else if((sensorValues[0] + sensorValues[1]) > bb){
        
//     if((sensorValues[4] + sensorValues[5]) < ww){
//         mode = 'R';//Stop Condition
//         error = 0;
//     }
//     else{
//         mode = 'F';//Stop Condition
//         // error = 0;
//     }  
//   }

//   else{
//     mode = 'F';
//   }  
// }

// else if((sensorValues[2] + sensorValues[3]) > bb){
//    mode = 'N';  //NO LINE
//     // Serial.println("b");
//     error = 0;
// }


// else{
//  mode = 'F';  
// }
// // mode = 'F';
}



//set motion begins------------------------------------------------------------------------------------------------------------------------------
void set_motion()
{
  switch (mode)
  {
    case 'N':
      backward(initial_motor_speed, initial_motor_speed);
      delay(20);
      move_inch();
      goAndTurnRight();
      recIntersection('B');
      break;
    case 'S':
      // move_inch();
      //  int startA = encoderRightCount;
      // int startB =   encoderLeftCount;
      // while((startA + startB) < 600 ){
      //     startA = encoderRightCount;
      //     startB =   encoderLeftCount;
      // }   
      backward(initial_motor_speed, initial_motor_speed);
      delay(20);
      move_inch();
      readIRvalue();
      
      if (mode == 'S')
      {
       maze_end();
      }
      else
      {
        goAndTurnLeft();
        recIntersection('L');
      }
      break;
    case 'R':
      
            
      // stop_motor();
      backward(initial_motor_speed, initial_motor_speed);
      delay(20);
      move_inch();      
      readIRvalue();
      
      if (mode == 'F')
     {
        recIntersection('S');
      }
     else
      {
          goAndTurnRight();
      }
      break;
    case 'L':
    backward(initial_motor_speed, initial_motor_speed);
      delay(20);
      move_inch();
      stop_motor();
      readIRvalue();
      
      if (mode == 'F')
     {
        goAndTurnLeft();
        recIntersection('L');
      }
     else
      { goAndTurnLeft();
      }
      break;
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
  }
}

//-----------------------------------------------------------------------------------------------------------
void move_inch()
{
   uint32_t start = encoderRightCount + encoderLeftCount;

  while((encoderRightCount + encoderLeftCount) - (start) < 100 ){
          forward(forward_speed, forward_speed);
      }   
  backward(forward_speed, forward_speed);
  delay(20);
  stop_motor();
}
//----------------------------------------------------------------------------------------------------------

void stop_motor()
{
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  PORTD &= !B01111000;
}
//-----------------------------------------------------------------------------------------------------------
void goAndTurnLeft()
{ 
    previousError = 0;
  left(rotating_speed);
  delay(10);
  do
  {
    left(rotating_speed);
    readIRvalue();
  } while (mode == 'F');
  // left(rotating_speed);
  // delay(30);
  do
  {
    left(rotating_speed);
    readIRvalue();
  } while (mode != 'F');
  //  do
  // {
  //   left(rotating_speed);
  //   readIRvalue();
  // } while (position  < 1000);
}

//------------------------------------------------------------------------------------------------
void goAndTurnRight()
{previousError = 0;
  right(rotating_speed);
  delay(10);
  do
  {
    right(rotating_speed);
    readIRvalue();
  }while (mode == 'F');
// right(rotating_speed);
//   delay(30);
  do
  {
    right(rotating_speed);
    readIRvalue();
  }while (mode != 'F');
// do

 
}
//-----------------------------------------------------------------------------------------------
void maze_end()
{
  Status++;
  stop_motor();
  led_signal(20);
  delay(10000);
}
//------------------------------------------------------------------------------------------------
void calculatePID()
{
  
  P = error;
  I = I + error;
  D = error-previousError;
  pid_value = (kp*P) + (ki*I) + (kd*D);
  previousError = error;
  
}
//-----------------------------------------------------------------------------------------------
void PIDmotor_control()
{
  right_motor_speed = initial_motor_speed - pid_value;
  left_motor_speed = initial_motor_speed + pid_value;
  right_motor_speed = constrain(right_motor_speed, 0, initial_motor_speed);
  left_motor_speed = constrain(left_motor_speed, 0, initial_motor_speed);
  forward(left_motor_speed, right_motor_speed);
  
}

//-------------------------------------------------------------------------------------------------------
void forward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
  PORTD = B01010000;
  // PORTD &= !B00010100;
  PORTB &= !B00100000;
}

void backward(int spd1, int spd2)
{
  analogWrite(en1, spd1);
  analogWrite(en2, spd2);
    PORTD =  B00101000;
  PORTB &= !B00100000;
}

void left(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  PORTD =  B01001000;
  PORTB |= B00100000;
}

void right(int spd)
{
  analogWrite(en1, spd);
  analogWrite(en2, spd);
  PORTD = B00110000;
  PORTB |= B00100000;
 }
//--------------------------------------------------------------------------------------


void actualrun(void)
{
  while (Status ==1)
  {
    readIRvalue();
    setmotionactual();
  }
}

//actual runfunctions--------------------------------------------------------------------------------------------
void setmotionactual()
{
  switch (mode)
  {
    case 'F':
      calculatePID();
      PIDmotor_control();
      break;
    case 'S':
      // if (pathIndex >= pathLength)
      //   maze_end();
      // else
      // {
  

      move_inch();
      readIRvalue();
  
      if (mode == 'S')
      {
        maze_end();
      }
      else
      {
      mazeTurn (path[pathIndex]);
        pathIndex++;
      }
       
      // }
      break;
    case 'L':
      // if (pathIndex >= pathLength)
      //   maze_end();
      // else
      // {

      move_inch();
      readIRvalue();
    
      if (mode == 'F')
     { mazeTurn (path[pathIndex]);
        pathIndex++;
      }
     else
      {
          goAndTurnLeft();
      }
      // }
      break;
    case 'R':
      // if (pathIndex >= pathLength)
      //   maze_end();
      // else
      // {

      move_inch();
      readIRvalue();
 
      if (mode == 'F')
     {
        mazeTurn (path[pathIndex]);
        pathIndex++;
      }
     else
      {
          goAndTurnRight();
      }
    
      // }
      break;
  }

}

void recIntersection(char Direction)
{
  path[pathLength] = Direction;
  // Serial.println(path[pathLength]); // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
}

void simplifyPath()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path[pathLength - 2] != 'B'){
    return;}
  int totalAngle = 0;
  int i;
  for (i = pathLength - 1; i > pathLength - 4; i--)
  {
    switch (path[i])
    {
      case 'R':
        totalAngle += 270;
        break;
      case 'L':
        totalAngle += 90;
        break;
      case 'B':
        totalAngle += 180;
        break;
         case 'S':
        totalAngle += 0;
        break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch (totalAngle)
  {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'L';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = 'R';
      break;
  }
  // The path is now two steps shorter.
  pathLength -= 2;
}

void mazeTurn (char dir)
{
  switch (dir)
  {
    case 'L': // Turn Left
      goAndTurnLeft();
      break;

    case 'R': // Turn Right
      goAndTurnRight();
      break;

    case 'B': // Turn Back
      goAndTurnLeft();
      break;

    case 'S': // Go Straight
      move_inch();
      break;
  }
}
