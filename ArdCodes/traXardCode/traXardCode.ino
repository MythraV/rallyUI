
/*
    Code version 2.x , Traxxas RallyCar Arduino code for RallyCar project
    The arduino has the following functions, read IMU data and write to NVIDIA,
    Read velocity and steer inputs from NVIDIA and write to servo/ESC,
    Read servo trim and truncate the throttle/gas accordingly.

    Author: Mythra Varun
    Version 2.2,   Date: 11/6/2017,  Time: 14:05 IMU flag added
*/


#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

// Define constants used
#define Led_Pin 13
#define InterruptTrimPin 2

// Servo objects for the steering and throttle/gas

Servo steeringServo;    // The servo handle for steering
Servo eSC;              // The servo handle for ESC
Servo test;              // Test sevo to check IMU, comment/remove before flash

// The variables for storing the steering and throttle velocities received from NVIDIA
double steerVel = 1500;            // Initialzing steering angle (Called velocity to confuse)
double throtVel = 1500;            // Initializing throttle velocity

//Maximum and minimum values for steering and throttle
int maxSteer = 1950;          // Max and min for servo is 1000-2000
int minSteer = 1050;           // We are setting the maximum and minumim value for truncating the servo range
int maxThrottle = 2000;       // The same settings for the Throttle/gas ESC
int minThrottle = 1000;       // 1500 + is forward 1000-1500 is reverse for throttle
int trimThrottle = 1500;  // The trim value where the throttle is truncated, starts at neutral which is 0 for throttle

int prevTrim = trimThrottle;  // The previous trim value before change, used to reset throttle velocity if trim changes.

int minTrim = 1380;       // Minimum servo trim pulse width, Knob at couter-clockwise limit
int maxTrim = 1620;       // Maximum servo trim pulse width, Knob at clockwise limit


// Variables associated with reading the trim signal.
bool trimFlag = true;        // If the trim interrupt has been called and a new PWM value has been set
volatile long pulseWidth = 0;  // The width of pulse in us(microseconds)
volatile long pulseStartTime  = 0;  // The time at beginning of the pulse (on rising edge) in us

// Variables for setting frequency for serial writing the IMU data;
long prevTime = 0;    // The time at beginning of previous serial write, in ms(milliseconds)
int interval = 6;     // Interval between writes in ms, frequency = 1000/interval (approx.)

unsigned long currentTime;

//IMU flag
int imu_flag = 0;

/* ------------------------------- Mapping and sign function --------------------------------*/
int fmap (int toMap, int in_min, int in_max, int out_min, int out_max) {
  return (int)((toMap - in_min) * ((1.0 * out_max - out_min) / (1.0 * in_max - in_min)) + out_min);
}

static char sgn(int16_t val) {
  if (val >= 0) return ('+');
  else
    return ('-');
}



/* ---------------------------------- Setup ----------------------------------------*/
void setup() {
  //I2C setup

  Wire.begin();            // join I2C bus
  Serial.begin(115200);    // initialize serial communication
  accelgyro.initialize();  // initialize IMU handle

  steeringServo.attach(6); // Steer servo connected to pin 6
  eSC.attach(7);           // ESC connected to pin 7
  //Initialize the servo and ESC
  //Centering the servo and ESC
  steeringServo.writeMicroseconds(1500);
  eSC.writeMicroseconds(1500);
  delay(1000);

  //verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  //Setup the interrput pin to read the trim signal;
  pinMode(InterruptTrimPin, INPUT_PULLUP);

  // External hardware interrupt is attached to the pin and the interrupt is set to trigger on 'CHANGE'
  // in state of pin. On trigger the Interrupt service routing trimPulseTimer is called.
  attachInterrupt(digitalPinToInterrupt(InterruptTrimPin), trimPulseTimer, CHANGE);

//  pinMode(Led_Pin, OUTPUT);  // configure LED pin
}

/*------------------------------ The main loop begins -----------------------------------*/

void loop() {
  /*--------------------------------- IMU READ ------------------------------------*/
  // Reading from IMU at the specified intervals
  // The IMU read is timed loop run at specific frequency, so some debug print code
  // and trim code is included in this low frequency loop;
  //noInterrupts();
  currentTime = millis();

  //if (currentTime - prevTime >= interval)
  if (currentTime - prevTime >= interval && imu_flag == 1)
  {

    prevTime = currentTime;
    // UNCOMMENT for IMU data to be read and writtent to Serial port, check issue of interference

    //detachInterrupt(digitalPinToInterrupt(InterruptTrimPin));
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device
    char accBuf[30];    // The buffer to store the Serial print data
    // Format in which IMU data is sent
    sprintf(accBuf, "I%c%05d%c%05d%c%05dU", sgn(ax), abs(ax), sgn(ay), abs(ay), sgn(az), abs(az));
    //attachInterrupt(digitalPinToInterrupt(InterruptTrimPin), trimPulseTimer, CHANGE);
    // Write IMU values to serial port
    Serial.println(accBuf);

    // For debug of ISR
    /*
          Serial.print(pulseWidth);
          Serial.print(" ");*/
    //Serial.println(trimThrottle);

  }

  /*---------------------------------------- TRIM READ -------------------------------*/
  // Reading servo Trim and adjusting the maxThrottle value
  if (trimFlag)
  {
    // trimThrottle truncates the max throttle value,
    // 1370 and 1610 are the min and max values of steering servo at the extremities ...
    // of steering trim knob.
    // 1500 is the base forward value for ESC / servo

    trimThrottle = fmap(pulseWidth, minTrim, maxTrim, 1500, maxThrottle);
    if (trimThrottle < 1500)
      trimThrottle = 1500;

    //reset flag
    trimFlag = false;

    // For debug
    //Serial.println(pulseWidth);
  }

  // Trim - truncate the Throttle if trim changes, i.e. trim flag set
  if (abs(prevTrim - trimThrottle) > 20)
  {
    //For debug of Trim
    /*
          Serial.print("*");
          Serial.print(trimThrottle);
          Serial.print(" ");
          Serial.print(prevTrim);
          Serial.print(" ");
          Serial.println(throtVel);
    */
    prevTrim = trimThrottle;
    if (throtVel > prevTrim)
    {
      eSC.writeMicroseconds(prevTrim);      // Set throttle esc value as trim value if trim value falls below set velocity
    }
    else
      eSC.writeMicroseconds(throtVel);  // Set throttle esc value back to set velocity if trim value is increased beyond set velocity
  }

  /*--------------------------------------- NVIDIA READ -------------------------------*/
  // Reading velocities from NVIDIA

  //Remove following two lines 19:15 10/13/17
  //  steerVel = 0.0;  //Reset steering velocity value
  //  throtVel = 0.0;  //Reset throttle velocity value
char rxBuf[11] ;  //Read buffer, reset buffer size to 11 based on write from nvidia  
  if (Serial.available() >= 4)         //Each data packet is 7 characters, 3 - throttle, 3 - steering
  {
    char readChar = Serial.read();
    if (readChar == 'I')
    {
      //Serial.println("In");
      if (Serial.available() && Serial.read() == 'M')
      { readChar = Serial.read();
        readChar = Serial.read();
        if (readChar == '0')
          { imu_flag = 0;
            //Serial.println("IMU off");
            }
        else if (readChar =='1')
          { imu_flag = 1;
            //Serial.println("IMU on");
            }
      }
    }
   
    if (readChar == 'A')
      {
        while (Serial.available()<10);
        for (int i = 1; i < 11; i++)        // Read 6 characters iteratively
          rxBuf[i] = Serial.read();
        steerVel = 1000 * ((int)rxBuf[2] - 48) + 100 * ((int)rxBuf[3] - 48) + 10 * ((int)rxBuf[4] - 48) + ((int)rxBuf[5] - 48);
        throtVel = 1000 * ((int)rxBuf[7] - 48) + 100 * ((int)rxBuf[8] - 48) + 10 * ((int)rxBuf[9] - 48) + ((int)rxBuf[10] - 48);
        if (rxBuf[1] == '-')
          steerVel *= -1;
        if (rxBuf[6] == '-')
          throtVel *= -1;
        //Serial.println(rxBuf);
        // For debug
        //Serial.println(steerVel);

        steerVel = fmap(steerVel, -2048, 2047, minSteer, maxSteer);  // Mapping the received value to limits
        steeringServo.writeMicroseconds(steerVel);                        // Set steer servo value

        // For debug
        //Serial.println(steerVel);

        // For debug
        //Serial.print(throtVel);
        // First half of throttle is reverse and second half is forward
        if (throtVel >= 0)
        {
          // Mapping the received value to limits
          throtVel = fmap(throtVel, 0, 2047, 1500, maxThrottle);
        }
        else if (throtVel < 0)
        {
          // Mapping the received value to limits
          // Uncomment following line and comment the line after if going reverse is needed
          // throtVel = fmap(throtVel, 0, 999, minThrottle, maxThrottle);
          throtVel = 1500;                //Comment if reverse needed
        }

        // The trimThrottle value truncates the maximum throttle possible.
        if (throtVel > prevTrim)
          eSC.writeMicroseconds(prevTrim);     // Set throttle esc value as trim value if set velocity is greater than trim
        else
          eSC.writeMicroseconds(throtVel);     // Set throttle esc value as set velocity if it is within trim value


        // For debug
        //Serial.println(throtVel);
      }
  }

  // For debug
  // blink LED to indicate activity
  //blinkState = !blinkState;
  //digitalWrite(Led_Pin, blinkState);
}


void trimPulseTimer() {
  if (digitalRead(InterruptTrimPin) == HIGH)
  {
    pulseStartTime = micros();
  }
  else
  {
    pulseWidth = micros() - pulseStartTime;
    trimFlag = true;
  }
}

