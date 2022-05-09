#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <NRF24.h>
#include <DRV8835MotorShield.h>
#include <Pixy2I2C.h>


/*
   This example uses the DRV8835MotorShield library to drive each motor with the
   Pololu DRV8835 Dual Motor Driver Shield for Arduino forward, then backward.
   The yellow user LED is on when a motor is set to a positive speed and off when
   a motor is set to a negative speed.
*/
/*
  Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
   =======
*/

#define shooterMOT A1

DRV8835MotorShield motors(7, 5, 8, 6);
NRF24 radio;
Pixy2I2C pixyCam;

bool tx;

const int highSpeed = 400;
const int lowSpeed = 150;

const int ultraL = 3;
const int ultraR = 4;
float distance;
volatile unsigned long timeLast = millis();
volatile float leftDist, rightDist;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
//set address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


enum States {initial, followRadio, followPixy, findGoal, shoot};
States driveState = initial;


void setup()
{
  Serial.begin(115200);

  pixyCam.init();
  pixyCam.changeProg("color_connected_components");
  pixyCam.ccc.getBlocks();

  radio.begin(9, 10);

  // Pin 7 sets the mode (Sender or Receiver). Connect to GND on the sender
  pinMode(7, INPUT_PULLUP);
  tx = !digitalRead(7);
  radio.setAddress(0xD2);   ////0xD2 if greenTeam, 0xD3 is blueTeam
  radio.startListening();

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  pinMode(ultraL, OUTPUT);
  pinMode(ultraR, OUTPUT);
  //pinMode(LED_PIN, OUTPUT);
  //pinMode(shooterMOT, OUTPUT);
  //digitalWrite(shooterMOT, LOW);

  leftDist = measureDistance(ultraL);
  if(leftDist == 0){
    leftDist = 100;
  }
  rightDist = measureDistance(ultraR);
  if(rightDist == 0){
    rightDist = 100;
  }
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  //motors.flipM2(true);
}



void loop() {
  double yaw, pitch, roll;
  float minDist = 8;

  switch (driveState) {

    /////////////////////////////////////////
    case initial:
    {
      ////do nothing, wait for START signal
      Serial.println("initial");

      char buf[32];
      if (radio.available())
      {
        uint8_t numBytes = radio.read(buf, sizeof(buf));
        Serial.print("Received: ");
        Serial.println(buf);
      }
      delay(3000);
      
      pixyCam.ccc.getBlocks();
      if (pixyCam.ccc.numBlocks) {
        Serial.println("got blocks!");
        if (pixyCam.ccc.blocks[0].m_signature == 1) {
          driveState = followPixy;
        }
      }
      else {
        driveState = followRadio;
      }

      break;
    }

    /////////////////////////////////////////
    case followRadio:
    {
      Serial.println("followRadio");
      /*if (radio.available())
        {
        char buf[32];
        uint8_t numBytes = radio.read(buf, sizeof(buf));
        Serial.print("Received: ");
        Serial.println(buf);
        }

        /////////decode buf info on puck position and drive towards it??????
        //buf will be array form [r1x,r1y, r2x,r2y, px,py];

        //get the robot's yaw angle from IMU, this is angle in rink ////////WHICH YOU MUST CALIBRATE IN SAME FRAME AS THE RADIO COORDINATES
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

        yaw = orientationData.orientation.x;
        pitch = orientationData.orientation.y;
        roll = orientationData.orientation.z;
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);

        //calculate angle from robot to puck
        float deltaX = px - r1x;
        float deltaY = py - r1y;
        float theta_desired = atan(deltaY / deltaX) * (180 / 3.1416);

        //turn robot such that it is pointing that angle and run it forward
        if (abs(theta_desired) < 10) {
        forward();
        delay(300);
        }
        else if (theta_desired > 10) {
        theta_desired = 360 - theta_desired;
        while (abs(yaw - theta_desired) > 10) {
          turnLeft;
          delay(100);

          sensors_event_t orientationData;
          bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

          yaw = orientationData.orientation.x;
          pitch = orientationData.orientation.y;
          roll = orientationData.orientation.z;
          uint8_t system, gyro, accel, mag = 0;
          bno.getCalibration(&system, &gyro, &accel, &mag);
        }

        forward();
        delay(300);
        }
        else if (theta_desired < -10) {
        theta_desired = -1 * theta_desired;
        while (abs(yaw - theta_desired) > 10) {
          turnRight;
          delay(100);

          sensors_event_t orientationData;
          bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

          yaw = orientationData.orientation.x;
          pitch = orientationData.orientation.y;
          roll = orientationData.orientation.z;
          uint8_t system, gyro, accel, mag = 0;
          bno.getCalibration(&system, &gyro, &accel, &mag);
        }

        forward();
        delay(300);
        }*/

      int vol = analogRead(A0);
      pixyCam.ccc.getBlocks();
      if (vol < 100) {
        driveState = shoot;
      }
      else if (pixyCam.ccc.numBlocks) {
        //Serial.println("got blocks!");
        if (pixyCam.ccc.blocks[0].m_signature == 1) {
          driveState = followPixy;
        }
      }
      else {
        driveState = followRadio;
      }

      break;
    }

    /////////////////////////////////////////
    case followPixy:
    {
      Serial.println("followPixy");
      int puck_x;

      if (pixyCam.ccc.numBlocks) {
        if (pixyCam.ccc.blocks[0].m_signature == 1) {
          puck_x = pixyCam.ccc.blocks[0].m_x;

          if (puck_x - 200 > 0) {
            turnRight();
            delay(50);
          }
          if (puck_x - 120 < 0) {
            turnLeft();
            delay(50);
          }
          if ((puck_x - 120 > 0) && (puck_x - 200 < 0)) {
            forward();
            delay(50);
          }
        }
      }


      pixyCam.ccc.getBlocks();
      int vol = analogRead(A0);
      if (vol < 100) {
        //driveState = findGoal;
        driveState = shoot;
      }
      else if (pixyCam.ccc.numBlocks) {
        //Serial.println("got blocks!");
        if (pixyCam.ccc.blocks[0].m_signature == 1) {
          driveState = followPixy;
        }
      }
      else {
        driveState = followRadio;
      }

      break;
    }

    /////////////////////////////////////////
    case findGoal:
    {
      Serial.println("findGoal");
      ////this should drive the robot towards goal using the radio, should have method for detecting if it can shoot on goal

      int vol = analogRead(A0);
      pixyCam.ccc.getBlocks();
      if (vol<100){//DETECT SHOT ON GOAL) {
      driveState = shoot;
      }
      else if (vol < 100) {
        driveState = findGoal;
      }
      else if (pixyCam.ccc.numBlocks) {
        //Serial.println("got blocks!");
        if (pixyCam.ccc.blocks[0].m_signature == 1) {
          driveState = followPixy;
        }
      }
      else {
        driveState = followRadio;
      }
      break;
    }

    /////////////////////////////////////////
    case shoot:
    {
      Serial.println("shoot");
      ////spin the motor of the shooting wheel (at max speed)
      //digitalWrite(shooterMOT, HIGH);
      //delay(200);
      //digitalWrite(shooterMOT, LOW);

      analogWrite(shooterMOT, 255);
      delay(1000);
      analogWrite(shooterMOT, 0);

      int vol = analogRead(A0);
      pixyCam.ccc.getBlocks();
      if (vol < 100) {
        driveState = shoot;
      }
      else if (pixyCam.ccc.numBlocks) {
        //Serial.println("got blocks!");
        if (pixyCam.ccc.blocks[0].m_signature == 1) {
          driveState = followPixy;
        }
      }
      else {
        driveState = followRadio;
      }
      break;
    }
  }


    /////////////////////////////////////////
    /////////////////////////////////////////
    /////////////////////////////////////////
/*
  Serial.print(leftDist);
  if (leftDist > minDist) {
    //Serial.println("detect LEFT");
    avoidLeft();
  }

  //Serial.println(leftDist);
  leftDist = measureDistance(ultraL); //change to CM!!!!!!!
  if(leftDist == 0){
    leftDist = 100;
  }
  
  Serial.print("      ");
  Serial.println(rightDist);
  if (rightDist > minDist) {
    //Serial.println("detect Right");
    avoidRight();
  }

  //Serial.println(rightDist);
  rightDist = measureDistance(ultraR); //change to CM!!!!!!!
  rightDist = measureDistance(ultraR);
  if(rightDist == 0){
    rightDist = 100;
  }*/
}

    /////////////////////////////////////////
    /////////////////////////////////////////
    /////////////////////////////////////////



void forward() {
  //digitalWrite(LED_PIN, HIGH);
  //Serial.println("forward");
  motors.setM1Speed(highSpeed);
  motors.setM2Speed(highSpeed);
}

void backward() {
  //digitalWrite(LED_PIN, LOW);
  //Serial.println("backward");
  motors.setM1Speed(highSpeed * (-1));
  motors.setM2Speed(highSpeed * (-1));
}

void turnLeft() {
  //Serial.println("left");
  //digitalWrite(LED_PIN, HIGH);
  motors.setM1Speed(lowSpeed);
  motors.setM2Speed(highSpeed);
}

void turnRight() {
  //Serial.println("right");
  //digitalWrite(LED_PIN, HIGH);
  motors.setM1Speed(highSpeed);
  motors.setM2Speed(lowSpeed);
}

void pttrnLeft() {
  //Serial.println("left in place");
  motors.setM1Speed((-1)*highSpeed );
  motors.setM2Speed(highSpeed);
}

void pttrnRight() {
  //Serial.println("right in place");
  motors.setM1Speed(highSpeed);
  motors.setM2Speed(highSpeed * (-1));
}

void stop() {
  //Serial.println("stop");
  //digitalWrite(LED_PIN, LOW);
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

float measureDistance(int signal) {
  // set pin as output so we can send a pulse
  pinMode(signal, OUTPUT);
  // set output to LOW
  digitalWrite(signal, LOW);
  delayMicroseconds(5);

  // now send the 5uS pulse out to activate Ping)))
  digitalWrite(signal, HIGH);
  delayMicroseconds(5);
  digitalWrite(signal, LOW);

  // now we need to change the digital pin
  // to input to read the incoming pulse
  pinMode(signal, INPUT);

  // finally, measure the length of the incoming pulse
  float pulseduration = pulseIn(signal, HIGH);
  float meas_dist = pulseduration / 2 * 343 / 10000;
  return meas_dist;
}

///unchanged from lab 3 modify to work with ULTRASOUND
void avoidLeft() {
  //Serial.println("avoid left");
  float distL = measureDistance(ultraL);

  if ((millis() - timeLast > 400) && (abs(distL - leftDist) > 0.1)) {
    //Serial.println("avoiding Left");
    backward();
    delay(50);

    turnRight();
    timeLast = millis();
    leftDist = distL;
    delay(50);
  }
}

void avoidRight() {
  //Serial.println("avoid right");
  float distR = measureDistance(ultraR);

  if ((millis() - timeLast > 400) && (abs(distR - rightDist) > 0.1)) {
    //Serial.println("avoiding Right");
    backward();
    delay(50);

    turnLeft;
    timeLast = millis();
    rightDist = distR;
    delay(50);
  }
}
