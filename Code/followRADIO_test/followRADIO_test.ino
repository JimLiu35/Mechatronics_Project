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

//blue team preset
//#define ident_char 'b'
//#define channel 0xF2
//#define goal_pos 2100

//green team preset
#define ident_char 'g'
#define channel 0xAA
#define goal_pos 300

#define shooterMOT 2 //A1

DRV8835MotorShield motors(7, 5, 8, 6);
NRF24 radio;
Pixy2I2C pixyCam;

bool tx;
float r1x, r1y, r2x, r2y, px, py;

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


enum States {initial, followRadio, followPixy, findGoal, shoot, gameEnd};
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
  radio.setAddress(channel);   ////0xD2 if greenTeam, 0xD3 is blueTeam
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
  pinMode(shooterMOT, OUTPUT);
  digitalWrite(shooterMOT, LOW);

  leftDist = measureDistance(ultraL);
  if (leftDist == 0) {
    leftDist = 100;
  }
  rightDist = measureDistance(ultraR);
  if (rightDist == 0) {
    rightDist = 100;
  }
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  //motors.flipM2(true);
}



void loop() {
  double yaw, pitch, roll;
  float minDist = 10;

  switch (driveState) {
    
      /////////////////////////////////////////
      case initial:
      {
        ////do nothing, wait for START signal
        Serial.println("initial");


        if (radio.available())
        {
          char buf[32];
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          Serial.print("Received: ");
          Serial.println(buf);
          delay(50);
          
          if (buf[0] == ident_char && buf[1] == '!' && buf[2] == '!' && buf[3] == '!') {
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
          }
          else{
            stopp();
          }
        }
        //else{
          //stopp();
        //}

        break;
      }
    
    /////////////////////////////////////////
    case followRadio:
      {
        Serial.println("followRadio");
        if (radio.available())
        {
          char buf[32];
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          //Serial.print("Received: ");
          //Serial.println(buf);

          //check if stop signal
          if (buf[0] == ident_char && buf[1] == '?' && buf[2] == '?' && buf[3] == '?') {
            Serial.println("STOP!!!!!!!!!!");
            driveState = gameEnd;
            break;
          }

          /////////decode buf info on puck position and drive towards it??????
          //buf will be array form [r1x,r1y, r2x,r2y, px,py];
          if (buf[0] == ident_char) {
            radioRX(buf);
          }
          if (buf[0] == ident_char && r1x != 0 && px != 0) {
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
            //Serial.print("delta x: ");
            //Serial.println(deltaX);
            //Serial.print("delta y: ");
            //Serial.println(deltaY);
            float theta_desired = atan2(deltaY, deltaX) * (180 / 3.1416);

            //turn robot such that it is pointing that angle and run it forward
            if (ident_char == 'b') {
              if (theta_desired < 0) {
                theta_desired = 360 + theta_desired;
                if (yaw < (theta_desired - 180)) {
                  yaw = yaw + 360;
                }
              }
              //else if (theta_desired > 0 && yaw > (theta_desired + 190)) {
                //yaw = yaw - 360;
              //}

              //Serial.println('b');
              if (abs(theta_desired - yaw) < 10) {
                forward();
                delay(250);
              }
              else if ((theta_desired - yaw) > 10) {
                //theta_desired = 360 - theta_desired;
                turnRight();
                delay(250);
              }
              else if ((theta_desired - yaw) < -10) {
                //theta_desired = 360 + theta_desired;
                turnLeft();
                delay(250);
              }
            }
            else {
              Serial.println('g');
              Serial.println(yaw);
              if (theta_desired < 0) {
                theta_desired = 360 + theta_desired;
                if (yaw < (theta_desired - 180)) {
                  yaw = yaw + 360;
                }
              }
              //else if (theta_desired > 0 && yaw > (theta_desired + 190)) {
                //yaw = yaw - 360;
              //}
              Serial.println(theta_desired);

              if (abs(theta_desired - yaw) < 10) {
                forward();
                delay(250);
              }
              else if ((theta_desired - yaw) > 10) {
                //theta_desired = 360 - theta_desired;
                turnLeft();
                delay(250);
              }
              else if ((theta_desired - yaw) < -10) {
                //theta_desired = 360 + theta_desired;
                turnRight();
                delay(250);
              }
            }
          }
        }

        int vol = analogRead(A0);
        pixyCam.ccc.getBlocks();
        if (vol < 100) {
          driveState = findGoal;
          //backward();
          //delay(500);
          //driveState = followRadio;
          //Serial.println("findGoal");
        }
        else if (pixyCam.ccc.numBlocks) {
          //Serial.println("got blocks!");
          if (pixyCam.ccc.blocks[0].m_signature == 1) {
            //driveState = followRadio;
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

        if (radio.available()) {
          char buf[32];
          uint8_t numBytes = radio.read(buf, sizeof(buf));

          //check if stop signal
          if (buf[0] == ident_char && buf[1] == '?' && buf[2] == '?' && buf[3] == '?') {
            driveState = gameEnd;
            break;
          }
        }

        pixyCam.ccc.getBlocks();
        int vol = analogRead(A0);
        if (vol < 100) {
          driveState = findGoal;
          //backward();
          //delay(1500);
          //driveState = followRadio;
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
        //Serial.println("findGoal");
        ////this should drive the robot towards goal using the radio, should have method for detecting if it can shoot on goal
        if (radio.available()) {
          char buf[32];
          uint8_t numBytes = radio.read(buf, sizeof(buf));
          radioRX(buf);

          //check if stop signal
          if (buf[0] == ident_char && buf[1] == '?' && buf[2] == '?' && buf[3] == '?') {
            driveState = gameEnd;
            break;
          }
          else if (buf[0] == ident_char) {
            radioRX(buf);

            /////////NEED FIND THE Y VAL READING THAT CORRESPONDS TO GOAL LIMITS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            float goal_topY, goal_botY;
            goal_topY = 700;
            goal_botY = 500;

            //Serial.println("y val control     ");
            //Serial.print(r1y);
            //Serial.print("            ");

            if (ident_char == 'b') {
              if (r1y < goal_botY && r1y != 0) {
                turnRight();
                delay(150);
              }
              if (r1y > goal_topY && r1y != 0) {
                turnLeft();
                delay(150);
              }
            }
            else {
               if (r1y < goal_botY && r1y != 0) {
                turnLeft();
                delay(150);
              }
              if (r1y > goal_topY && r1y != 0) {
                turnRight();
                delay(150);
              }
            }
            forward();
            delay(50);
          }
        }

        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

        yaw = orientationData.orientation.x;
        pitch = orientationData.orientation.y;
        roll = orientationData.orientation.z;
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);

        //Serial.println("angle controller        ");
        //Serial.print(yaw);
        //Serial.print("          ");

        if (yaw < 10 || yaw > 350) {
          forward();
          delay(50);
        }
        else if (yaw > 10 && yaw < 90) {
          turnLeft();
          delay(150);
        }
        else if (yaw < 350 && yaw > 270) {
          turnRight();
          delay(150);
        }
        else if (yaw < 270 && yaw > 180) {
          turnRight();
          delay(3.00);
        }
        else if (yaw < 180 && yaw > 90) {
          turnLeft();
          delay(300);
        }

        int vol = analogRead(A0);
        pixyCam.ccc.getBlocks();
        if (r1x < goal_pos && vol < 100) { //DETECT CLOSE TO GOALLLLLLLLLLLL ---> 180for'g',2280for'b'    ////////dont forget chnage sign for team, 'b'is>, 'g'is<
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
        digitalWrite(shooterMOT, HIGH);
        delay(100);
        //digitalWrite(shooterMOT, LOW);

        forward();
        //analogWrite(shooterMOT, 255);
        //delay(50);
        //analogWrite(shooterMOT, 0);

        if (radio.available()) {
          char buf[32];
          uint8_t numBytes = radio.read(buf, sizeof(buf));

          //check if stop signal
          if (buf[0] == ident_char && buf[1] == '?' && buf[2] == '?' && buf[3] == '?') {
            driveState = gameEnd;
            break;
          }
        }

        int vol = analogRead(A0);
        pixyCam.ccc.getBlocks();
        if (vol < 100) {
          driveState = shoot;
        }
        else if (pixyCam.ccc.numBlocks) {
          //Serial.println("got blocks!");
          if (pixyCam.ccc.blocks[0].m_signature == 1) {
            digitalWrite(shooterMOT, LOW);
            driveState = followPixy;
          }
        }
        else {
          digitalWrite(shooterMOT, LOW);
          driveState = followRadio;
        }
        break;
      }

    case gameEnd:
      {
        Serial.println("stop");
        stopp();
        break;
      }
  }


  /////////////////////////////////////////
  /////////////////////////////////////////
  /////////////////////////////////////////

  //Serial.print(leftDist);
  if (leftDist < minDist) {
    //Serial.println("detect LEFT");
    avoidLeft();
  }

  //Serial.println(leftDist);
  leftDist = measureDistance(ultraL); //change to CM!!!!!!!
  if (leftDist == 0) {
    leftDist = 100;
  }

  //Serial.print("      ");
  //Serial.println(rightDist);
  if (rightDist < minDist) {
    //Serial.println("detect Right");
    avoidRight();
  }

  //Serial.println(rightDist);
  rightDist = measureDistance(ultraR); //change to CM!!!!!!!
  if (rightDist == 0) {
    rightDist = 100;
  }
}

/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////



void forward() {
  //digitalWrite(LED_PIN, HIGH);
  Serial.println("forward");
  motors.setM1Speed(highSpeed);
  motors.setM2Speed(highSpeed);
}

void backward() {
  //digitalWrite(LED_PIN, LOW);
  Serial.println("backward");
  motors.setM1Speed(highSpeed * (-1));
  motors.setM2Speed(highSpeed * (-1));
}

void turnLeft() {
  Serial.println("left");
  //digitalWrite(LED_PIN, HIGH);
  motors.setM1Speed(lowSpeed);
  motors.setM2Speed(highSpeed);
}

void turnRight() {
  Serial.println("right");
  //digitalWrite(LED_PIN, HIGH);
  motors.setM1Speed(highSpeed);
  motors.setM2Speed(lowSpeed);
}

void pttrnLeft() {
  Serial.println("left in place");
  motors.setM1Speed((-1)*highSpeed );
  motors.setM2Speed(highSpeed);
}

void pttrnRight() {
  Serial.println("right in place");
  motors.setM1Speed(highSpeed);
  motors.setM2Speed(highSpeed * (-1));
}

void stopp() {
  Serial.println("stop");
  //digitalWrite(LED_PIN, LOW);
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

void radioRX(char msg[32]) {
  int i = 1;
  String r1x_msg = String(msg[i]);
  for (i = 2; i <= 4; i++) {
    r1x_msg.concat(String(msg[i]));
  }
  r1x = r1x_msg.toFloat();

  String r1y_msg = String(msg[i + 1]);
  for (i = 7; i <= 9; i++) {
    r1y_msg.concat(String(msg[i]));
  }
  r1y = r1y_msg.toFloat();

  String r2x_msg = String(msg[i + 1]);
  for (i = 12; i <= 14; i++) {
    r2x_msg.concat(String(msg[i]));
  }
  r2x = r2x_msg.toFloat();

  String r2y_msg = String(msg[i + 1]);
  for (i = 17; i <= 19; i++) {
    r2y_msg.concat(String(msg[i]));
  }
  r2y = r2y_msg.toFloat();

  String px_msg = String(msg[i + 1]);
  for (i = 22; i <= 24; i++) {
    px_msg.concat(String(msg[i]));
  }
  px = px_msg.toFloat();

  String py_msg = String(msg[i + 1]);
  for (i = 27; i <= 29; i++) {
    py_msg.concat(String(msg[i]));
  }
  py = py_msg.toFloat();
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
  //float distL = measureDistance(ultraL);

  //if ((millis() - timeLast > 400) && (abs(distL - leftDist) > 1)) {
  //Serial.println("avoiding Left");
  backward();
  delay(300);

  pttrnRight();
  timeLast = millis();
  //leftDist = distL;
  delay(300);
  //}
}

void avoidRight() {
  //Serial.println("avoid right");
  //float distR = measureDistance(ultraR);

  //if ((millis() - timeLast > 400) && (abs(distR - rightDist) > 1)) {
  //Serial.println("avoiding Right");
  backward();
  delay(300);

  pttrnLeft();
  timeLast = millis();
  //rightDist = distR;
  delay(300);
  // }
}
