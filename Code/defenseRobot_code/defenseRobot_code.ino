

uint8_t robotStates;      // Create global state variable
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates
char *text;              // Message read from the RF sensor


NRF24 radio;

Bot robotDefense;
Bot Goal;


uint8_t M1DIR = 2;
uint8_t M1PWM = 3;
uint8_t M2DIR = 4;
uint8_t M2PWM = 5;
DRV8835MotorShield motors = DRV8835MotorShield(M1DIR, M1PWM, M2DIR, M2PWM);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initial_RF(&radio, 1);
  robotStates = Initialize;
  Goal.x = 0;
  Goal.y = 0;
  Goal.theta = NULL;
}
}

void loop() {
  // put your main code here, to run repeatedly:
  // switch case

  switch (robotStates) {
    case Initialize:
      /* In this stae, robot will waiting for the signal .
          if the signal is received, change state to stay in front of door

          Need a boolean type function to check sound sensor or RF receiver.
          Return true if signal has been received
      */
      // function
      text = RF_receiver(&radio);                   // Read RF
      matchStatus = start_stop_message(text);       // Check matchStatus
      if (matchStatus == 1)
        robotStates = puckSearching;
      else
        robotStates = Initialize;


      break;

    case Inplace:
    /* In this state, the robot should move to right front to the goal 
     *  1. adjust the angle 
     *  2. move the place 
     *  3. Don't enter to the goal 
     */
      text = RF_receiver(&radio);                   // Read RF
      Getcoordinates(text, RF_coordinates);         // Get coordinates
      robotDefense.x = RF_coordinates[2];
      robotDefense.y = RF_coordinates[3];
   



      }
