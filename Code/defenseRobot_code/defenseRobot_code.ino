

uint8_t robotStates;      // Create global state variable
int matchStatus;          // matchStatus = 1 -- Start
//                                         2 -- Stop
//                                         3 -- Read Coordinates
char *text;              // Message read from the RF sensor

NRF24 radio;
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

}
