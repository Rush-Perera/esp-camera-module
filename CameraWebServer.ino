#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*
//Motor Driver properties

const unsigned int EN_A = 10;
const unsigned int IN1_A = 9;
const unsigned int IN2_A = 8;

const unsigned int IN1_B = 7;
const unsigned int IN2_B = 6;
const unsigned int EN_B = 5;
// Initialize both motors
//L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
*/

//M1 left moter
int enA = 10;
int in1 = 8;
int in2 = 9;

//M2

int enB = 5;
int in3 = 7;
int in4 = 6;

//PID Properties
const double kP = 0.08;
const double kD = 0.125;
double lastError = 0;
const int GOAL = 3500;
const unsigned char MAX_SPEED = 100;


void setup() {

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(23);

  calibrateLineSensor();
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}




void loop() {
    // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 7000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
    for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);

  //Compute error from line
  int error = GOAL - position;

  //Compute motor adjustment
  int adjustment = kP*error + kD*(error - lastError);

  //store the error for next increment
  lastError = error;

  //Adjust Motors
  mpower(1,1,constrain(MAX_SPEED - adjustment , 0, MAX_SPEED));
  mpower(2,1,constrain(MAX_SPEED + adjustment , 0, MAX_SPEED));


}


void calibrateLineSensor(){
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}

/*
mpower(motor number, rotation, speed);

motor number
1- 1st motor
2- 2nd motor

rotation
1- forward
0- stop
-1 - backward

speed 

0 - 255

*/
void mpower(int motor, int rotation, int speed) {
  int pwm;
  int pA;
  int pB;
  if (motor == 1){
    pwm = enA;
    pA = in1;
    pB = in2;
  } else if ( motor == 2){
    pwm = enB;
    pA = in3;
    pB = in4;
  } else{
    return;
  }

  if (rotation == 0) {
    digitalWrite(pA, LOW);
    digitalWrite(pB,LOW);
  } else if ( rotation == 1){
    digitalWrite(pA, HIGH);
    digitalWrite(pB, LOW);
  } else if (rotation == -1){
    digitalWrite(pA, LOW);
    digitalWrite(pB, HIGH);
  }

  analogWrite(pwm, speed);

}
