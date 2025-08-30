#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

#define AIN1 26
#define AIN2 25
#define BIN1 27
#define BIN2 32
#define PWMA 14
#define PWMB 12
#define STBY 33

#define LED_BUILTIN 2

//Configuraçoes dos motores
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int speedRight = 200;
int speedLeft = 200*0.80;

//Sensores var config
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){22, 2, 4, 17, 18, 19, 15, 23}, SensorCount);
  qtr.setEmitterPin(16);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 1000; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

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

void forward(){
    motor1.drive(speedRight,1000);
    motor2.drive(speedLeft,1000);
}

void left(){
    motor1.drive(speedRight,1000);
    motor2.drive(0,1000);
}

void right(){
    motor1.drive(0,1000);
    motor2.drive(speedLeft,1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t position = qtr.readLineWhite(sensorValues);
  
  if(sensorValues[3] > 500 && sensorValues[4] > 500 && sensorValues[5] > 500){
    forward();
  }else if(sensorValues[3] < 500 && (sensorValues[4] > 500 || sensorValues[5] > 500)){
    left();
  }else if(sensorValues[4] < 500 && (sensorValues[3] > 500 || sensorValues[2] > 500)){
    right();
  }

}
