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

#define lum 500

//Configuraçoes dos motores
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int speedRight = 180;
int speedLeft = speedRight*0.80;

//Sensores var config
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

void setup() {
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){22, 4, 17, 18, 23}, SensorCount);
  qtr.setEmitterPin(16);

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
    motor1.drive(speedRight - 30,1000);
    motor2.drive(0,1000);
}

void right(){
    motor1.drive(0,1000);
    motor2.drive(speedLeft - 30,1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t position = qtr.readLineWhite(sensorValues);

   for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  
  if(sensorValues[4] < lum){
    forward();
  }
  if(sensorValues[6] < lum || sensorValues[5] < lum && sensorValues[4] > lum){
    left();
  }
  if(sensorValues[2] < lum || sensorValues[3] < lum && sensorValues[4] > lum){
    right();
  }

}
