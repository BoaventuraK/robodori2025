#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif

//Assign the motor pins
#define AIN1 26
#define AIN2 25
#define BIN1 27
#define BIN2 32
#define PWMA 14
#define PWMB 12
#define STBY 33

const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

QTRSensors qtr;
BluetoothSerial SerialBT;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount]; //limite???????

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 300000;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 120;
int rtspeed = 120*0.90;

//float Kp = 0.061;
float Kp = lfspeed/3500;
float Ki = 0;
//float Kd = 0.0005;
float Kd = Kp*pow(10,(-2));

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;

int val, cnt = 0, v[3];

void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){21, 22, 4, 17, 18, 15, 19, 23}, SensorCount);  //colocar as portas dos sensores
  qtr.setEmitterPin(16);
  startMillis = millis();

  delay(500);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  
  Serial.begin(9600);
  SerialBT.begin();
  Serial.println("Bluetooth Started! Ready to pair...");
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    //rotina_calibrar();
    //Serial.print(i);
    //Serial.println(" ");
  }

   for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }

  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

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
  // print the calibration minimum values measured when emitters were on
  
  /*for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2; //faz uma media???
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();

  delay(1000);*/
}

void loop()
{
uint16_t position = qtr.readLineWhite(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position

  currentMillis = millis();
  if(currentMillis<= period){
    robot_control();
    //startMillis = currentMillis;
    //Serial.print("Current Millis");
    //Serial.println(currentMillis);
  } else {
    Serial.print("Entrou no else");
    brake(motor1, motor2);
  }
  
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  //delay(250);
}

void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 7000 (for a white line, use readLineWhite() instead)
  position = qtr.readLineWhite(sensorValues);
  error = 3500 - position;
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980 && sensorValues[5]>=980 && sensorValues[6]>=980 && sensorValues[7]>=980){ // A case when the line follower leaves the line
    if(previousError>0){       //Turn left if the line was to the left before
      motor_drive(-230,230);
    }
    else{
      motor_drive(230,-230); // Else turn right
    }
    position = qtr.readLineWhite(sensorValues);
  }
  PID_Linefollow(error);
  valuesread();
}

void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = rtspeed + PIDvalue;

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }
    motor_drive(lsp,rsp);
}
// This void delimits each instruction.
// The  Arduino knows that for each instruction it will receive 2 bytes.
void valuesread()  {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  for (int i = 0; i > 2; i++){
    Serial.print(v[1]);
    Serial.print(v[2]);
  }
  if (cnt == 2)
  cnt = 0;
  processing();
}

//In this void the the 2 read values are assigned.
void processing() {
 int a = v[1];
 if (a == 1) {
   Kp = v[2];
 }
 if (a == 2) {
   multiP = v[2];
 }
 if (a == 3) {
   Ki = v[2];
 }
 if (a == 4) {
   multiI = v[2];
 }
 if (a == 5) {
   Kd  = v[2];
 }
 if (a == 6) {
   multiD = v[2];
 }
 if (a == 7)  {
   digitalWrite(2, HIGH);
   delay(500);
   digitalWrite(2, LOW);
 }
}
void motor_drive(int left, int right){
  
  if(right>0)
  {
    motor1.drive(left);
    // motor2.setSpeed(right);
    // motor2.forward();
  }
  else 
  {
    motor1.drive(left);
    // motor2.setSpeed(right);
    // motor2.backward();
  }
  
 
  if(left>0)
  {
    motor2.drive(right);
    // motor1.setSpeed(left);
    // motor1.forward();
  }
  else 
  {
    motor2.drive(right);
    // motor1.setSpeed(left);
    // motor1.backward()';
  }
}

void rotina_calibrar(){
    left(motor1, motor2, 130 * 0.8);
    delay(500);
    right(motor1, motor2, 130);
    delay(500);
 
    /*
    int calibrarspd = 90*0.8;

    motor1.drive(90, 500);
    motor2.drive(calibrarspd*-1, 500);
    brake(motor1, motor2);
    
    motor1.drive(-90, 500);
    motor2.drive(calibrarspd, 500);
    brake(motor1, motor2);
    */
}
