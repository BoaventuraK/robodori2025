/* Referência
    Programa: Exemplo de uso do driver Ponte H L298N
    Autor: Gustavo Nery, Eletrogate.
    Arduino 1.8.12
    Adaptado por Josemar. Inicio das adaptações em 20/01/2023
    Ponte H
    Classe
    Bluetooth
    zé - adpatação para zé 12/05/2024
*/

//Para BluetoothSerial
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
char command;

boolean state = 1;

// Display
#include <Wire.h> // Biblioteca utilizada para fazer a comunicação com o I2C
//#include <LiquidCrystal_I2C.h> // Biblioteca utilizada para fazer a comunicação com o display 20x4
//#define col 16 //20 //16 // Serve para definir o numero de colunas do display utilizado
//#define lin  2 //4 // 2 // Serve para definir o numero de linhas do display utilizado
//#define ende  0x27 // Serve para definir o endereço do display.

//LiquidCrystal_I2C lcd(ende, col, lin); // Chamada da


//Para Menu
#define LED 13
//int command; //Int to store app command state.
int Speed = 204; // 0 - 255.
int Speedsec;
int buttonState = 0;
int lastButtonState = 0;
int Turnradius = 0; //Set the radius of a turn, 0 - 255 Note:the robot will malfunction if this is higher than int Speed.
int brakeTime = 45;
int brkonoff = 1; //1 for the electronic braking system, 0 for normal.


// Iremos fazer uma classe para facilitar o uso da ponte H L298N na manipulação dos motores na função Setup e Loop.

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

class DCMotor {
    //int spd = 255, pin1, pin2, enable1Pin;
    int spd = 255, pin1, pin2;

  public:

    //void Pinout(int in1, int in2, int en) { // Pinout é o método para a declaração dos pinos que vão controlar o objeto motor
    void Pinout(int in1, int in2) { // Pinout é o método para a declaração dos pinos que vão controlar o objeto motor
      pin1 = in1;
      pin2 = in2;
      //enable1Pin = en;
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
      //pinMode(enable1Pin, OUTPUT);

      ledcSetup(pwmChannel, freq, resolution); // configure LED PWM functionalitites
      //ledcAttachPin(enable1Pin, pwmChannel); // attach the channel to the GPIO to be controlled
    }

    void Speed(int in1) { // Speed é o método que irá ser responsável por salvar a velocidade de atuação do motor
      spd = in1;
    }


    void Forward() { // Forward é o método para fazer o motor girar para frente
      //Serial.print("Moving Forward  ");
      //Serial.print("spd: ");
      //Serial.println(spd);
      analogWrite(pin1, spd);
      digitalWrite(pin2, LOW);
      ledcWrite(pwmChannel, spd);
    }

    void Backward() { // Backward é o método para fazer o motor girar para trás
      //Serial.print("Moving Backwards  ");
      //Serial.print("spd: ");
      //Serial.println(spd);
      digitalWrite(pin1, LOW);
      analogWrite(pin2, spd);
      ledcWrite(pwmChannel, spd);
    }

    /*
        void Left() { // Left é o método para fazer o robô ir para a esquerda
          Serial.print("Moving Left  ");
          Serial.print("spd: ");
          Serial.println(spd);
          digitalWrite(pin1, LOW);
          analogWrite(pin2, spd);
          ledcWrite(pwmChannel, spd);
        }
    */


    void Stop() { // Stop é o metodo para fazer o motor ficar parado.
      //Serial.println("Motor stopped");
      analogWrite(pin1, LOW);
      analogWrite(pin2, LOW);
    }
}; //tem que ter o ponto e vírgula

DCMotor LeftMotorFront, LeftMotorRear, RightMotorFront, RightMotorRear; // Criação de quatro objetos motores, já que usaremos quatro motores, e eles já estão prontos para receber os comandos já configurados acima.

void setup() {
  Serial.begin(115200);
  //  btSerial.begin(9600);
  SerialBT.begin("ze");

  // Display
  //lcd.init(); // Serve para iniciar a comunicação com o display já conectado
  // lcd.backlight(); // Serve para ligar a luz do display
  // lcd.clear(); // Serve para limpar a tela do display
  //Exemplo
  // lcd.setCursor(0, 0); // Coloca o cursor do display na coluna 1 e linha 1
  // lcd.print("Sou Fox "); // Comando de saída com a mensagem que deve aparecer na coluna 2 e linha 1.
  // lcd.setCursor(0, 1); //Coloca o cursor do display na coluna 1 e linha 2
  // lcd.print("Estou no comando");  // Comando de saida com a mensagem que deve aparecer na coluna 2 e linha 2
  //delay(3000);  // delay de 5 segundos com todas as mensagens na tela
  //Teste
  //Motores
  //  LeftMotorFront.Pinout(26, 25, 33); // Seleção dos pinos que cada motor usará, como descrito na classe.
  //  LeftMotorRear.Pinout(17, 16, 4); // Seleção dos pinos que cada motor usará, como descrito na classe.
  //  RightMotorFront.Pinout(19, 18, 23); // Seleção dos pinos que cada motor usará, como descrito na classe.
  //  RightMotorRear.Pinout(12, 27, 14);  // Seleção dos pinos que cada motor usará, como descrito na classe.

  LeftMotorFront.Pinout(4, 16); // Seleção dos pinos que cada motor usará, como descrito na classe.
  LeftMotorRear.Pinout(5, 19); // Seleção dos pinos que cada motor usará, como descrito na classe.
  RightMotorFront.Pinout(22, 21); // Seleção dos pinos que cada motor usará, como descrito na classe.
  RightMotorRear.Pinout(18, 17);  // Seleção dos pinos que cada motor usará, como descrito na classe.

  Serial.println("Fim Setup");
  Serial.println("Conecte App"); // pronto para conectar no App Bluetooth
  //App no Android para controlar via Bluetooth
  // Bluetothh RC Controller ou Car Bluetooth RC
  //delay(2000);
  Stop(); //Initialize with motors stoped.

}

void loop() {
  LeftMotorFront.Speed(Speed);   // A velocidade do motor pode variar de 0 a 255, onde 255 é a velocidade máxima.
  LeftMotorRear.Speed(Speed);    // A velocidade do motor pode variar de 0 a 255, onde 255 é a velocidade máxima.
  RightMotorFront.Speed(Speed);  // A velocidade do motor pode variar de 0 a 255, onde 255 é a velocidade máxima.
  RightMotorRear.Speed(Speed);   // A velocidade do motor pode variar de 0 a 255, onde 255 é a velocidade máxima.

  //display(); // chama função display

  if (SerialBT.available()) {
    command = SerialBT.read();
    Serial.print("Comando pressionado: ");
    Serial.println(command);
    //delay(10); //JS
    if (command == 'X') {
      state = 1;
    } else if (command == 'x') {
      state = 0;
    }

  }

  switch (command) {
    case 'V': // teste JS
      buzina();
      break;
    case 'F':
      forward();
      break;
    case 'S':
      Stop(); // Stop motors.
      break;
    case 'B':
      back();
      break;
    case 'L':
      //left();
      if (command == 'L' && state == 1) {
        superleft ();
      } else if (command == 'L' && state == 0) {
        left ();
      }
      break;

    case 'R':
      //right();
      if (command == 'R' && state == 1) {
        superright  ();
      } else if (command == 'R' && state == 0) {
        right ();
      }
      break;
    case 'G':
      forwardleft();
      break;
    case 'I':
      forwardright();
      break;
    case 'H':
      backleft();
      break;
    case 'J':
      backright();
      break;
    case '0':
      Speed = 100;
      break;
    case '1':
      Speed = 140;
      break;
    case '2':
      Speed = 153;
      break;
    case '3':
      Speed = 165;
      break;
    case '4':
      Speed = 178;
      break;
    case '5':
      Speed = 191;
      break;
    case '6':
      Speed = 204;
      break;
    case '7':
      Speed = 216;
      break;
    case '8':
      Speed = 229;
      break;
    case '9':
      Speed = 242;
      break;
    case 'q':
      Speed = 255;
      break;
  }
  Speedsec = Turnradius;
  if (brkonoff == 1) {
    brakeOn();
  } else {
    brakeOff();
  }
}


void buzina() {
  Serial.print("Buzina On  ");
  delay(1000);
}


void Stop() {
  LeftMotorFront.Stop();  // Comando para o motor parar
  LeftMotorRear.Stop();   // Comando para o motor parar
  RightMotorFront.Stop(); // Comando para o motor parar
  RightMotorRear.Stop();  // Comando para o motor parar
}

void forward() {
  Serial.print("Moving forward  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Forward();   // Comando para o motor ir para frente
  LeftMotorRear.Forward();    // Comando para o motor ir para frente
  RightMotorFront.Forward();  // Comando para o motor ir para frente
  RightMotorRear.Forward();   // Comando para o motor ir para frente
}

void back() {
  Serial.print("Moving back  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Backward();  // Comando para o motor ir para trás
  LeftMotorRear.Backward();   // Comando para o motor ir para trás
  RightMotorFront.Backward(); // Comando para o motor ir para trás
  RightMotorRear.Backward();  // Comando para o motor ir para trás
}

void left() {
  Serial.print("Moving left  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Backward();  // Comando para o motor ir para trás
  LeftMotorRear.Backward();    // Comando para o motor ir para frente
  RightMotorFront.Forward();  // Comando para o motor ir para frente
  RightMotorRear.Forward();  // Comando para o motor ir para trás
}

void right() {
  Serial.print("Moving right  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Forward();   // Comando para o motor ir para frente
  LeftMotorRear.Forward();   // Comando para o motor ir para trás
  RightMotorFront.Backward(); // Comando para o motor ir para trás
  RightMotorRear.Backward();   // Comando para o motor ir para frente
}

//stationary rotation (spin)-testando
void superleft() {
  Serial.print("stationary rotation (spin) - superleft  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Backward();  // Comando para o motor ir para trás
  LeftMotorRear.Forward();   // Comando para o motor parar
  RightMotorFront.Forward(); // Comando para o motor parar
  RightMotorRear.Backward();  // Comando para o motor ir para trás
}
//stationary rotation (spin)-testando
void superright() {
  Serial.print("stationary rotation (spin) - superright  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Forward();  // Comando para o motor parar
  LeftMotorRear.Backward();   // Comando para o motor parar
  RightMotorFront.Backward(); // Comando para o motor parar
  RightMotorRear.Forward();  // Comando para o motor parar
}

void forwardleft() {
  Serial.print("Moving forwardleft  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Stop();  // Comando para o motor parar
  LeftMotorRear.Stop();    // Comando para o motor ir para frente
  RightMotorFront.Forward();  // Comando para o motor ir para frente
  RightMotorRear.Forward();  // Comando para o motor parar
}
void forwardright() {
  Serial.print("Moving forwardright  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Forward();   // Comando para o motor ir para frente
  LeftMotorRear.Forward();   // Comando para o motor parar
  RightMotorFront.Stop();  // Comando para o motor ir para frente
  RightMotorRear.Stop();   // Comando para o motor ir para frente
}

void backright() {
  Serial.print("Moving backright  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Stop();  // Comando para o motor parar
  LeftMotorRear.Stop();   // Comando para o motor ir para trás
  RightMotorFront.Backward(); // Comando para o motor ir para trás
  RightMotorRear.Backward();  // Comando para o motor parar
}

void backleft() {
  Serial.print("Moving backleft  ");
  Serial.print("spd: ");
  Serial.println(Speed);
  LeftMotorFront.Backward();  // Comando para o motor ir para trás
  LeftMotorRear.Backward();   // Comando para o motor parar
  RightMotorFront.Stop(); // Comando para o motor parar
  RightMotorRear.Stop();  // Comando para o motor ir para trás
}

void brakeOn() {
  //Here's the future use: an electronic braking system!
  // read the pushbutton input pin:
  buttonState = command;
  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == 'S') {
      if (lastButtonState != buttonState) {
        // digitalWrite(in1, HIGH);
        // digitalWrite(in2, HIGH);
        //  digitalWrite(in3, HIGH);
        // digitalWrite(in4, HIGH);
        delay(brakeTime);
        Stop();
      }
    }
    // save the current state as the last state,
    //for next time through the loop
    lastButtonState = buttonState;
  }
}
void brakeOff() {


}

//Display
void display () {
  //lcd.setCursor(0, 0); // Coloca o cursor do display na coluna 1 e linha 1
  // lcd.print("Bill Estranho "); // Comando de saída com a mensagem que deve aparecer na coluna 2 e linha 1.
  //lcd.setCursor(0, 1); //Coloca o cursor do display na coluna 1 e linha 2
  // lcd.print("2023");  // Comando de saida com a mensagem que deve aparecer na coluna 2 e linha 2
  //delay(3000);  // delay de 5 segundos com todas as mensagens na tela

  //lcd.clear(); // Serve para limpar a tela do display
  // lcd.setCursor(0, 0); // Coloca o cursor do display na coluna 1 e linha 1
  //  lcd.print("Sou Fox "); // Comando de saída com a mensagem que deve aparecer na coluna 2 e linha 1.
  //  lcd.setCursor(0, 1); //Coloca o cursor do display na coluna 1 e linha 2
  // lcd.print("Estou no comando");  // Comando de saida com a mensagem que deve aparecer na coluna 2 e linha 2
  //delay(3000);  // delay de 5 segundos com todas as mensagens na tela

  //lcd.clear(); // Serve para limpar a tela do display
  // lcd.setCursor(0, 0); //Coloca o cursor do display na coluna 1 e linha 1
  // lcd.print("UNEB");  // Comando de saida com a mensagem que deve aparecer na coluna 2 e linha 3
  // lcd.setCursor(0 , 1); //Coloca o cursor do display na coluna 1 e linha 2
  //  lcd.print("ACSO BahiaRT");  // Comando de saida com a mensagem que deve aparecer na coluna 2 e linha 4
  //delay(3000);  // delay de 5 segundos com todas as mensagens na tela
  // lcd.clear(); // Limpa o display até o loop ser reiniciado
}