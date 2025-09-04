

//Sensor ultrassônico:
const int pin_echo = 15;
const int pin_trig = 2;

unsigned long begin = 0;

unsigned long acaoAte = 0;
int etapa = 0;

enum status {WAIT, TRIG_HIGH, TRIG_LOW, ECHO};
status current_status = WAIT;


// Sensores infravermelhos:
// Se tem presenca de luz = 1 ; ausencia = 0
const int sensor_direita = 23;
const int sensor_esquerda = 13;

// Valores PwM utilizada no motor:
const int velocidade = 255;
const int parado = 0;

// Motores que vão para frente:
const int motorRightFront_Go = 22;
const int motorLeftFront_Go = 4;
const int motorRightBack_Go = 18;
const int motorLeftBack_Go = 5;

// Motores que vão para trás:
const int motorRightFront_Back = 21;
const int motorLeftFront_Back = 16;
const int motorRightBack_Back = 19;
const int motorLeftBack_Back = 17;

void setup() {

// Criação dos canais em seus respectivos pinos:
    for (int i = 0; i < 8; i++)
    {
      ledcSetup(i, 5000, 8);
    }

    ledcAttachPin(motorRightFront_Go, 0);
    ledcAttachPin(motorLeftFront_Go, 1);
    ledcAttachPin(motorRightBack_Go, 2);
    ledcAttachPin(motorLeftBack_Go, 3);
    ledcAttachPin(motorRightFront_Back, 4);
    ledcAttachPin(motorLeftFront_Back, 5);
    ledcAttachPin(motorRightBack_Back, 6);
    ledcAttachPin(motorLeftBack_Back, 7);
    

// Sensores:
  Serial.begin(9600);
  pinMode(sensor_direita, INPUT);
  pinMode(sensor_esquerda, INPUT);
  pinMode(pin_echo, INPUT);
  pinMode(pin_trig, OUTPUT);

} 

int distancia_inimigo()
{
  digitalWrite(pin_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trig, LOW);

  long duracao = pulseIn(pin_echo, HIGH);
  long distancia = duracao * 0.034 / 2;

  delay(500);
  return distancia;
 
}
 
void ir_frente() {
  ledcWrite(0, velocidade);
  ledcWrite(1, velocidade);
  ledcWrite(2, velocidade);
  ledcWrite(3, velocidade);
  ledcWrite(4, parado);
  ledcWrite(5, parado);
  ledcWrite(6, parado);
  ledcWrite(7, parado);
}

void ir_tras() {
  ledcWrite(0, parado);
  ledcWrite(1, parado);
  ledcWrite(2, parado);
  ledcWrite(3, parado);
  ledcWrite(4, velocidade);
  ledcWrite(5, velocidade);
  ledcWrite(6, velocidade);
  ledcWrite(7, velocidade);
}

void girar_direita() {
  ledcWrite(0, parado);
  ledcWrite(1, velocidade);
  ledcWrite(2, parado);
  ledcWrite(3, velocidade);
  ledcWrite(4, velocidade);
  ledcWrite(5, parado);
  ledcWrite(6, velocidade);
  ledcWrite(7, parado);
}

void girar_esquerda() {
  ledcWrite(0, velocidade);
  ledcWrite(1, parado);
  ledcWrite(2, velocidade);
  ledcWrite(3, parado);
  ledcWrite(4, parado);
  ledcWrite(5, velocidade);
  ledcWrite(6, parado);
  ledcWrite(7, velocidade);
}



void loop()
{

    int direita = digitalRead(sensor_direita);
    int esquerda = digitalRead(sensor_esquerda);

    // long tempo = 0;

    // girar_direita();

    long distancia = distancia_inimigo();
    unsigned long agora = millis();

    if(etapa == 0)
    {
        if(esquerda == 1)
        {
            etapa = 1;
            acaoAte = agora + 300;  //gira para a esquerda por 300ms
        }
        else if(direita == 1)
        {
            etapa = 2;
            acaoAte = agora + 300;  //gira para a direita por 300ms
        }
    }

    if (etapa == 2)
    {
        girar_esquerda();
        if (agora > acaoAte) {
            etapa = 3; 
            acaoAte = agora + 300; // ré
            }
            return;
    }
        
    if (etapa == 3) 
    { // ré
        ir_tras();
        if (agora > acaoAte) {
        etapa = 4;
        acaoAte = agora + 400; // girar depois da ré
        }
        return;
    }
    
    if (etapa == 4) 
    { // gira para procurar de novo
        girar_direita();  // poderia alternar para esquerda também
        if (agora > acaoAte) {
        etapa = 0; // terminou sequência, volta ao normal
        }
        return;
    }
    
    // --- 3. Se não está na borda ---
    if (distancia <= 40) 
    {
        ir_frente();   // atacar
    } else {
        girar_direita(); // procurar
    }
    
}
    