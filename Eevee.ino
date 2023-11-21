// Pinos ponte h
#define IN1   11
#define IN2   3
#define IN3   10
#define IN4   9
#define IR  7

// Variaveis controle de velocidade para módulos ponte-h drv8833 e l9110s
int PWMA;
int PWMB;

// Arrays que guardam os valores de leitura dos sensores para casos específicos 
int leituraPista[8]; // para calibração
int leituraLinha[8]; // para calibração
int sensores[8]; // para leitura analógica
int digital[8];  // para conversão de analógico a digital

const int pinosSensores[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Pista = 0, pista branca com linha preta
// Pista = 1, pista preta com linha branca
int pista = 1;

// Array que, após a calibração, guarda os valores individuais de funcionamento de cada sensor
int linha[8];

// Variáveis para calcular a posição do robô em relação à linha
long int somaPonderada, soma, posicao, posicaoLinha, ultimaPosicao;

// float KP = 0.1;
// float KD = 0.11333;

// Variáveis PID
float KP = 0.2;
float KD = 2;
float KI = 0;
int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int ultimoProporcional = 0;
int setpoint = 350;


// Dados Integral //
int erro1 = 0;
int erro2 = 0;
int erro3 = 0;
int erro4 = 0;
int erro5 = 0;
int erro6 = 0;



int velocidadeRobo = 80;
int velocidadeFrente = 240;
int velocidadeAtras = 120;
void setup() {  
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IR, OUTPUT);

  digitalWrite(IR, HIGH);


  for (int i = 0; i < 8; i++) {
    pinMode(pinosSensores[i], INPUT);
    }
  
 calibracao();
    
}
void loop() {
  freios();
  leituraSensores();
  PID();

  //motores(200, 200);
  //delay(15);
  //motores(0,0);
  //posicaoLinha = leituraSensores();
  //Serial.println(posicaoLinha);
 //leituraSensores();
}


// leituraSensores() lê todos os valores analógicos dos sensores,
// e os amarzena na array digital[8] já convertidos em 0 ou 1;
// Também faz a soma *ponderada* e soma normal dos valores dos sensores,
// a fim de calcular a posição do robô em relação à linha.
// Para 8 sensores, por exemplo, definiu-se a posição ideal (robô seguindo o meio da linha) como 350
// (100 de peso para cada sensor, começando do sensor 0 e indo até o 7)
int leituraSensores(void) {
  for (int i = 0; i < 8; i++) {
    sensores[i] = analogRead(pinosSensores[i]);
    if(pista==1) { if(sensores[i] >= linha[i]) { digital[i] = 0; } else { digital[i] = 1;}}
    if(pista==0) { if(sensores[i] >= linha[i]) { digital[i] = 1;} else {digital[i] = 0;}}
    //Serial.print(digital[i]);
    //Serial.print(" ");
    }
    //delay(15);
  //Serial.println(" ");

  somaPonderada = (700*digital[0]) + (600*digital[1]) + (500*digital[2]) + (400*digital[3]) + (300*digital[4]) + (200*digital[5]) + (100*digital[6]) + (0*digital[7]);
  soma = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]);
  posicao = (somaPonderada / soma);


// Conserva a última posição lida pelo robô caso os sensores saim completamente da linha
  if(ultimaPosicao<=100 && posicao==-1) {
    posicao = 0;
  }
   if(ultimaPosicao>=600 && posicao==-1) {
    posicao = 700;
  }

  ultimaPosicao = posicao;
  return posicao;
}


void motores(int esquerdo, int direito) {
  if(direito>=0) {
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
    PWMA = IN1;
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    direito = direito * (-1);
    PWMA = IN2;
  }
  analogWrite(PWMA, direito);


  if(esquerdo>=0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    PWMB = IN3;
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    esquerdo = esquerdo * (-1);
    PWMB = IN4;
  }
  analogWrite(PWMB, esquerdo);
}

// Lê, primeiramente, os valores apenas da pista
// Depois, lê os valores da linha
// Faz uma média entre esses valores, resultando na calibração
// O resultado da calibração é usado no momento de converter os dados analógicos para digitais.
void calibracao(){
  digitalWrite(13, HIGH);
  for (int i = 0; i <= 1000; i++) {
    calibracaoPista();
  }
  digitalWrite(13, LOW);
  delay(3000);
  digitalWrite(13, HIGH);
  for (int i = 0; i <= 1000; i++) {
    calibracaoLinha();
  }
  delay(100);
  resultadoCalibracao();
  digitalWrite(13, LOW);
}

void calibracaoPista() {
  for (int i = 0; i < 8; i++) {
    leituraPista[i] = analogRead(pinosSensores[i]);
    Serial.print(leituraPista[i]);
    Serial.print(" ");
    }
  Serial.println(" ");    
}

void calibracaoLinha() {
   for (int i = 0; i < 8; i++) {
    leituraLinha[i] = analogRead(pinosSensores[i]);
    Serial.print(leituraLinha[i]);
    Serial.print(" ");
    }
  Serial.println(" ");  
}


void resultadoCalibracao() {
  for (int i = 0; i < 8; i++) {
    linha[i] = (leituraPista[i] + leituraLinha[i])/2;
    Serial.print(linha[i]);
    Serial.print(" ");
    }
  Serial.println(" ");    
}


void PID() {
  proporcional = posicao - setpoint;
  derivativo = proporcional - ultimoProporcional;
  integral = erro1 + erro2 + erro3 + erro4 + erro5 + erro6;
  ultimoProporcional = proporcional;
  erro6 = erro5;
  erro5 = erro4;
  erro4 = erro3;
  erro3 = erro2;
  erro2 = erro1;
  erro1 = proporcional;
  int diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);
  if (diferencial > velocidadeRobo) diferencial = velocidadeRobo;
  else if (diferencial < -velocidadeRobo) diferencial = -velocidadeRobo;
  (diferencial < 0) ?
  motores(velocidadeRobo, velocidadeRobo + diferencial) : motores(velocidadeRobo - diferencial, velocidadeRobo);
}

void freios() {
  if (posicao <= 100) {
    motores(-velocidadeAtras, velocidadeFrente);
  }
  if (posicao >= 600) {
    motores(velocidadeFrente, -velocidadeAtras);
  }
}



















