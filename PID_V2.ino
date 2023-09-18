// pinos ponte h
#define IN1   6 
#define IN2   5
#define IN3   9
#define IN4   10
// variaveis controle de velocidade
int PWMA;
int PWMB;

int leituraPista[8];
int leituraLinha[8];
int sensores[8];
int digital[8];

const int pinosSensores[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// pista = 0, pista branca com linha preta
// pista = 1, pista preta com linha branca
int pista = 0;
int linha[8];
long int somaPonderada, soma, posicao, posicaoLinha, ultimaPosicao;


void setup() {  
  Serial.begin(115200);
  pinMode(13, OUTPUT);

  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(pinosSensores[i], INPUT);
    }
  
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
void loop() {
  motores(250,250);
  delay(15);
  //posicaoLinha = leituraSensores();
  //Serial.println(posicaoLinha);
}

int leituraSensores(void) {
  for (int i = 0; i < 8; i++) {
    sensores[i] = analogRead(pinosSensores[i]);
    if(pista==1) { if(sensores[i] >= linha[i]) { digital[i] = 0; } else { digital[i] = 1;}}
    if(pista==0) { if(sensores[i] >= linha[i]) { digital[i] = 1;} else {digital[i] = 0;}}
    //Serial.print(digital[i]);
    //Serial.print(" ");
    }
  //Serial.println(" ");

  somaPonderada = (700*digital[0]) + (600*digital[1]) + (500*digital[2]) + (400*digital[3]) + (300*digital[4]) + (200*digital[5]) + (100*digital[6]) + (0*digital[7]);
  soma = (digital[0] + digital[1] + digital[2] + digital[3] + digital[4] + digital[5] + digital[6] + digital[7]);
  posicao = (somaPonderada / soma);

  if(ultimaPosicao<=100 && posicao==-1) {
    posicao = 0;
  }
   if(ultimaPosicao>=600 && posicao==-1) {
    posicao = 700;
  }

  ultimaPosicao = posicao;
  return posicao;
}

void motores(int direito, int esquerdo) {
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
