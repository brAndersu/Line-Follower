// Define os pinos para o controle do motor
const int ligar_m1 = 3;
const int ligar_m2 = 4;
const int motorA1 = 5;
const int motorA2 = 6;
const int motorB1 = 7;
const int motorB2 = 8;

int sensorThreshold = 1000; // Valor do sensor para diminuir a velocidade
int motorSpeed = 255; // Velocidade máxima inicial

void setup()
{
    // Define os pinos como saídas
  pinMode(ligar_m1, OUTPUT);
  pinMode(ligar_m2, OUTPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  Serial.begin(9600);
  pinMode(2, OUTPUT);
  digitalWrite(2, 1);

    // Iniciar ambos os motores para a frente
  digitalWrite(ligar_m1, HIGH);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(ligar_m2, HIGH);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void loop()
{
  // Defini os pinos para cada um dos sensores
  int L1 = analogRead(A0);
  int L2 = analogRead(A1);
  int L3 = analogRead(A2);
  int L4 = analogRead(A3);
  int L5 = analogRead(A4);
  int L6 = analogRead(A5);
  int L7 = analogRead(A6);
  int L8 = analogRead(A7);

  if (L6 < 700) {
    // Se o sensor detectar uma alteração
    // Pare o motor B e continue girando o motor A
    digitalWrite(ligar_m2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }

  else {
    // Sensor está acima do limite, então reinicie o motor B
    digitalWrite(ligar_m2, HIGH);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }

  if (L3 < 700) {
    // Se o sensor detectar uma alteração
    // Pare o motor A e continue girando o motor B
    digitalWrite(ligar_m1, LOW);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
  }

  else {
    // Sensor está acima do limite, então reinicie o motor B
    digitalWrite(ligar_m1, HIGH);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  }

  Serial.print(L1);
  Serial.print(" ; ");
  Serial.print(L2);
  Serial.print(" ; ");
  Serial.print(L3);
  Serial.print(" ; ");
  Serial.print(L4);
  Serial.print(" ; ");
  Serial.print(L5);
  Serial.print(" ; ");
  Serial.print(L6);
  Serial.print(" ; ");
  Serial.print(L7);
  Serial.print(" ; ");
  Serial.println(L8);
  
  delay(500);  
}


