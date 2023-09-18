// Define os pinos para o controle do motor
const int ligar_m1 = 10;
const int ligar_m2 = 11;
const int motorA1 = 5;
const int motorA2 = 6;
const int motorB1 = 7;
const int motorB2 = 8;

int motorSpeed = 82;
int motorSpeed_Curve = 92;


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
  analogWrite(ligar_m1, motorSpeed);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  analogWrite(ligar_m2, motorSpeed);
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

  int limiar = 900;

  if (L6 > limiar) {
    // Se o sensor detectar uma alteração
    // Pare o motor B e continue girando o motor A
    // digitalWrite(ligar_m2, LOW);
    analogWrite(ligar_m1, motorSpeed_Curve);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }

  else {
    // Sensor está acima do limite, então reinicie o motor B
    // digitalWrite(ligar_m2, HIGH);
    analogWrite(ligar_m1, motorSpeed);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }

  if (L3 > limiar) {
    // Se o sensor detectar uma alteração
    // Pare o motor A e continue girando o motor B
    // digitalWrite(ligar_m1, LOW);
    analogWrite(ligar_m2, motorSpeed_Curve);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
  }

  else {
    // Sensor está acima do limite, então reinicie o motor B
    // digitalWrite(ligar_m1, HIGH);
    analogWrite(ligar_m2, motorSpeed);
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
  
  delay(10);  
}

//teste
