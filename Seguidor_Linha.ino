#include <QTRSensors.h>

#define NUM_SENSORS       2  // Número de sensores no sensor QTR-8RC
#define TIMEOUT           2500  // Tempo limite para a leitura de cada sensor (microssegundos)
#define EMITTER_PIN       2  // Pino para ligar/desligar o emissor de IR (2 no QTR-8RC)

// Crie um objeto QTRSensorsRC para ler os sensores
QTRSensorsRC qtrrc((unsigned char[]) { 3, 4}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

// Define os pinos para o controle do motor
const int motor1EnablePin = 8;
const int motor2EnablePin = 9;
const int motor1Input1 = 10;
const int motor1Input2 = 11;
const int motor2Input1 = 12;
const int motor2Input2 = 13;

int sensorThreshold = 1000; // Valor do sensor para diminuir a velocidade
int motorSpeed = 255; // Velocidade máxima inicial

void setup()
{
    // Define os pinos como saídas
  pinMode(motor1EnablePin, OUTPUT);
  pinMode(motor2EnablePin, OUTPUT);
  pinMode(motor1Input1, OUTPUT);
  pinMode(motor1Input2, OUTPUT);
  pinMode(motor2Input1, OUTPUT);
  pinMode(motor2Input2, OUTPUT);

  // Inicialize a comunicação serial
  Serial.begin(9600);

  // Configurar o emissor de IR
  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH); // Ligue o emissor de IR
  delay(500); // Aguarde 500ms para o emissor estabilizar

  // Calibrar os sensores
  Serial.println("Calibrando os sensores...");
  delay(1000);
  qtrrc.calibrate();
  Serial.println("Calibração concluída!");
  delay(1000);

    // Iniciar ambos os motores para a frente
  digitalWrite(motor1EnablePin, HIGH);
  digitalWrite(motor1Input1, HIGH);
  digitalWrite(motor1Input2, LOW);
  digitalWrite(motor2EnablePin, HIGH);
  digitalWrite(motor2Input1, HIGH);
  digitalWrite(motor2Input2, LOW);
}

void loop()
{
  // Lê os valores dos sensores
  unsigned int sensorValues[NUM_SENSORS];
  qtrrc.read(sensorValues);

  // Verifique o valor do sensor conectado ao pino 3 (índice 0 no array)
  if (sensorValues[0] < 1000) {
    // Se o sensor detectar uma alteração (ajuste o valor de limite conforme necessário)
    // Pare o motor A e continue girando o motor B
    digitalWrite(motor1EnablePin, LOW);
    digitalWrite(motor1Input1, LOW);
    digitalWrite(motor1Input2, LOW);
  }

  else {
    // Sensor está acima do limite, então reinicie o motor A
    digitalWrite(motor1EnablePin, HIGH);
    digitalWrite(motor1Input1, HIGH); // Define a direção do motor A
    digitalWrite(motor1Input2, LOW);
  }

    // Verifique o valor do sensor conectado ao pino 7 (índice 4 no array)
  if (sensorValues[1] < 1000) {
    // Se o sensor detectar uma alteração (ajuste o valor de limite conforme necessário)
    // Pare o motor B e continue girando o motor A
    digitalWrite(motor2EnablePin, LOW);
    digitalWrite(motor2Input1, LOW);
    digitalWrite(motor2Input2, LOW);
  }

  else {
    // Sensor está acima do limite, então reinicie o motor B
    digitalWrite(motor2EnablePin, HIGH);
    digitalWrite(motor2Input1, HIGH); // Define a direção do motor B
    digitalWrite(motor2Input2, LOW);
  }

  // Imprime os valores dos sensores lado a lado no Monitor Serial
  Serial.print("Valores dos sensores: ");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(" "); // Adicione um espaço entre os valores
  }
  Serial.println(); // Adicione uma quebra de linha no final

  delay(100); // Pequeno atraso para evitar leituras muito rápidas
}










void setup() {
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  digitalWrite(2, 1);
}

void loop() {

  int L1 = analogRead(A0);
  Serial.print(L1);
  Serial.print(" ; ");
  int L8 = analogRead(A1);
  Serial.println(L8);
  delay(500);  
}
