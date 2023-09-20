#define NUM_SENSORS 8 // Número de sensores
#define CALIBRATION_TIME 5000 // Tempo de calibração em milissegundos

int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Pinagem dos sensores
int sensorWeights[NUM_SENSORS] = {-40, -30, -20, -10, 10, 20, 30, 40}; // Pesos dos sensores
int minValues[NUM_SENSORS]; // Vetor para armazenar os valores mínimos
int maxValues[NUM_SENSORS]; // Vetor para armazenar os valores máximos

const int motorEsquerdo = 5;
const int motorDireito = 6;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 9;
const int IN4 = 10;

// Parâmetros do PID
double Setpoint, Input, Output;
double errSum = 0, lastErr = 0;
double Kp=1.3, Ki=0.2, Kd=2;

void setup() {
  Serial.begin(9600);
  
  // Inicializa os vetores minValues e maxValues
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
    pinMode(sensorPins[i], INPUT);
  }

  // Inicializa o PID
  Setpoint = 0; // Defina o valor de referência aqui
  // pinMode(13, LOW);
  // delay(3000);
  // pinMode(13, HIGH);
  // delay(2000);
  // // Inicia a calibração
  // calibrateSensors();
  // pinMode(13, LOW);
  // delay(2000);

}

void loop() {
  double position = readLinePosition(); // Lê a posição atual na linha
  Serial.print(" Input: ");
  Serial.print(position);

  Input = position;

  Compute(); // Calcula a saída do PID

  Serial.print(" output: ");
  Serial.println(Output);

  // Usa a saída do PID para controlar o carro
  controlCar(Output);
}

void calibrateSensors() {
  unsigned long startTime = millis();
  while (millis() - startTime < CALIBRATION_TIME) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int value = analogRead(sensorPins[i]);
      minValues[i] = min(minValues[i], value);
      maxValues[i] = max(maxValues[i], value);
      Serial.print("Valor Maximo: ");
      Serial.print(maxValues[i]);
      Serial.print("Valor Minimo: ");
      Serial.println(minValues[i]);
    }
  }
}

double readLinePosition() {
  double somaPonderada = 0;
  double soma = 0;
  double limmax = 1023;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = analogRead(sensorPins[i]);

    int valor = limmax - value;
    somaPonderada += (valor*sensorWeights[i]);
    soma += valor;
    

    Serial.print(valor);
    Serial.print(" ; ");
  }
  
  double posicao = (soma != 0) ? somaPonderada/soma : 0; // Evita divisão por zero
  
  Serial.print("Posicao: ");
  Serial.print(posicao);
  
  return posicao; // Retorna a posição calculada
}


void Compute() {
  double error = Setpoint - Input;
  errSum += error;
  double dErr = error - lastErr;

  Output = Kp * error + Ki * errSum + Kd * dErr;

  lastErr = error;
}

void controlCar(double output) {
  int speed = 65; // Velocidade base
  int leftSpeed = speed + abs(output);
  int rightSpeed = speed + abs(output);

  if (output > 0) {
    rightSpeed -= 2*abs(output);
  } else {
    leftSpeed -= 2*abs(output);
  }

  // Garante que a velocidade esteja entre 0 e 140
  leftSpeed = constrain(leftSpeed, 0, 150);
  rightSpeed = constrain(rightSpeed, 0, 150);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(motorEsquerdo, leftSpeed);
  analogWrite(motorDireito, rightSpeed);

  Serial.print("Motor Esquerdo: ");
  Serial.print(leftSpeed);
  Serial.print(" Motor Direito: ");
  Serial.print(rightSpeed);
  Serial.print(" ");
}
