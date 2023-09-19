#define NUM_SENSORS 8 // Número de sensores
#define CALIBRATION_TIME 5000 // Tempo de calibração em milissegundos

int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Pinagem dos sensores
int sensorWeights[NUM_SENSORS] = {-40, -30, -20, -10, 10, 20, 30, 40}; // Pesos dos sensores
int minValues[NUM_SENSORS]; // Vetor para armazenar os valores mínimos
int maxValues[NUM_SENSORS]; // Vetor para armazenar os valores máximos

// Parâmetros do PID
double Setpoint, Input, Output;
double errSum = 0, lastErr = 0;
double Kp=0.4, Ki=0, Kd=0;

void setup() {
  Serial.begin(9600);
  
  // Inicializa os vetores minValues e maxValues
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
    pinMode(sensorPins[i], INPUT);
  }

  // Inicializa o PID
  Setpoint = 5; // Defina o valor de referência aqui

  // Inicia a calibração
  calibrateSensors();
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
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = analogRead(sensorPins[i]);

    somaPonderada += (value*sensorWeights[i]);
    soma += value;
    

    Serial.print(value);
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
   // função para controlar o carro com base na saída do PID.
}
