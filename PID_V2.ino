#define NUM_SENSORS 8 // Número de sensores
#define CALIBRATION_TIME 5000 // Tempo de calibração em milissegundos

int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Pinagem dos sensores
int sensorWeights[NUM_SENSORS] = {-4, -3, -2, -1, 1, 2, 3, 4}; // Pesos dos sensores
int minValues[NUM_SENSORS]; // Vetor para armazenar os valores mínimos
int maxValues[NUM_SENSORS]; // Vetor para armazenar os valores máximos

double kp = 1.0; // Constante proporcional
double ki = 0.0; // Constante integral
double kd = 0.0; // Constante derivativa

double error = 0.0; // Erro atual
double lastError = 0.0; // Erro anterior
double integral = 0.0; // Integral do erro

void setup() {
  Serial.begin(9600);
  
  // Inicializa os vetores minValues e maxValues
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
    pinMode(sensorPins[i], INPUT);
  }

  // Inicia a calibração
  calibrateSensors();
}

void loop() {
  double position = readLinePosition(); // Lê a posição atual na linha

  // Calcula o erro
  error = position;

  // Calcula a integral do erro
  integral += error;

  // Calcula a derivada do erro
  double derivative = error - lastError;

  // Calcula a saída do PID
  double output = kp * error + ki * integral + kd * derivative;

  // Atualiza o erro anterior
  lastError = error;

  // Usa a saída do PID para controlar o carro
  controlCar(output);
}

void calibrateSensors() {
  unsigned long startTime = millis();
  while (millis() - startTime < CALIBRATION_TIME) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int value = analogRead(sensorPins[i]);
      minValues[i] = min(minValues[i], value);
      maxValues[i] = max(maxValues[i], value);
    }
  }
}

double readLinePosition() {
  int sum = 0;
  int total = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = analogRead(sensorPins[i]);
    int normalizedValue = (value - minValues[i]) / (double)(maxValues[i] - minValues[i]);
    
    sum += normalizedValue * sensorWeights[i];
    total += normalizedValue;
  }
  
  return total == 0 ? 0 : (double)sum / total;
}

void controlCar(double output) {
   // Implemente esta função para controlar seu carro com base na saída do PID.
}
