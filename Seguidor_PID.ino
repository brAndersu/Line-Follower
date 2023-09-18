// Pinos dos motores
const int ENA = 5;
const int ENB = 6;
const int in1 = 7;
const int in2 = 8;
const int in3 = 9;
const int in4 = 10;

// Parâmetros do PID
double Setpoint, Input, Output;
double errSum = 0, lastErr = 0;
double Kp=0.3, Ki=0.000008, Kd=0.000008;

void Compute() {
    double error = Setpoint - Input;
    errSum += error;
    double dErr = error - lastErr;

    Output = Kp * error + Ki * errSum + Kd * dErr;

    lastErr = error;
}

void setup()
{
    // Define os pinos como saídas
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    Serial.begin(9600);
    //Emiter PIN do sensor 
    pinMode(2, OUTPUT);
    digitalWrite(2, 1);

    // Inicializa o PID
    Setpoint = 5; // Defina o valor de referência aqui

    // Adiciona uma partida inicial para os motores
    analogWrite(ENA, 110); // Velocidade inicial para o motor A
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(ENB, 110); // Velocidade inicial para o motor B
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);

    delay(2000); // Mantém a velocidade inicial por 2 segundos

    analogWrite(ENA, 0); // Para o motor A
    analogWrite(ENB, 0); // Para o motor B
}

void loop() {

    //Leitura dos sensores
    int L4 = analogRead(A3); //Sensor da direita
    int L5 = analogRead(A4); //Sensor da esquerda

    Input = L5 - L4; // Diferença entre os sensores
    Compute();

//     // Ajusta o sinal de Output com base no sinal de Input
//     if (Input < 0 && Output > 0) {
//       Output = -Output;
// }

    // Aqui e possivel usar o valor de Output para controlar seus motores
    int baseSpeed = 125; // Velocidade base dos motores
    int maxChange = 125; // Máxima alteração permitida na velocidade dos motores

    // Calcula a velocidade de cada motor
    int motorSpeedA = constrain(baseSpeed - Output, 0, 250);
    int motorSpeedB = constrain(baseSpeed + Output, 0, 250);

    // Certifica-se de que a velocidade não ultrapasse o máximo permitido
    motorSpeedA = constrain(motorSpeedA, 0, 125);
    motorSpeedB = constrain(motorSpeedB, 0, 125);

    Serial.print("Sensor da direita: ");
    Serial.print(L4);
    Serial.print(" Sensor da esquerda: ");
    Serial.print(L5);
    Serial.print("Velocidade A: ");
    Serial.print(motorSpeedA);
    Serial.print(" Velocidade B: ");
    Serial.print(motorSpeedB);
    Serial.print(" Output: ");
    Serial.print(Output);
    Serial.print(" Input :");
    Serial.println(Input);

   // Define a velocidade dos motores
   analogWrite(ENA, motorSpeedA);
   analogWrite(ENB, motorSpeedB);
}
