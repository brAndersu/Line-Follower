#include <PID_v1.h>

// Pinos dos motores
const int ENA = 9;
const int ENB = 10;
const int in1 = 5;
const int in2 = 6;
const int in3 = 7;
const int in4 = 8;

// Parâmetros do PID
double Setpoint, Input, Output;
double Kp=0.007, Ki=0.00004, Kd=0.0000007;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
    Setpoint = 90; // Defina seu valor de referência aqui
    myPID.SetMode(AUTOMATIC);
}

void loop() {

    //Leitura dos sensores
    int L4 = analogRead(A3); //Sensor da direita
    int L5 = analogRead(A4); //Sensor da esquerda

    Input = L5 - L4; // Diferença entre os sensores
    myPID.Compute();

    // Agora você pode usar o valor de Output para controlar seus motores
    int baseSpeed = 75; // Velocidade base dos motores
    int maxChange = 75; // Máxima alteração permitida na velocidade dos motores

    // Calcula a velocidade de cada motor
    int motorSpeedA = baseSpeed + constrain(Output, -maxChange, maxChange);
    int motorSpeedB = baseSpeed - constrain(Output, -maxChange, maxChange);

    // Certifica-se de que a velocidade não ultrapasse o máximo permitido
    motorSpeedA = constrain(motorSpeedA, 0, 150);
    motorSpeedB = constrain(motorSpeedB, 0, 150);

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


  // //Ambos os motores girando para frente 
  // digitalWrite(in1, LOW);
  // digitalWrite(in2, HIGH);
  // digitalWrite(ENA, 255);
  // digitalWrite(in3, HIGH);
  // digitalWrite(in4, LOW);
  // digitalWrite(ENB, 255);










