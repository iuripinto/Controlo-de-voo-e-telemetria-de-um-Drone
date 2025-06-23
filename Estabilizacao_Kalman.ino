#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU9250_asukiaaa.h>

//Definição de Pinos
const int motor1Pin = 4;  // Frente-Esquerda
const int motor2Pin = 5;  // Frente-Direita
const int motor3Pin = 18; // Trás-Esquerda
const int motor4Pin = 19; // Trás-Direita

Servo motor1, motor2, motor3, motor4;

//Definição do MPU6500
MPU9250_asukiaaa mpu;
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch,RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0,KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0,KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

void kalman_1d(float KalmanState,float KalmanUncertainty, float KalmanInput,float KalmanMeasurement) {
KalmanState=KalmanState+0.004*KalmanInput;
KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
KalmanUncertainty=(1-KalmanGain) *KalmanUncertainty;
Kalman1DOutput[0]=KalmanState;Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_signals(void) {
Wire.beginTransmission(0x68);
Wire.write(0x1A);
Wire.write(0x05);
Wire.endTransmission();
Wire.beginTransmission(0x68);
Wire.write(0x1C);
Wire.write(0x10);
Wire.endTransmission();
Wire.beginTransmission(0x68);
Wire.write(0x3B);
Wire.endTransmission();
Wire.requestFrom(0x68,6);
int16_t AccXLSB = Wire.read() << 8 | Wire.read();
int16_t AccYLSB = Wire.read() << 8 | Wire.read();
int16_t AccZLSB = Wire.read() << 8 | Wire.read();
Wire.beginTransmission(0x68);
Wire.write(0x1B);
Wire.write(0x8);
Wire.endTransmission();
Wire.beginTransmission(0x68);
Wire.write(0x43);
Wire.endTransmission();
Wire.requestFrom(0x68,6);
int16_t GyroX=Wire.read()<<8 | Wire.read();
int16_t GyroY=Wire.read()<<8 | Wire.read();
int16_t GyroZ=Wire.read()<<8 | Wire.read();
RateRoll=(float)GyroX/65.5;
RatePitch=(float)GyroY/65.5;
RateYaw=(float)GyroZ/65.5;
AccX=(float)AccXLSB/4096+0.02;
AccY=(float)AccYLSB/4096+0.01;
AccZ=(float)AccZLSB/4096+0.05;
AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

//Variáveis do PID
float kp = 1.5, ki = 0.0, kd = 0.3;  
float lastErrorPitch = 0, lastErrorRoll = 0;
float integralPitch = 0, integralRoll = 0;

int throttleBase = 1400; // Potência Base

void calibrateESC(Servo &motor) {
    motor.writeMicroseconds(2000);
    delay(4000);
    motor.writeMicroseconds(1000);
    delay(4000);
}

void setup() {
    Serial.begin(115200);

    // pinos dos motores
    motor1.attach(motor1Pin, 1000, 2000);
    motor2.attach(motor2Pin, 1000, 2000);
    motor3.attach(motor3Pin, 1000, 2000);
    motor4.attach(motor4Pin, 1000, 2000);

    Serial.println("Calibrando ESCs...");
    calibrateESC(motor1);
    calibrateESC(motor2);
    calibrateESC(motor3);
    calibrateESC(motor4);
    Serial.println("Calibração concluída!");

    // Garantir estado inicial seguro para o ESC
    Serial.println("Enviando sinal de parada inicial...");
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    delay(3000); 

    // Inicializar o MPU
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
    }
    RateCalibrationRoll/=2000;
    RateCalibrationPitch/=2000;
    RateCalibrationYaw/=2000;
    LoopTimer=micros();

    Serial.println("Sistema Pronto!");
}

void loop() {
    //Dados do filtro de kalman
    gyro_signals();
    RateRoll-=RateCalibrationRoll;
    RatePitch-=RateCalibrationPitch;
    RateYaw-=RateCalibrationYaw;
    kalman_1d(KalmanAngleRoll,KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0];KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
    Serial.print("Roll Angle [°] ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" Pitch Angle [°] ");
    Serial.println(KalmanAnglePitch);
    while (micros() - LoopTimer < 4000);
    LoopTimer=micros();

    // Cálculo do PID para Pitch
    float errorPitch = KalmanAnglePitch;  
    integralPitch += errorPitch;
    integralPitch = constrain(integralPitch, -300, 300);
    float derivativePitch = errorPitch - lastErrorPitch;
    float pidOutputPitch = (kp * errorPitch) + (ki * integralPitch) + (kd * derivativePitch);
    lastErrorPitch = errorPitch;

    //Cálculo do PID para Roll
    float errorRoll = -KalmanAngleRoll;
    integralRoll += errorRoll;
    integralRoll = constrain(integralRoll, -300, 300);
    float derivativeRoll = errorRoll - lastErrorRoll;
    float pidOutputRoll = (kp * errorRoll) + (ki * integralRoll) + (kd * derivativeRoll);
    lastErrorRoll = errorRoll;


    // Correções nos Motores 
    int motor1Speed = throttleBase + pidOutputPitch + pidOutputRoll;
    int motor2Speed = throttleBase + pidOutputPitch - pidOutputRoll;
    int motor3Speed = throttleBase - pidOutputPitch + pidOutputRoll;
    int motor4Speed = throttleBase - pidOutputPitch - pidOutputRoll;

    // Aplicação de limites para os motores
    motor1Speed = constrain(motor1Speed, 1000, 2000);
    motor2Speed = constrain(motor2Speed, 1000, 2000);
    motor3Speed = constrain(motor3Speed, 1000, 2000);
    motor4Speed = constrain(motor4Speed, 1000, 2000);

    // Envio de dados
    motor1.writeMicroseconds(motor1Speed);
    motor2.writeMicroseconds(motor2Speed);
    motor3.writeMicroseconds(motor3Speed);
    motor4.writeMicroseconds(motor4Speed);

    // Apresentação de dados 
    Serial.print("Pitch: ");
    Serial.print(KalmanAnglePitch);
    Serial.print(" | Roll: ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" | M1: ");
    Serial.print(motor1Speed);
    Serial.print(" | M2: ");
    Serial.print(motor2Speed);
    Serial.print(" | M3: ");
    Serial.print(motor3Speed);
    Serial.print(" | M4: ");
    Serial.println(motor4Speed);

    delay(20);
}
