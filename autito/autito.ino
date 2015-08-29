#include <Servo.h>

const int led = 13;
const int sensorGiroRecta = 4; // digital In
const int sensorIzq = 12; // digital In
const int sensorCen = 8; // digital In
const int sensorDer = 7; // digital In
const int motorServo = 11; // PWM out
const int motorIzq1 = 10; // PWM out
const int motorIzq2 = 9; // PWM out
const int motorDer1 = 6; // PWM out
const int motorDer2 = 5; // PWM out

const int usTrigger = 2;
const int us1Echo = A0;
const int us2Echo = A1;

byte sensoresPiso = 0;
const NNN = 0;
const NNB = 1;
const NBN = 2;
const NBB = 3;
const BNN = 4;
const BNB = 5;
const BBN = 6;
const BBB = 7;

Servo empujador;  // create servo object to control a servo

void setup() {
  empujador.attach(motorServo);

  pinMode(sensorIzq, INPUT);
  pinMode(sensorCen, INPUT);
  pinMode(sensorDer, INPUT);

  // motores no usan pinmode
  
}

inline byte leerSensoresLinea() {
  return digitalRead(sensorIzq) + digitalRead(sensorCen) + digitalRead(sensorDer);
}

int estadoActual = SEGUIR DERECHO;
int estadoAnterior = SEGUIR DERECHO;
const int SEGUIR DERECHO = 0;
const int CORREGIR_HACIA_IZQUIERDA = 1;
const int CORREGIR_HACIA_DERECHA = 2;
void actualizarEstado() {
  if (estadoActual == SEGUIR DERECHO) {
    motores(100, 50);
    if (sensores == NNB || sensores == NBB) {
      estado_actual = CORREGIR_HACIA_DERECHA;
    }
    if (sensores == BNN || sensores == BBN) {
      estado_actual = CORREGIR_HACIA_IZQUIERDA;
    }
  }
}

void loop() {
  //empujador.write(val);                  // sets the servo position according to the scaled value 
  //delay(15);                           // waits for the servo to get there 

  sensoresPiso = leerSensoresLinea();
  actualizarEstado();
  
  
}
