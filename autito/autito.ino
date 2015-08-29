#include <Servo.h>
#include "motors.h"
#include "ultrasonic_sensors.h"

char debug_string_buffer[50];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer); \
  delay(1);
  
const bool DEBUG = true;

// pines
const int led = 13; // digital out
const int sensorIzq = A0;
const int sensorCen = A1;
const int sensorDer = A2;
const int motorServo = 11; // PWM out
const int motorIzqDireccionA = 12; // digital out
const int motorIzqDireccionB = 8; // digital out
const int motorIzqVelocidad = 10; // PWM out
const int motorDerDireccionA = 7; // digital out
const int motorDerDireccionB = 4; // digital out
const int motorDerVelocidad = 9; // PWM out
const int sensorGiroRecta = 2; // digital in
const int sensorPelotas = A3;
const int usTrigger = 1; // digital out
const int us1Echo = A4;
const int buzzer = 0; // digital out

byte sensoresPiso = 0;
const int NNN = 0;
const int NNB = 1;
const int NBN = 2;
const int NBB = 3;
const int BNN = 4;
const int BNB = 5;
const int BBN = 6;
const int BBB = 7;

const bool ADELANTE = true;
const bool ATRAS = false;
const int velocidadMaxima = 150;

const int SEGUIR_DERECHO = 0;
const int CORREGIR_HACIA_IZQUIERDA = 1;
const int CORREGIR_HACIA_DERECHA = 2;
int estadoActual = SEGUIR_DERECHO;
int estadoAnterior = SEGUIR_DERECHO;

int valorSensor = 0;

Servo empujador;
motor motorIzq(motorIzqDireccionA, motorIzqDireccionB, motorIzqVelocidad);
motor motorDer(motorDerDireccionA, motorDerDireccionB, motorDerVelocidad);
UltrasonicSensors usSensor;

void setup() {
  empujador.attach(motorServo);

  pinMode(led, OUTPUT);
  pinMode(sensorIzq, INPUT);
  pinMode(sensorCen, INPUT);
  pinMode(sensorDer, INPUT);
  pinMode(motorServo, OUTPUT);
  pinMode(motorIzqDireccionA, OUTPUT);
  pinMode(motorIzqDireccionB, OUTPUT);
  pinMode(motorIzqVelocidad, OUTPUT);
  pinMode(motorDerDireccionA, OUTPUT);
  pinMode(motorDerDireccionB, OUTPUT);
  pinMode(motorDerVelocidad, OUTPUT);
  pinMode(sensorGiroRecta, INPUT);
  pinMode(sensorPelotas, INPUT);
  pinMode(usTrigger, OUTPUT);
  pinMode(us1Echo, INPUT);
  pinMode(buzzer, OUTPUT);

  servoDescanso();

  if (DEBUG) {
    Serial.begin(9600);  
  }
}

inline byte leerSensoresLinea() {
  return digitalRead(sensorIzq) + digitalRead(sensorCen) + digitalRead(sensorDer);
}

int obtenerDistanciaFrontal() {
  int medicion1 = usSensor.a();
  delay(10);
  int medicion2 = usSensor.a();
  delay(10);
  int medicion3 = usSensor.a();
  delay(10);
  long int suma = medicion1 + medicion2 + medicion3;
  return suma / 3;
}

void seguirLinea() {
  sensoresPiso = leerSensoresLinea();
  switch (estadoActual) {
    case SEGUIR_DERECHO:
      motorIzq(velocidadMaxima, ADELANTE);
      motorDer(velocidadMaxima, ADELANTE);
      if (sensoresPiso == NNB || sensoresPiso == NBB) {
        estadoActual = CORREGIR_HACIA_DERECHA;
      }
      if (sensoresPiso == BNN || sensoresPiso == BBN) {
        estadoActual = CORREGIR_HACIA_IZQUIERDA;
      }
      break;
    case CORREGIR_HACIA_DERECHA:
      motorIzq(velocidadMaxima, ADELANTE);
      motorDer(0, ADELANTE);
      if (sensoresPiso == NBN) {
        estadoActual = SEGUIR_DERECHO;
      }
      break;
    case CORREGIR_HACIA_IZQUIERDA:
      motorIzq(0, ADELANTE);
      motorDer(velocidadMaxima, ADELANTE);
      if (sensoresPiso == NBN) {
        estadoActual = SEGUIR_DERECHO;
      }
      break;
  }
}

void empujarPelota() {
  empujador.write(80); // golpe
  delay(100);
  empujador.write(180); // vuelve
  delay(100);
}

void girarIzquierda() {
  // gira a la izquierda aprox 90 grados,
  // hasta volver a encontrar la pista por delante
  
  motorIzq(0, ADELANTE);
  motorDer(velocidadMaxima, ADELANTE);
  //espera un poco para que los sensores queden afuera de la pista
  delay(50);
  while(1) {
    sensoresPiso = leerSensoresLinea();
    // si leo cualquier sensor de adelante, listo
    if (sensoresPiso) {
      motorIzq(0, ADELANTE);
      motorDer(0, ADELANTE);
      break;
    }
  }
}

void darVueltaAtras() {
  // esto ocurre en la zona de carga, donde no tenemos referencias
  // hace el giro de 180 grados en el lugar
  motorIzq(velocidadMaxima, ADELANTE);
  motorIzq(velocidadMaxima, ATRAS);
  delay(1800); // ojímetro
  motorIzq(0, ADELANTE);
  motorIzq(0, ADELANTE);
  delay(200);
}

void avanzarHastaParedDelantera() {
  // avanza hasta que está adelante de la pared
  motorIzq(velocidadMaxima, ADELANTE);
  motorDer(velocidadMaxima, ADELANTE);
  while (1) {
    valorSensor = obtenerDistanciaFrontal();
    if (valorSensor < 200) {
      motorIzq(0, ADELANTE);
      motorDer(0, ADELANTE);
      break;
    }
  }
}

void esperarPelotas() {
unsigned int timerPelota;
  // espera que una pelota quede un tiempo estacionada
  // en el sensor
  while(1) {
    timerPelota = millis();
    valorSensor = analogRead(sensorPelotas);
    if (valorSensor > 700) {
      if ((millis() - timerPelota) >= 2000) {
        break;
      }
    }
  }
  
  // hacer aviso sonoro
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
  delay(1000);
  digitalWrite(buzzer, HIGH);
  delay(1000);
  digitalWrite(buzzer, LOW);
  delay(1000);
}

void servoDescanso(void) {
  empujador.write(180);
}

void servoGolpe(void) {
  empujador.write(80);
}

void loop() {

  /*while(1) {
    motorIzq(255, ADELANTE);
    motorDer(255, ADELANTE);
    delay(1000);
    motorIzq(255, ATRAS);
    motorDer(255, ADELANTE);
    delay(1000);
    motorIzq(255, ATRAS);
    motorDer(255, ATRAS);
    delay(1000);
    motorIzq(255, ADELANTE);
    motorDer(255, ATRAS);
    delay(1000);
  }*/

  // INICIO FASE UNO
  // (recorrer el circuito hasta posicionarse en la zona de carga)
  // seguir linea hasta encontrar una recta a la izquierda
  while (1) {
    seguirLinea();
    valorSensor = digitalRead(sensorGiroRecta);
    if (valorSensor) {
      girarIzquierda();
      avanzarHastaParedDelantera();
      while(1) {
        // esperar intervención humana, o algo así
      }
      // FIN FASE UNO
      break;
    }
  }


  // INICIO FASE DOS
  // (recibir pelotas, avisar cuando esté listo, y salir de la zona de carga)
    esperarPelotas();
    darVueltaAtras();
    motorIzq(velocidadMaxima, ADELANTE);
    motorDer(velocidadMaxima, ADELANTE);
    delay(50); // ojímetro
    while(1) {
      seguirLinea();
      valorSensor = digitalRead(sensorGiroRecta);
      if (valorSensor) {
        girarIzquierda();
        while(1) {
          // esperar intervención humana, o algo así
        }
        // FIN FASE DOS
        break;
      }
    }

    // INICIO FASE TRES
    // (seguir por el camino)
    // seguir linea hasta encontrar una recta a la izquierda
    while(1) {
      seguirLinea();
      valorSensor = digitalRead(sensorGiroRecta);
      if (valorSensor) {
        girarIzquierda();
      }
    }

  
}
