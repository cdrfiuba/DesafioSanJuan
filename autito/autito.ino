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
const int usTrigger = A4; 
const int us1Echo = 1; // digital out
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
const int velocidadMaxima = 255;
const int DISTANCIA_FRENADO_FRONTAL = 15;
const int VALOR_BLANCO = 700;

const int SEGUIR_DERECHO = 0;
const int CORREGIR_HACIA_IZQUIERDA = 1;
const int CORREGIR_HACIA_DERECHA = 2;
int estadoActual = SEGUIR_DERECHO;

long unsigned int ultimoTiempoSensorIzq = 0;
long unsigned int ultimoTiempoSensorDer = 0;

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
  //pinMode(usTrigger, OUTPUT);
  //pinMode(us1Echo, INPUT);
  pinMode(buzzer, OUTPUT);

  servoDescanso();

  motorIzq(0, ADELANTE);
  motorDer(0, ADELANTE);


  if (DEBUG) {
    Serial.begin(9600);  
  }
}

inline byte leerSensoresLinea() {
  int sensor1 = (analogRead(sensorIzq) < VALOR_BLANCO) ? 4 : 0;
  int sensor2 = (analogRead(sensorCen) < VALOR_BLANCO) ? 2 : 0;
  int sensor3 = (analogRead(sensorDer) < VALOR_BLANCO) ? 1 : 0;
  if (sensor1) {
    ultimoTiempoSensorIzq = millis();
  }  
  if (sensor3) {
    ultimoTiempoSensorDer = millis();
  }  
  return sensor1 + sensor2 + sensor3;
}
inline int leerSensorGiro() {
  return ((digitalRead(sensorGiroRecta) == 0) ? 1 : 0);
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

void frenar() {
  motorIzq(0, ADELANTE);
  motorDer(0, ADELANTE);
}

void girarIzquierda() {
  // gira a la izquierda aprox 90 grados,
  // hasta volver a encontrar la pista por delante
  motorIzq(velocidadMaxima, ATRAS);
  motorDer(velocidadMaxima, ATRAS);
  delay(5);
  frenar();
  delay(1000);
  while(!leerSensorGiro()){
    motorIzq(velocidadMaxima, ATRAS);
    motorDer(velocidadMaxima, ATRAS);
  }
  
  sensoresPiso = leerSensoresLinea();


  while(sensoresPiso) {
    sensoresPiso = leerSensoresLinea();
    motorIzq(velocidadMaxima, ATRAS);
    motorDer(velocidadMaxima, ADELANTE);
  }
  frenar();
  delay(1000);
  while(!sensoresPiso) {
    sensoresPiso = leerSensoresLinea();
    motorIzq(velocidadMaxima/2, ATRAS);
    motorDer(velocidadMaxima, ADELANTE);
  }
  frenar();
  delay(1000);
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
    if (valorSensor < DISTANCIA_FRENADO_FRONTAL) {
      frenar();
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

void encontrarCuarto() {
  while(1) {
    seguirLinea();
    valorSensor = leerSensorGiro();
    if (valorSensor) {
      girarIzquierda();
      estadoActual = SEGUIR_DERECHO;
      while(leerSensoresLinea() != BBB) seguirLinea();
      frenar();
      delay(1000);

      digitalWrite(buzzer, HIGH);
      int diferencia = ultimoTiempoSensorIzq - ultimoTiempoSensorDer;
      long int tiempoProporcional = abs(diferencia) * 20000;
      if (diferencia > 0) {
        motorIzq(0, ADELANTE);
        motorDer(velocidadMaxima, ATRAS);
      } else {
        motorIzq(velocidadMaxima, ATRAS);
        motorDer(0, ADELANTE);
      }
      delay(tiempoProporcional);
      frenar();
      digitalWrite(buzzer, LOW);
      
      /*
      while(leerSensoresLinea() != BBB){ 
        if (!(leerSensoresLinea() & 0b00000001))
          motorDer(velocidadMaxima, ATRAS);
        else
          motorDer(0, ATRAS);
          
        if (!(leerSensoresLinea() & (1<<2)))
          motorIzq(velocidadMaxima, ATRAS);
        else
          motorIzq(0, ATRAS);          
      }
      */
      return;
    }
  }
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
  
  // countdown pal arranque
  digitalWrite(buzzer, HIGH); delay(2000);
  digitalWrite(buzzer, LOW);  delay(2000);

  // INICIO FASE UNO
  // (recorrer el circuito hasta posicionarse en la zona de carga)
  // seguir linea hasta encontrar una recta a la izquierda
  encontrarCuarto();
  avanzarHastaParedDelantera();
  while(1) {
    // esperar intervención humana, o algo así
  }
  // FIN FASE UNO


    



  // INICIO FASE DOS
  // (recibir pelotas, avisar cuando esté listo, y salir de la zona de carga)
    esperarPelotas();
    darVueltaAtras();
    motorIzq(velocidadMaxima, ADELANTE);
    motorDer(velocidadMaxima, ADELANTE);
    delay(50); // ojímetro
    while(1) {
      seguirLinea();
      valorSensor = leerSensorGiro();
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
      valorSensor = leerSensorGiro();
      if (valorSensor) {
        girarIzquierda();
      }
    }

  
}
