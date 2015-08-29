#include <Servo.h>
#include "motors.h"
#include "ultrasonic_sensors.h"

char debug_string_buffer[50];
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer); \
  delay(1);
  
const bool DEBUG = true;

int faseActual = 4;
const int esperarEnFinDeFase = false;
const int ejecutarPreparacionFaseTres = false;

// pines
const int buzzer = 13; // PWM out
const int sensorIzq = A0;
const int sensorCen = A1;
const int sensorDer = A2;
const int motorServo = 11; // PWM out
const int motorIzqDireccionA = 12; // digital out
const int motorIzqDireccionB = 8; // digital out
const int motorIzqVelocidad = 6; // PWM out
const int motorDerDireccionA = 7; // digital out
const int motorDerDireccionB = 4; // digital out
const int motorDerVelocidad = 5; // PWM out
const int sensorGiroRecta = 2; // digital in
const int sensorPelotas = A3;
const int usTrigger = A4; 
const int us1Echo = 1; // digital out
const int led = 0; // digital out

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
const int velocidadMaximaGiro = 255;
const int DISTANCIA_FRENADO_FRONTAL = 15;
const int VALOR_BLANCO = 700;

const int SEGUIR_DERECHO = 0;
const int CORREGIR_HACIA_IZQUIERDA = 1;
const int CORREGIR_HACIA_DERECHA = 2;
int estadoActual = SEGUIR_DERECHO;

long unsigned int ultimoTiempoSensorIzq = 0;
long unsigned int ultimoTiempoSensorDer = 0;
long unsigned int tiempo = 0;

int valorSensor = 0;

// para calcular tiempo entre ciclos de PID.
// no debe ser 0, pues se usa para dividir
long int ultimoTiempoUs = 0; // guarda el valor de micros()
int tiempoUs = 0; // guarda el tiempo del ciclo
const int tiempoCicloReferencia = 500;

const int rangoVelocidad = velocidadMaxima;
const int velocidadFreno = 20;
int velocidadMotorFrenado;
const int haciaIzquierda = 0;
const int haciaDerecha = 1;
int direccionMovimientoLateral;

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
  //pinMode(motorServo, OUTPUT);
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
  pinMode(led, OUTPUT);

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
void seguirLineaPID() {
  int sensoresLinea = 0;
  const int coeficienteErrorPmult = 1;
  const int coeficienteErrorPdiv = 7;
  const int coeficienteErrorIdiv = 2500;
  const int coeficienteErrorDmult = 19;
  const int coeficienteErrorDdiv = 1;
  const int centroDeLinea = 1000;
  int reduccionVelocidad;
  int errP;
  int errPAnterior;
  int errI;
  int errD;
    
  int sensor1 = analogRead(sensorIzq);
  int sensor2 = analogRead(sensorCen);
  int sensor3 = analogRead(sensorDer);

  sensoresLinea = (
    (long)sensor1    * 0 + 
    (long)sensor2    * 1000 + 
    (long)sensor3    * 2000
  ) / (
    (long)sensor1    + 
    (long)sensor2    + 
    (long)sensor3
  );
  
  errP = sensoresLinea - centroDeLinea;
  errI = errP * tiempoCicloReferencia / tiempoUs;
  errD = (errP - errPAnterior) * tiempoUs / tiempoCicloReferencia;
  errPAnterior = errP;
  reduccionVelocidad = (errP * coeficienteErrorPmult) / coeficienteErrorPdiv  + (errD * coeficienteErrorDmult) / coeficienteErrorDdiv + errI / coeficienteErrorIdiv;

  // constrain
  if (reduccionVelocidad < -rangoVelocidad - velocidadFreno) {
    reduccionVelocidad = -rangoVelocidad - velocidadFreno;
  } else if (reduccionVelocidad > rangoVelocidad + velocidadFreno) {
    reduccionVelocidad = rangoVelocidad + velocidadFreno;
  }

  if (reduccionVelocidad < 0) {
    direccionMovimientoLateral = haciaIzquierda;
  } else {
    direccionMovimientoLateral = haciaDerecha;
  }

  reduccionVelocidad = abs(reduccionVelocidad);
  velocidadMotorFrenado = abs(rangoVelocidad - reduccionVelocidad);
  
  if (direccionMovimientoLateral == haciaIzquierda) {
    // si la reducción es mayor al rango de velocidad, 
    // uno de los motores va para atrás
    if (reduccionVelocidad > rangoVelocidad) {
      motorIzq(velocidadMotorFrenado, ATRAS);
      motorDer(rangoVelocidad, ADELANTE);
    } else {
      motorIzq(velocidadMotorFrenado, ADELANTE);
      motorDer(rangoVelocidad, ADELANTE);
    }
  } else if (direccionMovimientoLateral == haciaDerecha) {
    // si la reducción es mayor al rango de velocidad, 
    // uno de los motores va para atrás
    if (reduccionVelocidad > rangoVelocidad) {
      motorIzq(rangoVelocidad, ADELANTE);
      motorDer(velocidadMotorFrenado, ATRAS);
    } else {
      motorIzq(rangoVelocidad, ADELANTE);
      motorDer(velocidadMotorFrenado, ADELANTE);
    }
  }
      
  if (DEBUG) {
    tiempoUs = micros() - ultimoTiempoUs;
    debug("%.4i ", tiempoUs);
    debug("% .4i ", sensoresLinea);
    debug("% .4i ", errP);
    debug("% .3i\n", reduccionVelocidad);
  }  
  tiempoUs = micros() - ultimoTiempoUs;
  ultimoTiempoUs = micros();
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
  motorIzq(velocidadMaximaGiro, ATRAS);
  motorDer(velocidadMaximaGiro, ATRAS);
  delay(5);
  frenar();
  delay(1000);
  
  while(!leerSensorGiro()){
    motorIzq(velocidadMaximaGiro, ATRAS);
    motorDer(velocidadMaximaGiro, ATRAS);
  }

  sensoresPiso = leerSensoresLinea();
  while(sensoresPiso) {
    sensoresPiso = leerSensoresLinea();
    motorIzq(velocidadMaximaGiro, ATRAS);
    motorDer(velocidadMaximaGiro / 2, ADELANTE);
  }
  frenar();
  delay(1000);
  while(!sensoresPiso) {
    sensoresPiso = leerSensoresLinea();
    motorIzq(velocidadMaximaGiro / 2, ATRAS);
    motorDer(velocidadMaximaGiro, ADELANTE);
  }
  frenar();
  delay(1000);
}

void girarIzquierdaFijo() {
  // gira a la izquierda aprox 90 grados,
  // hasta volver a encontrar la pista por delante
  motorIzq(velocidadMaximaGiro, ATRAS);
  motorDer(velocidadMaximaGiro, ATRAS);
  delay(5);
  frenar();
  delay(1000);
  
  while(!leerSensorGiro()){
    motorIzq(velocidadMaximaGiro, ATRAS);
    motorDer(velocidadMaximaGiro, ATRAS);
  }

  sensoresPiso = leerSensoresLinea();
//  while(sensoresPiso) {
//    sensoresPiso = leerSensoresLinea();
//    motorIzq(velocidadMaximaGiro, ATRAS);
//    motorDer(velocidadMaximaGiro / 2, ADELANTE);
//  }
//  frenar();
//  delay(1000);
  while(!sensoresPiso) {
    sensoresPiso = leerSensoresLinea();
    motorIzq(velocidadMaximaGiro/2, ATRAS);
    motorDer(velocidadMaximaGiro/2, ADELANTE);
  }
  frenar();
  delay(1000);
}


void buscarAnguloEntrada() {
  
}

void darVueltaAtras() {
  // esto ocurre en la zona de carga, donde no tenemos referencias
  // hace el giro de 180 grados en el lugar
  motorIzq(velocidadMaxima, ATRAS);
  motorDer(velocidadMaxima, ATRAS);
  delay(700); // ojímetro
  motorIzq(velocidadMaxima, ADELANTE);
  motorDer(velocidadMaxima, ATRAS);
  delay(1450); // ojímetro
  motorIzq(0, ADELANTE);
  motorDer(0, ADELANTE);
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
    valorSensor = analogRead(sensorPelotas);
    
    if (valorSensor > 800) {
      delay(500);
      valorSensor = analogRead(sensorPelotas);
      if (valorSensor > 800) {
        delay(500);
        valorSensor = analogRead(sensorPelotas);
        if (valorSensor > 800) {
          delay(500);
          valorSensor = analogRead(sensorPelotas);
          if (valorSensor > 800) {
              break;
          }
        }
      }
    }
  }
  
  // hacer aviso sonoro
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(500);
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(500);
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

      digitalWrite(led, HIGH);
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
      digitalWrite(led, LOW);
      
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
  digitalWrite(led, HIGH); delay(2000);
  digitalWrite(led, LOW);  delay(2000);

  if (faseActual == 1) {
    // INICIO FASE UNO
    // (recorrer el circuito hasta posicionarse en la zona de carga)
    // seguir linea hasta encontrar una recta a la izquierda
    encontrarCuarto();
    avanzarHastaParedDelantera();
    if (esperarEnFinDeFase) {
      while(1) {
        // esperar intervención humana, o algo así
      }
    }
    // FIN FASE UNO
    faseActual++;
  }

  if (faseActual == 2) {
    // INICIO FASE DOS
    // (recibir pelotas, avisar cuando esté listo, y salir de la zona de carga)
      esperarPelotas();
      darVueltaAtras();
      motorIzq(velocidadMaxima, ADELANTE);
      motorDer(velocidadMaxima, ADELANTE);
      delay(3000); // ojímetro
      estadoActual = SEGUIR_DERECHO;
      while(1) {
        seguirLinea();
        valorSensor = leerSensorGiro();
        if (valorSensor) {
          if (esperarEnFinDeFase) {
            while(1) {
              // esperar intervención humana, o algo así
            }
          }
          // FIN FASE DOS
          faseActual++;
          break;
        }
      }
  }
  if (faseActual == 3 && ejecutarPreparacionFaseTres) {
    // preparacion especial para la fase 3
    while(1) {
      seguirLinea();
      valorSensor = leerSensorGiro();
      if (valorSensor) {
        // fin de la preparación de la fase tres
        break;
      }
    }
  }
  if (faseActual == 3) {
    // INICIO FASE TRES
    // (seguir por el camino hasta entrar a primera zona de descarga)
    // seguir linea hasta encontrar una recta a la izquierda
    while(1) {
      girarIzquierda();
      encontrarCuarto();
    }
    // FIN FASE TRES
    faseActual++;
  }
  
  if (faseActual == 4) {
    // INICIO FASE TRES
    // (descargar pelotas en cuartos)
    resolverCuarto();
    avanzarCuarto();
  
    resolverCuarto();
    avanzarCuarto();    
    
    resolverCuarto();
    avanzarCuarto();

    
    // FIN FASE CUATRO
    faseActual++;
  }

  if (faseActual == 5) {
    while(1) {
     digitalWrite(led, HIGH); delay(200); 
     digitalWrite(led, LOW);  delay(200); 
    }
  }
  
}

const int DISTANCIA_OBSTACULO = 35;

void resolverCuarto (){
  if (buscarObstaculo() == true){
    orientarse();
    emitirRuido();
    delay(1000);
  } else {
    orientarse();
    depositarPelota();
    delay(1000);
  }
}

void avanzarCuarto() {
  // avanzar hasta que el sensor de giro esté en la puerta
  while(1) {
    seguirLinea();
    valorSensor = leerSensorGiro();
    if (valorSensor){
      break;
    }
  }
  // avanzar hasta que el sensor de giro deje de estar en la puerta
  while(1) {
    seguirLinea();
    valorSensor = leerSensorGiro();
    if (!valorSensor){
      break;
    }
  }
  // seguir línea hasta salir de la T (donde no hay pista),
  // ahí avanzar derecho hasta detectar el giro
  while(1) {
    seguirLinea();
    // 
    if (leerSensoresLinea() == BBB) {
      while(1){
        motorIzq(velocidadMaxima, ADELANTE);
        motorDer(velocidadMaxima, ADELANTE);
        valorSensor = leerSensorGiro();
        if (valorSensor) {
          frenar();
          delay(1000);
          girarIzquierdaFijo();
          valorSensor = leerSensorGiro();
          tiempo = millis();
          while(millis()- tiempo < 1000){
            seguirLinea();
          }
          encontrarCuarto();
          return;
        }
      }
    }
  }
  girarIzquierda();
  encontrarCuarto();
  return;
    
  
}

bool buscarObstaculo() {
  valorSensor = obtenerDistanciaFrontal();
  if (valorSensor < DISTANCIA_OBSTACULO)
    return true;
  else
    return false;
}

void emitirRuido(){
  return;
}

void depositarPelota(){
  servoGolpe();
  delay(200);
  servoDescanso();
}

void orientarse(){
  while(!leerSensorGiro()){
    motorIzq(velocidadMaximaGiro, ATRAS);
    motorDer(velocidadMaximaGiro, ADELANTE);
  }
  frenar();
  delay(1000);
 
  sensoresPiso = leerSensoresLinea();
  while(!sensoresPiso) {
    sensoresPiso = leerSensoresLinea();
    motorIzq(velocidadMaximaGiro, ATRAS);
    motorDer(velocidadMaximaGiro, ADELANTE);
  }
  frenar();
  delay(1000);
}



