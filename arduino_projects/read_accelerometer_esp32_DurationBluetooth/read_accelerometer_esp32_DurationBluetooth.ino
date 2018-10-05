// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t ax, ay, az;
//int16_t gx, gy, gz;

//aceleraciones calculadas
//float ax_m_s2 , ay_m_s2;
//float az_m_s2, gx_deg_s, gy_deg_, gz_deg_s;
uint16_t resultantBaseAcceleration, resultantAcceleration, shortMargin, largeMargin, accelerationOverMargin;

//timestamps y flags
unsigned long initialTimestamp, finalTimestamp, duration, dataToSend;
bool movementStarted, connectedToApp;
int trainingType;

void setup() {
  Serial.begin(57600);    //Iniciando puerto serial
  SerialBT.begin("TaekwonGo Band"); //Bluetooth device name

  Wire.begin();           //Iniciando I2C
  sensor.initialize();    //Iniciando el sensor acelerometro

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  //Umbrales y flags
  movementStarted = false;
  shortMargin = 3.0 / (9.81 / 16384.0);
  largeMargin = 10.0 / (9.81 / 16384.0);
  Serial.print("Margen menor:\t");
  Serial.println(shortMargin);
  Serial.print("Margen mayor:\t");
  Serial.println(largeMargin);

}

void initializeConnectionWithApp() {
  connectedToApp = true;
  Serial.print("SETUP Recibido:\t");
  Serial.print(trainingType);
  if (trainingType == 49) {
    //(caracter ASCII de 1)
    Serial.println(" (Fuerza)");
  }
  else {
    Serial.println(" (Velocidad)");
  }

  //Lee las aceleraciones iniciales.
  sensor.getAcceleration(&ax, &ay, &az);
  resultantBaseAcceleration = getResultantAcceleration(ax, ay, az);
  Serial.print("Aceleración base:\t");
  Serial.println(resultantBaseAcceleration);
}

void loop() {

  if (SerialBT.hasClient()) {
    while (!connectedToApp) {
      trainingType = SerialBT.read();
      if (trainingType != 0) {
        initializeConnectionWithApp();
        break;
      }
    }
    // Leer las aceleraciones
    sensor.getAcceleration(&ax, &ay, &az);
    resultantAcceleration = getResultantAcceleration(ax, ay, az);
    Serial.print("Aceleración resultante:\t");
    Serial.println(resultantAcceleration);


    if (movementStarted) {
      if (distanceToBase(resultantAcceleration) <= shortMargin) {
        Serial.print("Distance to base:\t");
        Serial.println(distanceToBase(resultantAcceleration));
        movementHasFinished();
      }
    }
    else {
      if (distanceToBase(resultantAcceleration) >= largeMargin) {
        Serial.print("Distance to base:\t");
        Serial.println(distanceToBase(resultantAcceleration));
        movementHasStarted();
      }
    }
    delay(10);
  }
  else
  {
    connectedToApp = false;
    delay(1000);
  }
}

void movementHasStarted() {
  //empezó el golpe
  Serial.println("Golpe iniciado");

  initialTimestamp = millis();
  accelerationOverMargin = resultantAcceleration;
  movementStarted = true;
}

void movementHasFinished() {

  //se completó el golpe
  Serial.println("Golpe completado");

  finalTimestamp = millis();
  duration = finalTimestamp - initialTimestamp;

  //enviarlo
  Serial.print("Movimiento finalizado. ");
  
  if (trainingType == 49) {
    //Fuerza (caracter ASCII de 1)
    Serial.print("Fuerza de impacto (raw):\t");
    dataToSend = (unsigned long) accelerationOverMargin;
  }
  else {
    //Velocidad
    Serial.print("Duración (ms):\t");
    dataToSend = duration;
  }

  Serial.println(dataToSend);
  char dataBuffer [sizeof(unsigned long) * 8 + 1];
  sprintf(dataBuffer, "%lu", dataToSend);
  int count = countDigits(dataToSend);

  SerialBT.write((uint8_t*)dataBuffer, sizeof(char)*count);
  SerialBT.write((uint8_t*)";", sizeof(char));

  movementStarted = false;
}

uint16_t getResultantAcceleration(int16_t x, int16_t y, int16_t z) {
  return sqrt(x * x + y * y + z * z);
}

uint16_t distanceToBase(uint16_t resultantAccel) {
  int16_t distance = resultantAccel - resultantBaseAcceleration;
  if (distance  < 0)
    return 0 - distance;
  else
    return distance;
}

int countDigits(unsigned long d) {
  int count = 0;
  unsigned long n = d;
  while (n != 0) {
    n /= 10;
    ++count;
  }
  return count;
}
