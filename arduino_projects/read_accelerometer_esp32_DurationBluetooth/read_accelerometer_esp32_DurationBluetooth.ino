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
float ax_m_s2 , ay_m_s2;
//float az_m_s2, gx_deg_s, gy_deg_, gz_deg_s;
float resultantBaseAcceleration, resultantAcceleration, shortMargin, largeMargin;

//timestamps y flags
unsigned long initialTimestamp, finalTimestamp, duration;
bool movementStarted;

void setup() {
  Serial.begin(57600);    //Iniciando puerto serial  
  SerialBT.begin("TaekwonGo Band"); //Bluetooth device name
  
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor acelerometro

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  //Lee las aceleraciones iniciales. 
  //Quizas esto haya que llevarlo al momento en que el celular se conecta!
  sensor.getAcceleration(&ax, &ay, &az);
  ax_m_s2 = ax * (9.81/16384.0);
  ay_m_s2 = ay * (9.81/16384.0);  
  resultantBaseAcceleration = getResultantAcceleration(ax_m_s2, ay_m_s2);
    
  //Umbrales y flags
  movementStarted = false;
  shortMargin = 5.0;
  largeMargin = 8.5;
  
}

void loop() {
  
  // Leer las aceleraciones
  sensor.getAcceleration(&ax, &ay, &az);
  ax_m_s2 = ax * (9.81/16384.0);
  ay_m_s2 = ay * (9.81/16384.0);
  
  resultantAcceleration = getResultantAcceleration(ax_m_s2, ay_m_s2);
  
  if(movementStarted){
    
     if(distanceToBase(resultantAcceleration) <= shortMargin){
       //se completó el golpe
       Serial.println("Golpe completado");
       
       finalTimestamp = millis();
       duration = finalTimestamp - initialTimestamp;
       
       //enviarlo
       
        Serial.print("Movimiento detectado. Duración (ms):\t");
        Serial.println(duration);
        
        char dataBuffer [sizeof(unsigned long)*8+1];
        sprintf(dataBuffer, "%lu", duration); 
        
        int count = countDigits(duration);

        SerialBT.write((uint8_t*)dataBuffer,sizeof(char)*count);
        SerialBT.write((uint8_t*)";",sizeof(char));
       
        movementStarted = false;
     }
    
  }
  else{
        if(distanceToBase(resultantAcceleration) >= largeMargin){
          //empezó el golpe
          Serial.println("Golpe iniciado");
          
          initialTimestamp = millis();
          movementStarted = true;
          
        }
  } 

  delay(10);
}

float getResultantAcceleration(float x, float y){
  return sqrt(x*x + y*y);
}

float distanceToBase(float resultantAccel){
  float distance = resultantAccel - resultantBaseAcceleration;
  if (distance  < 0)
    return 0 - distance;
  else
    return distance;
}

int countDigits(unsigned long d){
  
      int count = 0;
      
      unsigned long n = d;
      
      while(n != 0) { 
         n /= 10; 
         ++count; 
      }

      return count;
      
}
