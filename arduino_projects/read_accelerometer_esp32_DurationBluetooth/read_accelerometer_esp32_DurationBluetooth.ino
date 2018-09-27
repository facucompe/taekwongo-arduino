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
uint16_t resultantBaseAcceleration, resultantAcceleration, shortMargin, largeMargin;

//timestamps y flags
unsigned long initialTimestamp, finalTimestamp, duration;
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
  shortMargin = 5.0 / (9.81/16384.0);
  largeMargin = 8.5 / (9.81/16384.0);
  
}

void loop() {

  if(SerialBT.hasClient()){
    
    while(!connectedToApp){
      
      trainingType = SerialBT.read();
  
        if(trainingType != 0){
          connectedToApp = true;
          Serial.print("SETUP Recibido:\t");
          Serial.print(trainingType);
          if(trainingType == 49){
            Serial.println(" (Fuerza)");
          }
          else {
            Serial.println(" (Velocidad)");
          }
    
          //Lee las aceleraciones iniciales.
          sensor.getAcceleration(&ax, &ay, &az);
          resultantBaseAcceleration = getResultantAcceleration(ax, ay);

          break;
      }
  
    }
  
  // Leer las aceleraciones
  sensor.getAcceleration(&ax, &ay, &az);
  
  resultantAcceleration = getResultantAcceleration(ax, ay);
  
  if(movementStarted){
    
     if(distanceToBase(resultantAcceleration) <= shortMargin){
       //se completó el golpe
       Serial.println("Golpe completado");
       
       finalTimestamp = millis();
       duration = finalTimestamp - initialTimestamp;
       
       //enviarlo
       
        Serial.print("Movimiento finalizado. ");
        
          if(trainingType == 49){
            //Fuerza (caracter ASCII de 1)
            Serial.print("Fuerza de impacto (raw):\t");
            Serial.println(resultantAcceleration);

            uint16_t mask   = B11111111;          // 0000 0000 1111 1111
            uint8_t first_half   = resultantAcceleration >> 8;
            uint8_t sencond_half = resultantAcceleration & mask;
            char dataBuffer [sizeof(uint8_t)*8+1];
            
            sprintf(dataBuffer, "%u", first_half);            
            int count = countDigits8(duration);   
            SerialBT.write((uint8_t*)dataBuffer,sizeof(char)*count);

            sprintf(dataBuffer, "%u", sencond_half);            
            count = countDigits8(duration);   
            SerialBT.write((uint8_t*)dataBuffer,sizeof(char)*count);
            
            SerialBT.write((uint8_t*)";",sizeof(char));
            
          }
          else{
            //Velocidad
            Serial.print("Duración (ms):\t");
            Serial.println(duration);
          
          
            char dataBuffer [sizeof(unsigned long)*8+1];
            sprintf(dataBuffer, "%lu", duration); 
            
            int count = countDigits(duration);
    
            SerialBT.write((uint8_t*)dataBuffer,sizeof(char)*count);
            SerialBT.write((uint8_t*)";",sizeof(char));
        }
       
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
  else
  {

    connectedToApp = false;
    delay(1000);
    
  }
}

uint16_t getResultantAcceleration(int16_t x, int16_t y){
  return sqrt(x*x + y*y);
}

uint16_t distanceToBase(uint16_t resultantAccel){
  uint16_t distance = resultantAccel - resultantBaseAcceleration;
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

int countDigits8(uint8_t d){  
      int count = 0;      
      uint8_t n = d;      
      while(n != 0) { 
         n /= 10; 
         ++count; 
      }
      return count;      
}
