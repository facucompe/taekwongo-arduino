// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <chrono>


using namespace std::chrono;


// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax_m_s2 , ay_m_s2, az_m_s2, gx_deg_s, gy_deg_, gz_deg_s;
float resultantBaseAcceleration, resultantAcceleration, shortMargin, largeMargin;
milliseconds initialTimestamp;
bool movementStarted;

void setup() {
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection())Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  //Medir valores base iniciales
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  ax_m_s2 = ax * (9.81/16384.0);
  ay_m_s2 = ay * (9.81/16384.0);  
  resultantBaseAcceleration = getResultantAcceleration(ax_m_s2, ay_m_s2);
  
  //Umbrales y flags
  movementStarted = false;
  shortMargin = 5.0;
  largeMargin = 8.5;
  
}

float getResultantAcceleration(float x, float y){
	return sqrt(x*x + y*y);
}

float distanceToBase(float resultantAccel){
	float distance = resultantAccel - resultantBaseAcceleration;
	if (distance  < 0)
		return 0-distance;
	else
		return distance;
}

void loop() {
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  ax_m_s2 = ax * (9.81/16384.0);
  ay_m_s2 = ay * (9.81/16384.0);
  az_m_s2 = az * (9.81/16384.0);
  gx_deg_s = gx * (250.0/32768.0);
  gy_deg_s = gy * (250.0/32768.0);
  gz_deg_s = gz * (250.0/32768.0);
  
  resultantAcceleration = getResultantAcceleration(ax_m_s2, ay_m_s2);
  
  if(movementStarted){
	  
	   if(distanceToBase(resultantAcceleration) <= shortMargin){
		   //se completó el golpe
		   
		   milliseconds finalTimestamp = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		   milliseconds duration = finalTimestamp - initialTimestamp;
		   
		   //enviarlo
		   
		    Serial.print("Movimiento detectado. Duración:\t");
			Serial.println(duration);
		   
		   movementStarted = false;
	   }
	  
  }
  else{
  if(distanceToBase(resultantAcceleration) >= largeMargin){
	  //empezó el golpe
	  
	  initialTimestamp =  duration_cast< milliseconds >(system_clock::now().time_since_epoch());
	  movementStarted = true;
	  
	}
  }
  

  delay(10);
}
 
