// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax_m_s2 , ay_m_s2, az_m_s2, gx_deg_s, gy_deg_, gz_deg_s;
float resultantBaseAcceleration, resultantAcceleration, shortMargin, largeMargin;
unsigned long initialTimestamp, finalTimestamp, duration;
bool movementStarted;

float measurementsX[78] = {-5.13,-5.29,-5.08,-5,-5.21,-5.23,-5.19,-5.29,-5.16,-5.21,-5.11,-5.16,-5.05,-5.2,-5.19,-5.16,-5.24,-5.47,-5.92,-5.59,-5.28,-5.23,-4.98,-5.24,-5.28,-5.43,-5.98,-9.08,-2.4,-0.98,-4.19,-4.68,-4.61,-4.75,-4.62,-4.75,-4.68,-4.63,-4.42,-4.53,-4.5,-4.57,-4.46,-4.13,-4.77,-8.21,43437,-3.27,-5.5,-4.61,-4.55,-4.54,-4.36,-4.46,-4.23,-4.47,-4.22,-4.16,-3.77,-2.75,-2.17,-1.75,-1.44,-0.69,-7.12,6.13,1.39,0.27,-0.69,-0.69,-0.52,-0.23,-0.09,0.02,-2.15,7.66,-1.51,0};;
float measurementsY[78] = {-0.11,-0.07,0.24,-0.29,-0.27,-0.15,-0.01,-0.07,0.04,-0.03,-0.1,-0.05,-0.02,-0.19,-0.15,-0.04,-0.02,0.06,0.01,-0.1,0.48,-0.64,-0.36,-0.42,-0.15,0.03,-0.72,-17.65,15.17,-1.32,0.29,-1.8,-1.18,-1.43,-0.81,-1.13,-0.55,-0.74,-0.63,-0.66,-0.53,-0.53,-0.54,1.61,-4.87,-9.1,17.64,43191,-0.91,-2.22,-0.93,-0.4,0,-0.36,-0.22,0.23,0.27,0.04,-0.2,-0.13,-0.13,0,-0.36,1.58,-19.26,6.61,3.52,2.78,-2.85,-3.56,-1.93,-0.84,0.62,-0.99,-8.57,19.62,-1.94,0};;

int pos, maxPos;

void setup() {
  Serial.begin(57600);    //Iniciando puerto serial  
  
  //Medir valores base iniciales
  pos = 0;
  maxPos = 78;
  
  ax_m_s2 = measurementsX[pos];
  ay_m_s2 = measurementsY[pos];   
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
		return 0 - distance;
	else
		return distance;
}

void loop() {
	
	pos++;
  
	if(pos >= maxPos){
	  pos = 0;
	  Serial.println("Se leyeron todas las mediciones");
	}
	
  // Leer las aceleraciones y velocidades angulares

  ax_m_s2 = measurementsX[pos];
  ay_m_s2 = measurementsY[pos];
  
  resultantAcceleration = getResultantAcceleration(ax_m_s2, ay_m_s2);
  
  if(movementStarted){
	  
	   if(distanceToBase(resultantAcceleration) <= shortMargin){
		   //se completó el golpe
		   
		   finalTimestamp = millis();
		   duration = finalTimestamp - initialTimestamp;
		   
		   //enviarlo
		   
		    Serial.print("Movimiento detectado. Duración:\t");
			Serial.println(duration);
		   
		   movementStarted = false;
	   }
	  
  }
  else{
  if(distanceToBase(resultantAcceleration) >= largeMargin){
	  //empezó el golpe
	  
	  initialTimestamp = millis();
	  movementStarted = true;
	  
	}
  } 

  delay(10);
}
 
