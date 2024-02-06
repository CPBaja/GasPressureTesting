#include <Arduino.h> 
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include <wire.h>
#include <iostream>
#include <SPI.h>
#include <SD.h>
//min is 0.2 mV, max is 150 mV pre amped, 
//read 5 times to get average before reading

const int IMUpin = 8;      //input IMU1
const int pressPin = 18;   //input pressure pin
const int output = 6;      //output pin 6 digital
float filtPress = 0.0;     //filtered pressure
float prevFiltPress = 0.0; //previous filter pressure
Adafruit_LSM6DS accelerometer;
Adafruit_Sensor *lsm_accel;
float accelx = 0.0;        
float accely = 0.0;
float accelz = 0.0;
float pressure = 0.0;
float cutoffFrequency = 1.0;  //maybe not required for acceleration, maybe for pressure
float delta = 0.2;          //test value for filtering

float lowFilter(float);    //delta only
float lowFilter2(float);   //with cutofff frequency??? maybe only applicable for 


void setup() {
  Serial.begin(9600); 
  pinMode(IMUpin, INPUT);  //
  pinMode(pressPin, INPUT);//input from pressure
  Wire.setSCL(19);         //clock at pin 19
  Wire.setSDA(pressPin);   //analog input from presure sensor
  if (!accelerometer.begin_I2C()) {
    Serial.println("CAN'T FIND THE LSM!!!!!");
    while (1) {
      delay(10);
    }
  }
  lsm_accel = accelerometer.getAccelerometerSensor();
  lsm_accel->printSensorDetails();
  //SD.begin; 
}

void loop() { // collect 5 times, then average, then filter
  sensors_event_t accel;
  lsm_accel->getEvent(&accel);

  for(int i = 1; i < 5, i++;)
  {
    float rawPressure = analogRead(pressPin);    //read once
    accelx += accel.acceleration.x;
    accely += accel.acceleration.y;
    accelz += accel.acceleration.y;
    pressure += rawPressure;
  }
  pressure = pressure/5;                         //find average
  accelx = accelx/5;                               //average

  pressure = lowFilter(pressure);    //run analog thru
  Serial.print(accelx);       //test output value
  Serial.print(accely);       //test output value
  Serial.print(accelz);       //test output value

  //there are going to be 2 more accel values for y and z
  Serial.print(pressure);     //test output value
  
  accelx = 0;    //reset
  accely = 0;    //reset
  accelz = 0;    //reset
  pressure = 0; //reset

}

// put function definitions here:
float lowFilter(float rawPressure) {         
  filtPress = (delta*rawPressure)+((1.0-delta)*prevFiltPress);
  
  prevFiltPress = filtPress;
  return filtPress;
}

float lowFilter2(float rawPressure) {    //not done yet, might not even be used
  return pow(2.718, (1-exp(-delta*2*cutoffFrequency)));


}
