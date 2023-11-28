#include <Arduino.h>
#include <wire.h>
#include <iostream>
//min is 0.2 mV, max is 150 mV pre amped, 
//read 5 times to get average before filtering

const int IMUpin = 8;
const int pressPin = 18;
float filtPress = 0.0;
float prevFiltPress = 0.0;
float accel = 0.0;
float pressure = 0.0;
float cutoffFrequency = 1.0;
float delta = 0.2;

float lowFilter(float);    //delta only
float lowFilter2(float);   //with cutofff frequency??? maybe not applicable


void setup() {
  pinMode(IMUpin, INPUT);
  pinMode(pressPin, INPUT);
  Wire.setSCL(19);
  Wire.setSDA(pressPin);
  Serial.begin(9600);

}

void loop() { // grab values then do it with loops, collect 5 times, then average, then filter
 

  for(int i = 0; i < 5, i++;)
  {
    float rawAccel = digitalRead(IMUpin);        //read once
    float rawPressure = analogRead(pressPin);    //read once
    accel+= rawAccel;
    pressure += rawPressure;
  }
  pressure = pressure/5;                         //find average
  accel = accel/5;                               //average

  lowFilter(pressure);        //run analog thru
  Serial.print(accel);        //test output value
  Serial.print(pressure);     //test output value
  
  accel = 0;    //reset
  pressure = 0; //reset

}

// put function definitions here:
float lowFilter(float rawPressure) {         
  filtPress = (delta*rawPressure)+((1.0-delta)*prevFiltPress);
  
  prevFiltPress = filtPress;
}

float lowFilter2(float rawPressure) {    //not done yet
  pow(2.718, (1-exp(-delta*2*cutoffFrequency)));


}
