
/******************************************************************************
ISL29125_basics.ino
Simple example for using the ISL29125 RGB sensor library.
Jordan McConnell @ SparkFun Electronics
11 Apr 2014
https://github.com/sparkfun/ISL29125_Breakout

This example declares an SFE_ISL29125 object called RGB_sensor. The 
object/sensor is initialized with a basic configuration so that it continuously
samples the light intensity of red, green and blue spectrums. These values are
read from the sensor every 2 seconds and printed to the Serial monitor.

Developed/Tested with:
Arduino Uno
Arduino IDE 1.0.5

Requires:
SFE_ISL29125_Library

This code is beerware.
Distributed as-is; no warranty is given. 
******************************************************************************/

#include <Wire.h>
#include "SFE_ISL29125.h"
#include <Adafruit_NeoPixel.h>

#define LEDSTRIP 8
unsigned long time_now = 0;

// Declare sensor object
SFE_ISL29125 RGB_sensor;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, LEDSTRIP, NEO_GRB + NEO_KHZ800);

void turnOn() {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));
    strip.show();
    //delay(10);
  }
}

void turnOff() {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
    strip.show();
    //delay(10);
  }
}

void setup()
{
  // Initialize serial communication
  unsigned int red = 0;
  unsigned int green = 0;
  unsigned int blue = 0;
  
  Serial.begin(115200);

  // Initialize the ISL29125 with simple configuration so it starts sampling
  if (RGB_sensor.init())
  {
    Serial.println("Sensor Initialization Successful\n\r");
  }
  DDRB |= 0b00000001;

  strip.begin();
  strip.show();
}

void colour_sense()
{
  turnOn();
  time_now = millis();   
  while(millis() < time_now + 100);

  red = 0;
  green = 0;
  blue = 0;

  for (int i = 0; i < 10; i ++)
  {
    red += RGB_sensor.readRed();
    green += RGB_sensor.readGreen();
    blue += RGB_sensor.readBlue();
  }
  red /= 10;
  green/=10;
  blue /= 10;
  
  // Print out readings, change HEX to DEC if you prefer decimal output
  Serial.print("Red: "); Serial.println(red,DEC);
  Serial.print("Green: "); Serial.println(green,DEC);
  Serial.print("Blue: "); Serial.println(blue,DEC);
  Serial.println();

  //send back to pi to print
  if (red > green)
  {
    Serial.println("red");
    
  }
  else
  {
    Serial.println("green");
  }
}
// Read sensor values for each color and print them to serial monitor
void loop()
{
  // Read sensor values (16 bit integers)
  
  //turnOff();
  //delay(1000);
}
