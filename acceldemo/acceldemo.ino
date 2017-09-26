// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

double POWER_MAX = 20.0;
double POWER_MIN = 1.0;

double power = POWER_MAX;
double last_t = millis() / 1000.0;

// How many seconds it takes for power level to halve
double HALF_LIFE = 3.0;

int NUM_LEDS = 6;
int LED_PINS[] = {2,3,4,5,6,7}; // ordered from low to high

int EL_PIN = 9;

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // Setup LED pins and turn them on
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], HIGH);
  }
}

void loop() {
  lis.read();      // get X Y and Z data at once

  sensors_event_t event;
  lis.getEvent(&event);

  // Compute time
  double time_secs = millis() / 1000.0;
  double delta_t = time_secs - last_t;
  last_t = time_secs;

  // Get x,y,z components of acceleration vector
  double x = event.acceleration.x;
  double y = event.acceleration.y;
  double z = event.acceleration.z - 9.8; // Negate gravity

  // Find the acceleration vector magnitude
  double a = sqrt(x*x + y*y + z*z);

  // Assume power is acceleration over time
  double p = a * delta_t;

  // Decay power using half-life forumla ( power * e^(-t/h) )
  power = power * pow(M_E, -delta_t/HALF_LIFE);

  // Add last power to total
  power = power + p;

  // Get a percentage of power from the min to the max
  double power_per = (power - POWER_MIN) / (POWER_MAX - POWER_MIN);

  // Constrain to [0,1]
  power_per = constrain(power_per, 0.0, 1.0);

  // How many LEDs to light according to power percentage
  int num = power_per * double(NUM_LEDS);

  // Turn on num leds
  for (int i = 0; i < num; i++) {
    digitalWrite(LED_PINS[i], HIGH);
  }

  // Turn off the rest leds
  for (int i = NUM_LEDS - 1; i >= num; i--) {
    digitalWrite(LED_PINS[i], LOW);
  }

  // Pulse EL panel
  analogWrite(EL_PIN, (int)(power_per * 255.0));

  // Debug output
  // Serial.print("X:  "); Serial.print(lis.x);
  // Serial.print("  \tY:  "); Serial.print(lis.y);
  // Serial.print("  \tZ:  "); Serial.print(lis.z);
  // Serial.print("\t\tX: "); Serial.print(x);
  // Serial.print(" \tY: "); Serial.print(y);
  // Serial.print(" \tZ: "); Serial.print(z);
  // Serial.print("  m/s^2 ");
  // Serial.print(" \tt: "); Serial.print(delta_t);
  // Serial.print(" \tf: "); Serial.print(f);
  // Serial.print(" \tdp: "); Serial.print(p);
  // Serial.print(" \tp: "); Serial.print(power);
  // Serial.print(" \tp%: "); Serial.print(power_per * 100.0);
  // Serial.println();
}
