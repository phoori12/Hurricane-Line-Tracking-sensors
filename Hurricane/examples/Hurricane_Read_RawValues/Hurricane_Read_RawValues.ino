#include <Hurricane.h>

#define TIMEOUT 2500
#define NUM_SENSORS 12
Hurricane lineTracker((unsigned char[]) {
  PB9, PB8, PB7, PB6, PA10, PA9, PA8, PB10, PB15, PB14, PB13, PB12
},
TIMEOUT);
unsigned int sensorValues[12];
void setup()

{
  delay(500);
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(1000);
}

void loop()
{
  // read raw sensor values
  lineTracker.read(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(250);
}
