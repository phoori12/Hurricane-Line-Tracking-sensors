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
  delay(1000);
  pinMode(PC13, OUTPUT);
  Serial.begin(9600);
  digitalWrite(PC13, HIGH);
  for (int i = 0; i < 400; i++)
  {
    lineTracker.calibrate(); // Initialize CalibratedMinimumOn and CalibratedMaximumOn arrays //
  }
  digitalWrite(PC13, LOW);
  // Read the sensors calibrated minimum values //
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(lineTracker.calibratedMinimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  // Read the sensors calibrated maximum values //
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(lineTracker.calibratedMaximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}

void loop()
{
  unsigned int position = lineTracker.readLine(sensorValues);
  for (unsigned char i = 0; i < 12; i++)
  {
    Serial.print(sensorValues[i]); // Read the sensors calibrated value
    Serial.print('\t');
  }
  Serial.print(position); // Position of the sensor above the line
  Serial.println();
}
