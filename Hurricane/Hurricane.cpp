#include <stdlib.h>
#include "Hurricane.h"
#include <Arduino.h>


// Base class constructor
HurricaneSens::HurricaneSens() : calibratedMinimum(nullptr),
                                       calibratedMaximum(nullptr),
                                       _pins(nullptr)
{
    // empty
}


// Base class data member initialization (called by derived class init())
void HurricaneSens::init(unsigned char *pins)
{
    calibratedMinimum = nullptr;
    calibratedMaximum = nullptr;

    _lastValue = 0; // assume initially that the line is left.
    unsigned char numSensors = 12;
    if (numSensors > HRC_MAX_SENSORS)
        _numSensors = HRC_MAX_SENSORS;
    else
        _numSensors = numSensors;

    if (_pins == nullptr)
    {
        _pins = (unsigned char*)malloc(sizeof(unsigned char)*_numSensors);
        if (_pins == nullptr)
            return;
    }

    unsigned char i;
    for (i = 0; i < _numSensors; i++)
    {
        _pins[i] = pins[i];
    }

}


// Reads the sensor values into an array. There *MUST* be space
// for as many values as there were sensors specified in the constructor.
// Example usage:
// unsigned int sensor_values[8];
// sensors.read(sensor_values);
// The values returned are a measure of the reflectance in abstract units,
// with higher values corresponding to lower reflectance (e.g. a black
// surface or a void).
void HurricaneSens::read(unsigned int *sensor_values)
{
    readPrivate(sensor_values);
}



// Resets the calibration.
void HurricaneSens::resetCalibration()
{
    unsigned char i;
    for(i=0;i<_numSensors;i++)
    {
        if(calibratedMinimum)
            calibratedMinimum[i] = _maxValue;
        if(calibratedMaximum)
            calibratedMaximum[i] = 0;
    }
}

// Reads the sensors 10 times and uses the results for
// calibration.  The sensor values are not returned; instead, the
// maximum and minimum values found over time are stored internally
// and used for the readCalibrated() method.
void HurricaneSens::calibrate()
{
    calibrateOnOrOff(&calibratedMinimum,
                        &calibratedMaximum);

}

void HurricaneSens::calibrateOnOrOff(unsigned int **calibratedMinimum,
                                     unsigned int **calibratedMaximum)
{
    int i;
    unsigned int sensor_values[16];
    unsigned int max_sensor_values[16];
    unsigned int min_sensor_values[16];

    // Allocate the arrays if necessary.
    if(*calibratedMaximum == 0)
    {
        *calibratedMaximum = (unsigned int*)malloc(sizeof(unsigned int)*_numSensors);

        // If the malloc failed, don't continue.
        if(*calibratedMaximum == 0)
            return;

        // Initialize the max and min calibrated values to values that
        // will cause the first reading to update them.

        for(i=0;i<_numSensors;i++)
            (*calibratedMaximum)[i] = 0;
    }
    if(*calibratedMinimum == 0)
    {
        *calibratedMinimum = (unsigned int*)malloc(sizeof(unsigned int)*_numSensors);

        // If the malloc failed, don't continue.
        if(*calibratedMinimum == 0)
            return;

        for(i=0;i<_numSensors;i++)
            (*calibratedMinimum)[i] = _maxValue;
    }

    int j;
    for(j=0;j<10;j++)
    {
        read(sensor_values);
        for(i=0;i<_numSensors;i++)
        {
            // set the max we found THIS time
            if(j == 0 || max_sensor_values[i] < sensor_values[i])
                max_sensor_values[i] = sensor_values[i];

            // set the min we found THIS time
            if(j == 0 || min_sensor_values[i] > sensor_values[i])
                min_sensor_values[i] = sensor_values[i];
        }
    }

    // record the min and max calibration values
    for(i=0;i<_numSensors;i++)
    {
        if(min_sensor_values[i] > (*calibratedMaximum)[i])
            (*calibratedMaximum)[i] = min_sensor_values[i];
        if(max_sensor_values[i] < (*calibratedMinimum)[i])
            (*calibratedMinimum)[i] = max_sensor_values[i];
    }
}


// Returns values calibrated to a value between 0 and 1000, where
// 0 corresponds to the minimum value read by calibrate() and 1000
// corresponds to the maximum value.  Calibration values are
// stored separately for each sensor, so that differences in the
// sensors are accounted for automatically.
void HurricaneSens::readCalibrated(unsigned int *sensor_values)
{
    int i;

    // if not calibrated, do nothing
    if(!calibratedMinimum || !calibratedMaximum)
        return;

    // read the needed values
    read(sensor_values);

    for(i=0;i<_numSensors;i++)
    {
        unsigned int calmin,calmax;
        unsigned int denominator;

        // find the correct calibration
        calmax = calibratedMaximum[i];
        calmin = calibratedMinimum[i];

        denominator = calmax - calmin;

        signed int x = 0;
        if(denominator != 0)
            x = (((signed long)sensor_values[i]) - calmin)
                * 1000 / denominator;
        if(x < 0)
            x = 0;
        else if(x > 1000)
            x = 1000;
        if (x < 800) x = 0;
        sensor_values[i] = x;
    }

}


// Operates the same as read calibrated, but also returns an
// estimated position of the robot with respect to a line. The
// estimate is made using a weighted average of the sensor indices
// multiplied by 1000, so that a return value of 0 indicates that
// the line is directly below sensor 0, a return value of 1000
// indicates that the line is directly below sensor 1, 2000
// indicates that it's below sensor 2000, etc.  Intermediate
// values indicate that the line is between two sensors.  The
// formula is:
//
//    0*value0 + 1000*value1 + 2000*value2 + ...
//   --------------------------------------------
//         value0  +  value1  +  value2 + ...
//
// By default, this function assumes a dark line (high values)
// surrounded by white (low values).  If your line is light on
// black, set the optional second argument white_line to true.  In
// this case, each sensor value will be replaced by (1000-value)
// before the averaging.
int HurricaneSens::readLine(unsigned int *sensor_values, unsigned char white_line)
{
    unsigned char i, on_line = 0;
    unsigned long avg; // this is for the weighted total, which is long
                       // before division
    unsigned int sum; // this is for the denominator which is <= 64000

    readCalibrated(sensor_values);

    avg = 0;
    sum = 0;

    for(i=0;i<_numSensors;i++) {
        int value = sensor_values[i];
        if(white_line)
            value = 1000-value;

        // keep track of whether we see the line at all
        if(value > 200) {
            on_line = 1;
        }

        // only average in values that are above a noise threshold
        if(value > 50) {
            avg += (long)(value) * (i * 1000);
            sum += value;
        }
    }

    if(!on_line)
    {
        // If it last read to the left of center, return 0.
        if(_lastValue < (_numSensors-1)*1000/2)
            return 0;

        // If it last read to the right of center, return the max.
        else
            return (_numSensors-1)*1000;

    }

    _lastValue = avg/sum;

    return _lastValue;
}




// Derived RC class constructor
Hurricane::Hurricane(unsigned char* pins, unsigned int timeout)
{
    init(pins, timeout);
}


// The array 'pins' contains the Arduino pin number for each sensor.

// 'numSensors' specifies the length of the 'pins' array (i.e. the
// number of QTR-RC sensors you are using).  numSensors must be
// no greater than 16.

// 'timeout' specifies the length of time in microseconds beyond
// which you consider the sensor reading completely black.  That is to say,
// if the pulse length for a pin exceeds 'timeout', pulse timing will stop
// and the reading for that pin will be considered full black.
// It is recommended that you set timeout to be between 1000 and
// 3000 us, depending on things like the height of your sensors and
// ambient lighting.  Using timeout allows you to shorten the
// duration of a sensor-reading cycle while still maintaining
// useful analog measurements of reflectance

// 'emitterPin' is the Arduino pin that controls the IR LEDs on the 8RC
// modules.  If you are using a 1RC (i.e. if there is no emitter pin),
// or if you just want the emitters on all the time and don't want to
// use an I/O pin to control it, use a value of 255 (QTR_NO_EMITTER_PIN).
void Hurricane::init(unsigned char* pins, unsigned int timeout)
{
    Hurricane::init(pins);

    _maxValue = timeout;
}


// Reads the sensor values into an array. There *MUST* be space
// for as many values as there were sensors specified in the constructor.
// Example usage:
// unsigned int sensor_values[8];
// sensors.read(sensor_values);
// ...
// The values returned are in microseconds and range from 0 to
// timeout (as specified in the constructor).
// A 'step' of n means that the first of every n sensors is read, starting
// with 'start' (this is 0-indexed, so start = 0 means start with the first
// sensor). For example, step = 2, start = 1 means read the *even-numbered*
// sensors.
void Hurricane::readPrivate(unsigned int *sensor_values, unsigned char step, unsigned char start)
{
    unsigned char i;

    if (_pins == 0)
        return;

    for(i = start; i < _numSensors; i += step)
    {
        sensor_values[i] = _maxValue;
        pinMode(_pins[i], OUTPUT);      // make sensor line an output (drives low briefly, but doesn't matter)
        digitalWrite(_pins[i], HIGH);   // drive sensor line high
    }

    delayMicroseconds(10);              // charge lines for 10 us

    for(i = start; i < _numSensors; i += step)
    {
        pinMode(_pins[i], INPUT);       // make sensor line an input
        digitalWrite(_pins[i], LOW);    // important: disable internal pull-up!
    }

    unsigned long startTime = micros();
    while (micros() - startTime < _maxValue)
    {
        unsigned int time = micros() - startTime;
        for (i = start; i < _numSensors; i += step)
        {
            if (digitalRead(_pins[i]) == LOW && time < sensor_values[i])
                sensor_values[i] = time;
        }
    }
}


// the destructor frees up allocated memory
HurricaneSens::~HurricaneSens()
{
    if (_pins)
        free(_pins);
    if(calibratedMaximum)
        free(calibratedMaximum);
    if(calibratedMinimum)
        free(calibratedMinimum);
}
