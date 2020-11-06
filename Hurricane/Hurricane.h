/*
  QTRSensors - library for using Pololu QTR reflectance sensors and reflectance
    sensor arrays.  The object used is determined by the type of the sensor
    (analog or RC; dimmable or non-dimmable).  Then simply specify in the
    constructor which Arduino I/O pins are connected to a QTR sensor, and the
    read() method will obtain reflectance measurements for those sensors.
    Smaller sensor values correspond to higher reflectance (e.g.  white) while
    larger sensor values correspond to lower reflectance (e.g.  black or a
    void).

    * QTRSensorsRC should be used for original (non-dimmable) QTR-xRC sensors.
    * QTRSensorsAnalog should be used for original (non-dimmable) QTR-xA sensors.
    * QTRDimmableRC should be used for dimmable QTR-xD-xRC and QTRX-xD-xRC sensors.
    * QTRDimmableAnalog should be used for dimmable QTR-xD-xA and QTRX-xD-xA sensors.
*/

#ifndef Hurricane_h
#define Hurricane_h

#define HRC_MAX_SENSORS 16

// Base class: this class cannot be instantiated directly.
// Instead, you should instantiate one of its derived classes.
class HurricaneSens
{
  public:

    // Reads the sensor values into an array. There *MUST* be space
    // for as many values as there were sensors specified in the constructor.
    // Example usage:
    // unsigned int sensor_values[8];
    // sensors.read(sensor_values);
    // The values returned are a measure of the reflectance in abstract units,
    // with higher values corresponding to lower reflectance (e.g. a black
    // surface or a void).
    // If measureOffAndOn is true, measures the values with the
    // emitters on AND off and returns on - (timeout - off).  If this
    // value is less than zero, it returns zero.
    // This method will call the appropriate derived class's readPrivate(),
    // which is defined as a virtual function in the base class and
    // overridden by each derived class's own implementation.
    virtual void read(unsigned int *sensor_values);

    // Reads the sensors for calibration.  The sensor values are
    // not returned; instead, the maximum and minimum values found
    // over time are stored internally and used for the
    // readCalibrated() method.
    void calibrate();

    // Resets all calibration that has been done.
    void resetCalibration();

    // Returns values calibrated to a value between 0 and 1000, where
    // 0 corresponds to the minimum value read by calibrate() and 1000
    // corresponds to the maximum value.  Calibration values are
    // stored separately for each sensor, so that differences in the
    // sensors are accounted for automatically.
    void readCalibrated(unsigned int *sensor_values);

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
    int readLine(unsigned int *sensor_values, unsigned char white_line = 0);

    // Calibrated minumum and maximum values. These start at 1000 and
    // 0, respectively, so that the very first sensor reading will
    // update both of them.
    //
    // The pointers are unallocated until calibrate() is called, and
    // then allocated to exactly the size required.  Depending on the
    // readMode argument to calibrate, only the On or Off values may
    // be allocated, as required.
    //
    // These variables are made public so that you can use them for
    // your own calculations and do things like saving the values to
    // EEPROM, performing sanity checking, etc.
    unsigned int *calibratedMinimum;
    unsigned int *calibratedMaximum;

    ~HurricaneSens();

  protected:
    HurricaneSens();

    virtual void init(unsigned char *pins);

    virtual void readPrivate(unsigned int *sensor_values, unsigned char step = 1, unsigned char start = 0) = 0;

    unsigned char *_pins;
    unsigned char _numSensors = 12;
    unsigned int _maxValue; // the maximum value returned by readPrivate()

  private:

    // Handles the actual calibration. calibratedMinimum and
    // calibratedMaximum are pointers to the requested calibration
    // arrays, which will be allocated if necessary.
    void calibrateOnOrOff(unsigned int **calibratedMinimum,
                          unsigned int **calibratedMaximum);

    int _lastValue;
};


// Object to be used for original (non-dimmable) QTR-xRC sensors
class Hurricane : virtual public HurricaneSens
{
  public:

    // if this constructor is used, the user must call init() before using
    // the methods in this class
    Hurricane() {}

    // this constructor just calls init()
    Hurricane(unsigned char *pins,
              unsigned int timeout = 2500);

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
    virtual void init(unsigned char* pins,
          unsigned int timeout = 2500);

  private:

    // Reads the sensor values into an array. There *MUST* be space
    // for as many values as there were sensors specified in the constructor.
    // Example usage:
    // unsigned int sensor_values[8];
    // sensors.read(sensor_values);
    // The values returned are a measure of the reflectance in microseconds.
    // A 'step' of n means that the first of every n sensors is read, starting
    // with 'start' (this is 0-indexed, so start = 0 means start with the first
    // sensor). For example, step = 2, start = 1 means read the *even-numbered*
    // sensors.
    void readPrivate(unsigned int *sensor_values, unsigned char step = 1, unsigned char start = 0) override;
};



#endif
