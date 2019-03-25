#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = 500;

/* 
 *  Class to represent a single line sensor
 */
class Line_Sensor
{
  public:
    //Constructor
    Line_Sensor(int Line_pin);
    //Calibrate
    void calibrate();
    //Return the uncalibrated value from the sensor
    float read_raw();
    //Return the calibrated value from the sensor
    float read_calibrated();
    
  private:
  
    int pin;
    float sensorValue = 0;
    float sensorMin = 1023;        // minimum sensor value
    float sensorMax = 0;           // maximum sensor value
    
    
};

Line_Sensor::Line_Sensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

float Line_Sensor::read_raw()
{
  return analogRead(pin);
}

void Line_Sensor::calibrate()
{
  
    sensorValue = analogRead(pin);

    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  
}

float Line_Sensor::read_calibrated()
{
  /*
   * Write code to return a calibrated reading here
   */
   sensorValue = analogRead(pin);
   return  (sensorValue - sensorMin) * 1023 / (sensorMax - sensorMin);
//   return  sensorValue;
}


#endif
