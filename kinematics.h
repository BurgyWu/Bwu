#ifndef _Kinematics
#define _Kinematics_h

//You may want to use these variables
const int WHEEL_DIAMETER = 70;
const int WHEEL_DISTANCE = 140;
const int GEAR_RATIO = 120;
const int COUNTS_PER_SHAFT_REVOLUTION = 12;
const int COUNTS_PER_WHEEL_REVOLUTION =  1440;
const float COUNTS_PER_MM = 6.548;
const float MM_PER_COUNT = 0.1527;

class Kinematics
{
  public:
    //Public variables and methods go here
 Kinematics(float x, float y, float z);
    void setGains(float X, float Y, float Z);
    void reset(); 
    void coordinatereset();
    float update(float e0_count, float e1_count); 
    float returnX();
    float returnY();
    float returnZ();
    void print_components(); 
   
  private:
    //Private variables and methods go here

   //Control gains
    float X=0;
    float Y=0;
    float Z=0;
    float left_countold=0;
    float right_countold=0;
    
};

Kinematics::Kinematics(float x, float y, float z)
{
  X = x;
  Y = y;
  Z = z;
 
}

 float Kinematics::update(float left_countnew, float right_countnew)
 {
  X=X+((left_countnew-left_countold)*MM_PER_COUNT + (right_countnew-right_countold)*MM_PER_COUNT)/2 * cos(Z);
  Y=Y+((left_countnew-left_countold)*MM_PER_COUNT + (right_countnew-right_countold)*MM_PER_COUNT)/2 * sin(Z);
  Z=Z+((left_countnew-left_countold)*MM_PER_COUNT - (right_countnew-right_countold)*MM_PER_COUNT)/WHEEL_DISTANCE;
 
  left_countold=left_countnew;
  right_countold=right_countnew;
 }

float Kinematics::returnX()
{
  return X;
}

float Kinematics::returnY()
{
  return Y;
}

float Kinematics::returnZ()
{
  return Z;
}

 void Kinematics::print_components()
{
  Serial.print("X: ");
  Serial.print(X);
  Serial.print("Y: ");
  Serial.print(Y);
  Serial.print("Theta: ");
  Serial.println(Z);
}
void Kinematics::coordinatereset()
{
  X=0;
  Y=0;
  Z=0;
  left_countold=0;
  right_countold=0;
}


#endif
