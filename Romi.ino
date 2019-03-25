#include "encoders.h"
#include "pid.h"
#include "kinematics.h"
#include "line_sensors.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define LOOP_DELAY 10
#define BUZZER_PIN 6
byte l_speed=20;
byte r_speed=20;
float Kp_pose = 0.1; //Proportional gain for position controller
float Kd_pose = -0.05; //Derivative gain for position controller
float Ki_pose = 0; //Integral gain for position controller
float Kp = 0.01; //turn
float Kd = 0; 
float Ki = 0; 

float Kpl=0.5; 
float Kdl=0;
float Kil=0;

float leftSpeed=0;
float rightSpeed=0;
float leftpin=0;
float rightpin=0;
float ds=0;
float dp=0;
float finalSpeedLeft=0;
float finalSpeedRight=0;
float D;
float nxc=0;
float nyc=0;
float nD=0;
float nzc=0;
double hangle=0;
float degree=0;

PID leftPose(Kp_pose, Kd_pose, Ki_pose); //go straight PID
PID rightPose(Kp_pose, Kd_pose, Ki_pose); 

PID leftturn(Kp, Kd, Ki); //follower turn PID
PID rightturn(Kp, Kd, Ki);

PID lturnhome(Kpl, Kdl, Kil); // turn PID
PID rturnhome(Kpl, Kdl, Kil);
byte stage;

enum {go,follower,buzzer,turn,reset,gohome,stopnow};

Kinematics records(0, 0 ,0);

float svleft=0;
float svright=0;
float svcentre=0;
float LineCentre =0;
float Linetotal= 0;


#define LINE_LEFT_PIN A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A4 //Pin for the right line sensor
Line_Sensor lineLeft(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
Line_Sensor lineCentre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
Line_Sensor lineRight(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

#define BAUD_RATE = 115200;

void setupMotorPins()
{
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );
  pinMode( BUZZER_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );
}

// put your setup code here, to run once:
void setup() 
{
 analogWrite(BUZZER_PIN, 120);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
//  //Assign motor pins and set direction
while (millis() < 7000) {
  lineLeft.calibrate();
  lineCentre.calibrate();
  lineRight.calibrate();
}
//  
  analogWrite(BUZZER_PIN, 120);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  delay(3000);
  // These two function set up the pin
  // change interrupts for the encoders.
  setupEncoder0();
  setupEncoder1();
  setupMotorPins();

  // Initialise the Serial communication

  
 stage=go;
  Serial.begin( 9600 );
}


void loop() 
{
   // read the sensor:
  
  svleft = lineLeft.read_calibrated();
  svcentre = lineCentre.read_calibrated();
  svright = lineRight.read_calibrated();
  Linetotal=svleft+svcentre+svright;
  LineCentre =(svleft/Linetotal*1000)+(svcentre/Linetotal*2000)+(svright/Linetotal*3000);
records.update(count_e0, count_e1);
float xc=records.returnX();
float yc=records.returnY();
float zc=records.returnZ();
 D=sqrt(xc*xc+yc*yc);
 Serial.print("zc");
 Serial.print(zc);
 Serial.print("xc");
 Serial.print(xc);
 Serial.print("yc");
 Serial.println(yc);

//*********************movement************************//
switch(stage){
  case go:
  if (Linetotal<800){
   ds = (e0_speed + e1_speed) / 2;
   leftSpeed = leftPose.update(ds, e0_speed);
   rightSpeed = rightPose.update(ds, e1_speed);
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );
  analogWrite( L_PWM_PIN, l_speed+leftSpeed );
  analogWrite( R_PWM_PIN, r_speed+rightSpeed );
  }
  if (Linetotal>800){
  stage++;
  }
  
  break;
  
  case follower:

   dp=(svleft+svright)/2;
    leftpin=leftturn.update(dp, svleft);
    rightpin=rightturn.update(dp, svright);
   
    finalSpeedLeft = l_speed + leftpin;
    finalSpeedRight = r_speed + rightpin;
   
 if ( abs(LineCentre-2000)<100 && Linetotal>800){
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );

 }
 if (LineCentre-2000> 200  ){
   digitalWrite( R_DIR_PIN, HIGH );

 }
 

 if(LineCentre-2000< -200 ){
  digitalWrite( L_DIR_PIN, HIGH  );

 
 }
if(Linetotal<800){
finalSpeedRight=0 ;
finalSpeedLeft=0 ; 

nxc=abs(xc);
nyc=abs(yc);
nzc=zc;
nD=sqrt(nxc*nxc+nyc*nyc);

 stage++;

 }
  analogWrite( L_PWM_PIN, finalSpeedLeft);
  analogWrite( R_PWM_PIN, finalSpeedRight);
break;
  case buzzer:
records.coordinatereset();
count_e0=0;
count_e1=0;
analogWrite(BUZZER_PIN, 120);
delay(500);
digitalWrite(BUZZER_PIN, LOW);
delay(3000);
stage++;
  break; 

  case turn:


  digitalWrite( L_DIR_PIN, HIGH  );
  digitalWrite( R_DIR_PIN, LOW );

hangle=180-(nzc*180/PI-atan(nyc/nxc)*180/PI); //homeangle
if(hangle-(count_e0-count_e1)/16<1)
{
stage++;

 }
  analogWrite( L_PWM_PIN, 20);
  analogWrite( R_PWM_PIN, 20);
  
  break;
  case reset:
 analogWrite( L_PWM_PIN, 0);
 analogWrite( R_PWM_PIN, 0);

  stage++;
  
  break;

  case gohome:

   leftSpeed = lturnhome.update(hangle*0.975, zc*180/PI);
//   rightSpeed = rturnhome.update(hangle, zc*180/3.14);
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );



if( sqrt(xc*xc+yc*yc)>nD*1.015)
{
stage ++;
}
  analogWrite( L_PWM_PIN, 25-leftSpeed );
  analogWrite( R_PWM_PIN, 25+rightSpeed );
  break;

  case stopnow:
  analogWrite( L_PWM_PIN, 0 );
  analogWrite( R_PWM_PIN, 0 );
  break;
}  
}
