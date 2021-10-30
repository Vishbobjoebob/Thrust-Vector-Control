#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <SFE_BMP180.h>
#include <Servo.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define ALTITUDE 35.6 //Altitude of Location Relative to Sea Level
//dt = 0.10
  
Adafruit_BNO055 bno = Adafruit_BNO055();
SFE_BMP180 pressure;
Servo servoX ;
Servo servoY ;

int counter = 0;
int xTiltAcc;
int yTiltAcc;
float xTiltGy;
float yTiltGy;
float xTiltGyOld;
float yTiltGyOld;
int xTiltAccOld;
int yTiltAccOld;

float filterXTilt ;
float filterYTilt ;

float ServoXPos;
float ServoYPos;

float dt;
unsigned long millisOld;

//Servo parameters
int x_min = 40  ;  
int x_max = 135 ; 
int y_min = 7 ;
int y_max = 90 ;
double x_rest = 93 ;
double y_rest = 50 ;

/*double pitchPID(double target) {
  iPitchPID.kP = 0;
  iPitchPID.kD = 0;
  iPitchPID.kI = 0;

  //iPitchPID.current = get_average_inertial(); //BNO Filter
  iPitchPID.error = target - iPitchPID.current;
  iPitchPID.integral += iPitchPID.error;
  iPitchPID.derivative = iPitchPID.error - iPitchPID.lastError;
  iPitchPID.lastError = iPitchPID.error;

  return (((iPitchPID.error) * (iPitchPID.kP)) + ((iPitchPID.derivative) * (iPitchPID.kD)) + 
    ((iPitchPID.integral) * (iPitchPID.kI)));
}
*/
float complementaryFilter(float accelReading, float gyroReading) ;

float complementaryFilter(float accelReading, float gyroReading) {
  double alpha = 0.9 ;
  return (1-alpha)*(gyroReading) + alpha*accelReading ;
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Welcome to the Flight Computer setup!"); 
  delay(100) ;
  Serial.println("Let's start with the BNO055 IMU Calibration");
  delay(1000) ;
  
  sensors_event_t event; 
  bno.getEvent(&event);
  
  //Sensor initilization
  if(bno.begin())
  {
    Serial.println("BNO055 Initialized");
  }
  else {
    Serial.println("No BNO055. Try again!");
    while(1) ;
  }

  // Calibration
  
  Serial.println("Calibrating Inertials...");
  uint8_t system, gyro, accel, mg = 0;
  delay(1000) ;

  while(1) {
    bno.getCalibration(&system, &gyro, &accel, &mg) ;
    Serial.print("System: ");
    Serial.print(system);
    Serial.print(", Gyro: ");
    Serial.print(gyro);
    Serial.print(", Accel: ");
    Serial.print(accel);
    Serial.print(", Mg: ");
    Serial.println(mg);
    if (system == 3 && gyro == 3 && accel == 3 && mg == 3 ) {
      Serial.println("Inertials Calibrated");
      break ;
    }
    if (counter > 120) {
      Serial.println("Inertial Calibration Failed... Try again");
      while(1);
    }
    delay(1000);
    counter += 1;
  }
  delay(1000) ;

  //BMP Initilization
  int8_t temp=bno.getTemp();
  
  if (pressure.begin()){ //If initialization was successful, continue
    Serial.println("BMP180 Initilized");
  }
  else{
    Serial.println("No BMP. Try again!");
    while(1) ;  
  }


  //Resetting and Calibrating TVC
  Serial.println("Calibrating TVC") ;

  servoX.attach(4) ;
  servoY.attach(6) ;

  delay(1000) ;

  servoX.write(x_rest) ;
  servoY.write(y_rest) ;

  delay(2000) ;
  
  servoX.write(x_max) ;
  delay(100);
  servoX.write(x_rest) ;

  delay(2000) ;

  servoX.write(x_min) ;
  delay(1000);
  servoX.write(x_rest) ;

  delay(2000) ;

  servoY.write(y_min) ;
  delay(1000);
  servoY.write(y_rest) ;

  delay(2000) ;

  servoY.write(y_max) ;
  delay(1000);
  servoY.write(y_rest) ;

  delay(2000) ;

  servoX.write(x_min) ;
  servoY.write(y_min) ;
  delay(1000);
  servoX.write(x_rest) ;
  servoY.write(y_rest) ;

  delay(2000) ;

  servoX.write(x_min) ;
  servoY.write(y_max) ;
  delay(1000);
  servoX.write(x_rest) ;
  servoY.write(y_rest) ;

  delay(2000) ;

  servoX.write(x_max) ;
  servoY.write(y_min) ;
  delay(1000);
  servoX.write(x_rest) ;
  servoY.write(y_rest) ;

  delay(2000) ;

  servoX.write(x_max) ;
  servoY.write(y_max) ;
  delay(1000);
  servoX.write(x_rest) ;
  servoY.write(y_rest) ;
  
  delay(2000) ;

  delay(10000) ;
  Serial.println("Setup Complete. Moving to ground state...");
  
  bno.setExtCrystalUse(true);
  millisOld=millis();

}
 
void loop() 
  /* Raw Sensor Readings */ {
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE) ;

  xTiltAcc= -atan2(acc.y()/9.807,acc.x()/9.807)/2/3.141592654*360 ;
  yTiltAcc= atan2(acc.z()/9.807,acc.x()/9.807)/2/3.141592654*360 ;

  //Reading Post-processing
  if (xTiltAcc > 0) {
    xTiltAcc = xTiltAcc - 180 ;
  }
  else if (xTiltAcc < 0) {
    xTiltAcc = xTiltAcc + 180 ;
  }

  if (yTiltAcc > 0) {
    yTiltAcc = yTiltAcc - 180 ;
  }
  else if (yTiltAcc < 0) {
    yTiltAcc = yTiltAcc + 180 ;
  }

  if (xTiltAcc > 15) {
      xTiltAcc = 15;
  }

  else if (xTiltAcc < -15) {
      xTiltAcc = -15;
  }
  if (yTiltAcc > 15) {
      yTiltAcc = 15;
    }
  else if (yTiltAcc < -15) {
      yTiltAcc = -15;
    }

  
  xTiltGy = xTiltGyOld + gyr.y() * dt;
  yTiltGy = yTiltGyOld + gyr.z() * dt;
  
  filterXTilt = complementaryFilter(xTiltAcc, xTiltGy) ;
  filterYTilt = complementaryFilter(yTiltAcc, yTiltGy) ;

  ServoXPos = map(xTiltAcc, -15.00, 15.00, x_min, x_max);
  ServoYPos = map(yTiltAcc, -15.00, 15.00, y_min, y_max);


  servoX.write(ServoXPos);
  servoY.write(ServoYPos);
  
  Serial.print(xTiltGy) ;
  Serial.print(", ");
  Serial.print(yTiltGy) ;
  Serial.print(", ");
  Serial.print(xTiltAcc) ;
  Serial.print(", ");
  Serial.print(yTiltAcc) ;
  Serial.print(", ");
  Serial.print(filterXTilt) ;
  Serial.print(", ");
  Serial.println(filterYTilt) ;
  Serial.print(", ");
  Serial.print(ServoXPos) ;
  Serial.print(", ");
  Serial.println(ServoYPos) ;

  
  //Serial.print(xTiltGy);
  //Serial.print(",");
  //Serial.println(yTiltGy);
  
  //Serial.print(filterXTilt);
 // Serial.println(filterYTilt);
 
dt=(millis()-millisOld)/1000.00;
millisOld=millis(); 

xTiltGyOld = xTiltGy ;
yTiltGyOld = yTiltGy ;
xTiltAccOld = xTiltAcc;
yTiltAccOld = yTiltAcc;

delay(BNO055_SAMPLERATE_DELAY_MS);  
  
}
