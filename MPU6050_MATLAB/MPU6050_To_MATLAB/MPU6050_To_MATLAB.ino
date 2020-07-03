#define G     9.81

#include <Wire.h>
#include <MPU6050.h>

void EulerToQuaternion(double phi, double theta, double psi);

MPU6050 mpu;

// Timers
unsigned long timer = 0;
int timeStep = 0.02;

// Pitch, Roll
double roll = 0, pitch = 0;

double acc_roll = 0, acc_pitch = 0;

double gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
double prev_gyro_roll = 0, prev_gyro_pitch = 0, prev_gyro_yaw = 0;

double cosRoll = 0, cosPitch = 0;
double sinRoll = 0, sinPitch = 0;
double tanRoll = 0, tanPitch = 0;

//Quaternion
double q[4] = {0,};


void setup() 
{
  Serial.begin(115200);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop()
{
  timer = millis();
  
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();
 
  // Output
  Serial.print(normAccel.XAxis);
  Serial.print(", ");
  Serial.print(normAccel.YAxis);
  Serial.print(", ");
  Serial.print(normAccel.ZAxis);
    
  Serial.print(", ");
  Serial.print(normGyro.XAxis);
  Serial.print(", ");
  Serial.print(normGyro.YAxis);
  Serial.print(", ");
  Serial.print(normGyro.ZAxis);

  Serial.println();

  delay(100);
}
