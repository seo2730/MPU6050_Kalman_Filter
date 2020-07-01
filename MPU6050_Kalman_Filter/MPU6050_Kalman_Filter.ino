#define G     9.81

#include <Wire.h>
#include <MPU6050.h>

void EulerToQuaternion(float phi, float theta, float psi);
void EulerKalman(float *A, float *z);

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll
float roll = 0, pitch = 0;

float acc_roll = 0, acc_pitch = 0;

float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
float prev_gyro_roll = 0, prev_gyro_pitch = 0, prev_gyro_yaw = 0;

float cosRoll = 0, cosPitch = 0;
float sinRoll = 0, sinPitch = 0;
float tanRoll = 0, tanPitch = 0;

//Quaternion
float q[4] = {0,};

// 시스템 모델 변수
float A[4][4] = {0,};

float H[4][4] = {{1,0,0,0},
                 {0,1,0,0},
                 {0,0,1,0},
                 {0,0,0,1}};

// 잡음 공분산
float Q[4][4] = {{0.0001,0,0,0},
                 {0,0.0001,0,0},
                 {0,0,0.0001,0},
                 {0,0,0,0.0001}};

float R[4][4] = {{10,0,0,0},
                 {0,10,0,0},
                 {0,0,10,0},
                 {0,0,0,10}};

//Kalman 변수
float x[4][1] = {{1,0,0,0}};
float xp[4][4] = {0,};

float P[4][4] = {{1,0,0,0},
                 {0,1,0,0},
                 {0,0,1,0},
                 {0,0,0,1}};
                
float Pp[4][4] = {0,};
float K[4][4] = {0,};


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

  cosRoll = cos(prev_gyro_roll); 
  sinRoll = sin(prev_gyro_roll); 
  tanRoll = tan(prev_gyro_roll);
  
  cosPitch = cos(prev_gyro_pitch);
  sinPitch = sin(prev_gyro_pitch);
  tanPitch = tan(prev_gyro_pitch);
  
  // Calculate Pitch & Roll by Accel
  acc_roll = asin(normAccel.XAxis/G)*180.0/M_PI;
  acc_pitch = asin(-normAccel.YAxis/(G*cos(roll)))*180.0/M_PI;

  // Calculate Pitch & Roll by Gyro
  gyro_roll = prev_gyro_roll + timeStep * (normGyro.XAxis + normGyro.YAxis*sinRoll*tanPitch + normGyro.ZAxis*cosRoll*tanPitch);
  gyro_pitch = prev_gyro_pitch + timeStep * (normGyro.YAxis*cosRoll-normGyro.ZAxis*sinRoll);
  gyro_yaw = prev_gyro_yaw + timeStep * (normGyro.YAxis*sinRoll/cosPitch + normGyro.ZAxis*cosRoll/cosPitch);

  //A 행렬
  A[0][0]=1; A[0][1]=-timeStep/2*normGyro.XAxis; A[0][2]=-timeStep/2*normGyro.YAxis; A[0][3]=-timeStep/2*normGyro.ZAxis;
  A[1][0]=timeStep/2*normGyro.XAxis; A[0][1]=1; A[0][2]=-timeStep/2*normGyro.ZAxis; A[0][3]=timeStep/2*normGyro.YAxis;
  A[2][0]=timeStep/2*normGyro.YAxis; A[0][1]=timeStep/2*normGyro.ZAxis; A[0][2]=1; A[0][3]=-timeStep/2*normGyro.XAxis;
  A[3][0]=timeStep/2*normGyro.ZAxis; A[0][1]=-timeStep/2*normGyro.YAxis; A[0][2]=timeStep/2*normGyro.XAxis; A[0][3]=1;

  EulerToQuaternion(gyro_roll,gyro_pitch,0);
  // Output
  Serial.print(q[0]);
  Serial.print(' ');
  Serial.print(q[1]);
  Serial.print(' ');
  Serial.print(q[2]);
  Serial.print(' ');
  Serial.print(q[3]);
  /*
  Serial.print(" Roll = ");
  Serial.print(roll);
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  */
  
  Serial.println();

  prev_gyro_roll = gyro_roll;
  prev_gyro_pitch = gyro_pitch;
  prev_gyro_yaw = gyro_yaw;
  
  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}

void EulerToQuaternion(float phi, float theta, float psi)
{
  float sinPhi   = sin(phi/2);   float cosPhi   = cos(phi/2);
  float sinTheta = sin(theta/2); float cosTheta = cos(theta/2);
  float sinPsi   = sin(psi/2);   float cosPsi   = cos(psi/2);

  q[0] = cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi;
  q[1] = sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi;
  q[2] = cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi;
  q[3] = cosPhi*cosTheta*sinPsi + sinPhi*sinTheta*cosPsi;
}

void EulerKalman(float *A, float *z)
{
  
}
