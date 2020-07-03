#define G     9.81

#include <Wire.h>
#include <MPU6050.h>

void EulerToQuaternion(double phi, double theta, double psi);
void EulerKalman();
int InvMatrix(int n, const double* A, double* b);

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

// 시스템 모델 변수
double A[4][4] = {0,};

double H[4][4] = {{1,0,0,0},
                 {0,1,0,0},
                 {0,0,1,0},
                 {0,0,0,1}};

// 잡음 공분산
double Q[4][4] = {{0.0001,0,0,0},
                 {0,0.0001,0,0},
                 {0,0,0.0001,0},
                 {0,0,0,0.0001}};

double R[4][4] = {{10,0,0,0},
                 {0,10,0,0},
                 {0,0,10,0},
                 {0,0,0,10}};

//Kalman 변수
double x[4][1] = {{1},
                 {0},
                 {0},
                 {0}};
                 
double xp[4][1] = {0,};
double z[4][1] = {0,};
double zK[4][1] = {0,};

double P[4][4] = {{1,0,0,0},
                 {0,1,0,0},
                 {0,0,1,0},
                 {0,0,0,1}};
                
double Pp[4][4] = {0,};
double Pp_K[4][4] = {0,};
double K[4][4] = {0,};

double M[4][4] = {0,};
double inv_M[4][4] = {0,};


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
  acc_roll = asin(normAccel.XAxis/G);
  //-(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  //asin(normAccel.XAxis/G)*180.0/M_PI;
  acc_pitch = asin(-normAccel.YAxis/(G*cos(roll)));
  //(atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  //asin(-normAccel.YAxis/(G*cos(roll)))*180.0/M_PI;

  // Calculate Pitch & Roll by Gyro
  gyro_roll = prev_gyro_roll + timeStep * (normGyro.XAxis + normGyro.YAxis*sinRoll*tanPitch + normGyro.ZAxis*cosRoll*tanPitch);
  //gyro_roll + normGyro.XAxis * timeStep;
  //prev_gyro_roll + timeStep * (normGyro.XAxis + normGyro.YAxis*sinRoll*tanPitch + normGyro.ZAxis*cosRoll*tanPitch);
  gyro_pitch = prev_gyro_pitch + timeStep * (normGyro.YAxis*cosRoll-normGyro.ZAxis*sinRoll);
  //gyro_pitch + normGyro.YAxis * timeStep;
  //prev_gyro_pitch + timeStep * (normGyro.YAxis*cosRoll-normGyro.ZAxis*sinRoll);
  gyro_yaw = prev_gyro_yaw + timeStep * (normGyro.YAxis*sinRoll/cosPitch + normGyro.ZAxis*cosRoll/cosPitch);
  //gyro_yaw + normGyro.ZAxis * timeStep;
  //prev_gyro_yaw + timeStep * (normGyro.YAxis*sinRoll/cosPitch + normGyro.ZAxis*cosRoll/cosPitch);

  //A 행렬
  A[0][0]=1; A[0][1]=-timeStep*0.5*normGyro.XAxis; A[0][2]=-timeStep*0.5*normGyro.YAxis; A[0][3]=-timeStep*0.5*normGyro.ZAxis;
  A[1][0]=timeStep*0.5*normGyro.XAxis; A[0][1]=1; A[0][2]=timeStep*0.5*normGyro.ZAxis; A[0][3]=-timeStep*0.5*normGyro.YAxis;
  A[2][0]=timeStep*0.5*normGyro.YAxis; A[0][1]=-timeStep*0.5*normGyro.ZAxis; A[0][2]=1; A[0][3]=timeStep*0.5*normGyro.XAxis;
  A[3][0]=timeStep*0.5*normGyro.ZAxis; A[0][1]=timeStep*0.5*normGyro.YAxis; A[0][2]=-timeStep*0.5*normGyro.XAxis; A[0][3]=1;

  EulerToQuaternion(acc_roll,acc_pitch,0);
  EulerKalman();
  
  // Output
  /*
  Serial.print(q[0]);
  Serial.print(' ');
  Serial.print(q[1]);
  Serial.print(' ');
  Serial.print(q[2]);
  Serial.print(' ');
  Serial.print(q[3]);
  */
    
  //Serial.print("Gyro Roll = ");
  //Serial.print(gyro_roll);
  //Serial.print(" Gyro Pitch = ");
  //Serial.print(gyro_pitch);
 
  //Serial.print(" Acc Roll = ");
  //Serial.print(acc_roll);
  //Serial.print(" Acc Pitch = ");
  //Serial.print(acc_pitch);

  //Serial.print(roll);
  //Serial.print(", ");
  //Serial.print(pitch);

/*  Serial.print(xp[0][0]);
  Serial.print(", ");
  Serial.print(xp[1][0]);
  Serial.print(", ");
  Serial.print(xp[2][0]);
  Serial.print(", ");
  Serial.print(xp[3][0]);*/

  Serial.print(q[0]);
  Serial.print(", ");
  Serial.print(q[1]);
  Serial.print(", ");
  Serial.print(q[2]);
  Serial.print(", ");
  Serial.print(q[3]);

  Serial.println();

  prev_gyro_roll = gyro_roll;
  prev_gyro_pitch = gyro_pitch;
  prev_gyro_yaw = gyro_yaw;

  delay(100);
  
}

void EulerToQuaternion(double phi, double theta, double psi)
{
  double sinPhi   = sin(phi*0.5);   double cosPhi   = cos(phi*0.5);
  double sinTheta = sin(theta*0.5); double cosTheta = cos(theta*0.5);
  double sinPsi   = sin(psi*0.5);   double cosPsi   = cos(psi*0.5);

  q[0] = cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi;
  q[1] = sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi;
  q[2] = cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi;
  q[3] = cosPhi*cosTheta*sinPsi + sinPhi*sinTheta*cosPsi;
}

void EulerKalman()
{
  ///////// xp = A*x; /////////////
/*  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      for(int k=0;k<4;k++)
        xp[i][j] += A[i][k]*x[k][j]; */
        
  xp[0][0] = A[0][0]*x[0][0]+A[0][1]*x[1][0]+A[0][2]*x[2][0]+A[0][3]*x[3][0];
  xp[1][0] = A[1][0]*x[0][0]+A[1][1]*x[1][0]+A[1][2]*x[2][0]+A[1][3]*x[3][0];
  xp[2][0] = A[2][0]*x[0][0]+A[2][1]*x[1][0]+A[2][2]*x[2][0]+A[2][3]*x[3][0];
  xp[3][0] = A[3][0]*x[0][0]+A[3][1]*x[1][0]+A[3][2]*x[2][0]+A[3][3]*x[3][0];

  ///////// Pp = A*P*A' + Q; /////////////
  Pp[0][1] = A[0][0]*(A[0][0]*P[0][0] + A[0][1]*P[1][0] + A[0][2]*P[2][0] + A[0][3]*P[3][0]) + A[0][1]*(A[0][0]*P[0][1] + A[0][1]*P[1][1] + A[0][2]*P[2][1] + A[0][3]*P[3][1]) + A[0][2]*(A[0][0]*P[0][2] + A[0][1]*P[1][2] + A[0][2]*P[2][2] + A[0][3]*P[3][2]) + A[0][3]*(A[0][0]*P[0][3] + A[0][1]*P[1][3] + A[0][2]*P[2][3] + A[0][3]*P[3][3]) + 1/10000;
  Pp[0][2] = A[1][0]*(A[0][0]*P[0][0] + A[0][1]*P[1][0] + A[0][2]*P[2][0] + A[0][3]*P[3][0]) + A[1][1]*(A[0][0]*P[0][1] + A[0][1]*P[1][1] + A[0][2]*P[2][1] + A[0][3]*P[3][1]) + A[1][2]*(A[0][0]*P[0][2] + A[0][1]*P[1][2] + A[0][2]*P[2][2] + A[0][3]*P[3][2]) + A[1][3]*(A[0][0]*P[0][3] + A[0][1]*P[1][3] + A[0][2]*P[2][3] + A[0][3]*P[3][3]);
  Pp[0][3] = A[2][0]*(A[0][0]*P[0][0] + A[0][1]*P[1][0] + A[0][2]*P[2][0] + A[0][3]*P[3][0]) + A[2][1]*(A[0][0]*P[0][1] + A[0][1]*P[1][1] + A[0][2]*P[2][1] + A[0][3]*P[3][1]) + A[2][2]*(A[0][0]*P[0][2] + A[0][1]*P[1][2] + A[0][2]*P[2][2] + A[0][3]*P[3][2]) + A[2][3]*(A[0][0]*P[0][3] + A[0][1]*P[1][3] + A[0][2]*P[2][3] + A[0][3]*P[3][3]);
  Pp[0][4] = A[3][0]*(A[0][0]*P[0][0] + A[0][1]*P[1][0] + A[0][2]*P[2][0] + A[0][3]*P[3][0]) + A[3][1]*(A[0][0]*P[0][1] + A[0][1]*P[1][1] + A[0][2]*P[2][1] + A[0][3]*P[3][1]) + A[3][2]*(A[0][0]*P[0][2] + A[0][1]*P[1][2] + A[0][2]*P[2][2] + A[0][3]*P[3][2]) + A[3][3]*(A[0][0]*P[0][3] + A[0][1]*P[1][3] + A[0][2]*P[2][3] + A[0][3]*P[3][3]);
  
  Pp[1][0] = A[0][0]*(A[1][0]*P[0][0] + A[1][1]*P[1][0] + A[1][2]*P[2][0] + A[1][3]*P[3][0]) + A[0][1]*(A[1][0]*P[0][1] + A[1][1]*P[1][1] + A[1][2]*P[2][1] + A[1][3]*P[3][1]) + A[0][2]*(A[1][0]*P[0][2] + A[1][1]*P[1][2] + A[1][2]*P[2][2] + A[1][3]*P[3][2]) + A[0][3]*(A[1][0]*P[0][3] + A[1][1]*P[1][3] + A[1][2]*P[2][3] + A[1][3]*P[3][3]) ;
  Pp[1][1] = A[1][0]*(A[1][0]*P[0][0] + A[1][1]*P[1][0] + A[1][2]*P[2][0] + A[1][3]*P[3][0]) + A[1][1]*(A[1][0]*P[0][1] + A[1][1]*P[1][1] + A[1][2]*P[2][1] + A[1][3]*P[3][1]) + A[1][2]*(A[1][0]*P[0][2] + A[1][1]*P[1][2] + A[1][2]*P[2][2] + A[1][3]*P[3][2]) + A[1][3]*(A[1][0]*P[0][3] + A[1][1]*P[1][3] + A[1][2]*P[2][3] + A[1][3]*P[3][3]) + 1/10000;
  Pp[1][2] = A[2][0]*(A[1][0]*P[0][0] + A[1][1]*P[1][0] + A[1][2]*P[2][0] + A[1][3]*P[3][0]) + A[2][1]*(A[1][0]*P[0][1] + A[1][1]*P[1][1] + A[1][2]*P[2][1] + A[1][3]*P[3][1]) + A[2][2]*(A[1][0]*P[0][2] + A[1][1]*P[1][2] + A[1][2]*P[2][2] + A[1][3]*P[3][2]) + A[2][3]*(A[1][0]*P[0][3] + A[1][1]*P[1][3] + A[1][2]*P[2][3] + A[1][3]*P[3][3]);
  Pp[1][3] = A[3][0]*(A[1][0]*P[0][0] + A[1][1]*P[1][0] + A[1][2]*P[2][0] + A[1][3]*P[3][0]) + A[3][1]*(A[1][0]*P[0][1] + A[1][1]*P[1][1] + A[1][2]*P[2][1] + A[1][3]*P[3][1]) + A[3][2]*(A[1][0]*P[0][2] + A[1][1]*P[1][2] + A[1][2]*P[2][2] + A[1][3]*P[3][2]) + A[3][3]*(A[1][0]*P[0][3] + A[1][1]*P[1][3] + A[1][2]*P[2][3] + A[1][3]*P[3][3]);
  
  Pp[2][0] = A[0][0]*(A[2][0]*P[0][0] + A[2][1]*P[1][0] + A[2][2]*P[2][0] + A[2][3]*P[3][0]) + A[0][1]*(A[2][0]*P[0][1] + A[2][1]*P[1][1] + A[2][2]*P[2][1] + A[2][3]*P[3][1]) + A[0][2]*(A[2][0]*P[0][2] + A[2][1]*P[1][2] + A[2][2]*P[2][2] + A[2][3]*P[3][2]) + A[0][3]*(A[2][0]*P[0][3] + A[2][1]*P[1][3] + A[2][2]*P[2][3] + A[2][3]*P[3][3]);
  Pp[2][1] = A[1][0]*(A[2][0]*P[0][0] + A[2][1]*P[1][0] + A[2][2]*P[2][0] + A[2][3]*P[3][0]) + A[1][1]*(A[2][0]*P[0][1] + A[2][1]*P[1][1] + A[2][2]*P[2][1] + A[2][3]*P[3][1]) + A[1][2]*(A[2][0]*P[0][2] + A[2][1]*P[1][2] + A[2][2]*P[2][2] + A[2][3]*P[3][2]) + A[1][3]*(A[2][0]*P[0][3] + A[2][1]*P[1][3] + A[2][2]*P[2][3] + A[2][3]*P[3][3]);
  Pp[2][2] = A[2][0]*(A[2][0]*P[0][0] + A[2][1]*P[1][0] + A[2][2]*P[2][0] + A[2][3]*P[3][0]) + A[2][1]*(A[2][0]*P[0][1] + A[2][1]*P[1][1] + A[2][2]*P[2][1] + A[2][3]*P[3][1]) + A[2][2]*(A[2][0]*P[0][2] + A[2][1]*P[1][2] + A[2][2]*P[2][2] + A[2][3]*P[3][2]) + A[2][3]*(A[2][0]*P[0][3] + A[2][1]*P[1][3] + A[2][2]*P[2][3] + A[2][3]*P[3][3]) + 1/10000;
  Pp[2][3] = A[3][0]*(A[2][0]*P[0][0] + A[2][1]*P[1][0] + A[2][2]*P[2][0] + A[2][3]*P[3][0]) + A[3][1]*(A[2][0]*P[0][1] + A[2][1]*P[1][1] + A[2][2]*P[2][1] + A[2][3]*P[3][1]) + A[3][2]*(A[2][0]*P[0][2] + A[2][1]*P[1][2] + A[2][2]*P[2][2] + A[2][3]*P[3][2]) + A[3][3]*(A[2][0]*P[0][3] + A[2][1]*P[1][3] + A[2][2]*P[2][3] + A[2][3]*P[3][3]);
  
  Pp[3][0] = A[0][0]*(A[3][0]*P[0][0] + A[3][1]*P[1][0] + A[3][2]*P[2][0] + A[3][3]*P[3][0]) + A[0][1]*(A[3][0]*P[0][1] + A[3][1]*P[1][1] + A[3][2]*P[2][1] + A[3][3]*P[3][1]) + A[0][2]*(A[3][0]*P[0][2] + A[3][1]*P[1][2] + A[3][2]*P[2][2] + A[3][3]*P[3][2]) + A[0][3]*(A[3][0]*P[0][3] + A[3][1]*P[1][3] + A[3][2]*P[2][3] + A[3][3]*P[3][3]) ;
  Pp[3][1] = A[1][0]*(A[3][0]*P[0][0] + A[3][1]*P[1][0] + A[3][2]*P[2][0] + A[3][3]*P[3][0]) + A[1][1]*(A[3][0]*P[0][1] + A[3][1]*P[1][1] + A[3][2]*P[2][1] + A[3][3]*P[3][1]) + A[1][2]*(A[3][0]*P[0][2] + A[3][1]*P[1][2] + A[3][2]*P[2][2] + A[3][3]*P[3][2]) + A[1][3]*(A[3][0]*P[0][3] + A[3][1]*P[1][3] + A[3][2]*P[2][3] + A[3][3]*P[3][3]);
  Pp[3][2] = A[2][0]*(A[3][0]*P[0][0] + A[3][1]*P[1][0] + A[3][2]*P[2][0] + A[3][3]*P[3][0]) + A[2][1]*(A[3][0]*P[0][1] + A[3][1]*P[1][1] + A[3][2]*P[2][1] + A[3][3]*P[3][1]) + A[2][2]*(A[3][0]*P[0][2] + A[3][1]*P[1][2] + A[3][2]*P[2][2] + A[3][3]*P[3][2]) + A[2][3]*(A[3][0]*P[0][3] + A[3][1]*P[1][3] + A[3][2]*P[2][3] + A[3][3]*P[3][3]);
  Pp[3][3] = A[3][0]*(A[3][0]*P[0][0] + A[3][1]*P[1][0] + A[3][2]*P[2][0] + A[3][3]*P[3][0]) + A[3][1]*(A[3][0]*P[0][1] + A[3][1]*P[1][1] + A[3][2]*P[2][1] + A[3][3]*P[3][1]) + A[3][2]*(A[3][0]*P[0][2] + A[3][1]*P[1][2] + A[3][2]*P[2][2] + A[3][3]*P[3][2]) + A[3][3]*(A[3][0]*P[0][3] + A[3][1]*P[1][3] + A[3][2]*P[2][3] + A[3][3]*P[3][3]) + 1/10000;


  ///////// K = Pp*H'*(H*Pp*H'+R)^-1; /////////////
    // 행렬 덧셈
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      M[i][j] = Pp[i][j] + R[i][j];
      
    //역행렬
  InvMatrix(4,(double*)M,(double*)inv_M);

    // 행렬 곱셈  
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      for(int k=0;k<4;k++)
        K[i][j] += Pp[i][k]*inv_M[k][j];


  ///////// x = xp + K*(z - H*xp); /////////////
  for(int i=0;i<4;i++)
     z[i][0] = q[i] - xp[i][0];
    
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
        zK[i][0] += K[i][j]*z[j][0];  
 
  for(int i=0;i<4;i++)
      x[i][0] = xp[i][0] + zK[i][0];

  ///////// P = Pp - K*H*Pp; /////////////
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      for(int k=0;k<4;k++)
        Pp_K[i][j] += K[i][k]*Pp[k][j];

  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      P[i][j] = Pp[i][j] - Pp_K[i][j];


  roll = atan2(2*(x[3][0]*x[4][0] + x[1][0]*x[2][0]) , 1-2*(x[2][0]*x[2][0] + x[3][0]*x[3][0]))*180.0/M_PI;
  pitch = -asin(2*(x[2][0]*x[4][0] - x[1][0]*x[3][0]))*180.0/M_PI;
}

int InvMatrix(int n, const double* A, double* b)  // 역행렬 구하는 함수
{
    double m;
    register int i, j, k;
    double* a = new double[n*n];

    if(a==NULL) 
  return 0;
    for(i=0; i<n*n; i++) 
  a[i]=A[i];
 for(i=0; i<n; i++) 
 {
  for(j=0; j<n; j++)
  {
            b[i*n+j]=(i==j)?1.:0.;
        }
    }
    for(i=0; i<n; i++)
 {
  if(a[i*n+i]==0.) 
  {
   if(i==n-1) 
   {
    delete[] a;
    return 0;
            }
            for(k=1; i+k<n; k++)
   {
                if(a[i*n+i+k] != 0.) 
     break;
            }
            if(i+k>=n)
   {
                delete[] a;
    return 0;
            }
            for(j=0; j<n; j++) 
   {
                m = a[i*n+j];
                a[i*n+j] = a[(i+k)*n+j];
                a[(i+k)*n+j] = m;
                m = b[i*n+j];
                b[i*n+j] = b[(i+k)*n+j];
                b[(i+k)*n+j] = m;
            }
        }
        m = a[i*n+i];
        for(j=0; j<n; j++) 
  {
            a[i*n+j]/=m;
            b[i*n+j]/=m;
        }
        for(j=0; j<n; j++) 
  {
            if(i==j) 
    continue;

            m = a[j*n+i];
            for(k=0; k<n; k++)   
   {
                a[j*n+k] -= a[i*n+k]*m;
                b[j*n+k] -= b[i*n+k]*m;
            }
        }
    }
    delete[] a;
    return 1;
}
