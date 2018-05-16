
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
//#include "I2Cdev.h"
#include "MPU6050.h"
#include "IMU.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
//#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

//int16_t ax, ay, az;
float Acc[3];
//int16_t gx, gy, gz;
float Gyro[3];
IMU_Handler hIMU;
//float euler;
int countmotion=0;
float stepout[3] = { 0 ,0 ,0};
int stepall=0;
float Posout[3] = {0 ,0 ,0};


// Pitch, Roll and Yaw values
float yawraw = 0;
float yaw = 0;

// Timers
//unsigned long timer = 0;
float timeStep = 0.002;

//int now;
//int last;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
//#define OUTPUT_READABLE_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
//    // join I2C bus (I2Cdev library doesn't do this automatically)
//    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//        Wire.begin();
//    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//        Fastwire::setup(400, true);
//    #endif
//
//    // initialize serial communication
//    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
//    // it's really up to you depending on your project)
//    Serial.begin(115200);
//
//    // initialize device
//    Serial.println("Initializing I2C devices...");
//    accelgyro.initialize();
//
//    // verify connection
//    Serial.println("Testing device connections...");
//    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//    accelgyro.setXAccelOffset(-3792);
//    accelgyro.setYAccelOffset(-726);
//    accelgyro.setZAccelOffset(1067);
//    accelgyro.setXGyroOffset(-167);
//    accelgyro.setXGyroOffset(103);
//    accelgyro.setXGyroOffset(-86);
    Serial.begin(115200);
    
    Serial.println("Initialize MPU6050");
    
    while(!accelgyro.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      delay(500);
    }

    accelgyro.calibrateGyro();
    accelgyro.setThreshold(3);
    // Inint IMU
    ResetInitialCalibration(&hIMU);
    resetIMU(&hIMU, 0.024 );
//    last=0;
//    // configure Arduino LED for
//    pinMode(LED_PIN, OUTPUT);
}

void loop() {
      Vector Accel = accelgyro.readRawAccel();
      Vector Gyros = accelgyro.readNormalizeGyro();
//      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//      setAccGyro(Acc,Gyro,ax,ay,az,gx,gy,gz);
      Acc[0] = Accel.XAxis/16384.0f;
      Acc[1] = Accel.YAxis/16384.0f;
      Acc[2] = Accel.ZAxis/16384.0f;
      Gyro[0] = Gyros.XAxis;
      Gyro[1] = Gyros.YAxis;
      Gyro[2] = Gyros.ZAxis;   
        
      // Calculate Pitch, Roll and Yaw
      yawraw = yawraw + Gyro[2] * timeStep;
      yaw = yawraw*2.25;
      if((yaw >= 360) || ( yaw <= -360))
         yaw = 0;
//      yaw = toEulerAngle(hIMU.q)*180/PI;
//      Serial.print("yaw= ");
//      Serial.println(yaw);
      
         
      DoInitialAverageValueCalculate(&hIMU, Gyro, Acc);
      SetThreshold(&hIMU);
      RotateIMUCoordinateByAccAndGyr(&hIMU, Gyro, Acc);
      IMU_new_data(&hIMU, Gyro, Acc);
      IMU_Process(&hIMU);
}

void setAccGyro(float * Accel, float * Gyroscpoe,int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  Accel[0] = (ax/16384.0f); Accel[1] = (ay/16384.0f); Accel[2] = (az/16384.0f);
  Gyroscpoe[0] = gx/131.0f; Gyroscpoe[1] = gy/131.0f; Gyroscpoe[2] = gz/131.0f;
}

void quaternProd(float* ab, float* a, float* b)
{
  ab[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  ab[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  ab[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  ab[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void quaternConj(float* a)
{
  a[1] = -a[1];
  a[2] = -a[2];
  a[3] = -a[3];
}

void SetThreshold(IMU_Handler* hIMU)
{
  hIMU->ChannelThresHold_A[0] = hIMU->InitCaliAverage.Ax - 0.2;
  hIMU->ChannelThresHold_A[1] = hIMU->InitCaliAverage.Ax + 0.2;
  hIMU->ChannelThresHold_A[2] = hIMU->InitCaliAverage.Ay - 0.2;
  hIMU->ChannelThresHold_A[3] = hIMU->InitCaliAverage.Ay + 0.2;
  hIMU->ChannelThresHold_A[4] = hIMU->InitCaliAverage.Az - 0.2;
  hIMU->ChannelThresHold_A[5] = hIMU->InitCaliAverage.Az + 0.2;


  hIMU->ChannelThresHold_G[0] = -20;
  hIMU->ChannelThresHold_G[1] = 20;
  hIMU->ChannelThresHold_G[2] = -20;
  hIMU->ChannelThresHold_G[3] = 20;
  hIMU->ChannelThresHold_G[4] = -20;
  hIMU->ChannelThresHold_G[5] = 20;
  
  hIMU->Z_threadhold = 0.05;
}

void ResetInitialCalibration(IMU_Handler* hIMU)
{
  hIMU->InitCaliAverage.PointCount = 0;
}

void RotateIMUCoordinateByAccAndGyr(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
  int i;
  float Gyro[3], Accelero[3];
  hIMU->Kp = 50;

  Gyro[0] = Gyroscope[0]; Gyro[1] = Gyroscope[1]; Gyro[2] = Gyroscope[2];

  for (i = 0; i < 1; i++)
  {
    Accelero[0] = Accelerometer[0]; Accelero[1] = Accelerometer[1]; Accelero[2] = Accelerometer[2];
    updateIMU(hIMU, Gyro, Accelero);
  }

}

void DoInitialAverageValueCalculate(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
  hIMU->InitCaliAverage.Ax = hIMU->InitCaliAverage.Ax * hIMU->InitCaliAverage.PointCount + Accelerometer[0];
  hIMU->InitCaliAverage.Ay = hIMU->InitCaliAverage.Ay * hIMU->InitCaliAverage.PointCount + Accelerometer[1];
  hIMU->InitCaliAverage.Az = hIMU->InitCaliAverage.Az * hIMU->InitCaliAverage.PointCount + Accelerometer[2];

  hIMU->InitCaliAverage.Gx = hIMU->InitCaliAverage.Gx * hIMU->InitCaliAverage.PointCount + Gyroscope[0];
  hIMU->InitCaliAverage.Gy = hIMU->InitCaliAverage.Gy * hIMU->InitCaliAverage.PointCount + Gyroscope[1];
  hIMU->InitCaliAverage.Gz = hIMU->InitCaliAverage.Gz * hIMU->InitCaliAverage.PointCount + Gyroscope[2];

  hIMU->InitCaliAverage.PointCount++;

  hIMU->InitCaliAverage.Ax /= hIMU->InitCaliAverage.PointCount;
  hIMU->InitCaliAverage.Ay /= hIMU->InitCaliAverage.PointCount;
  hIMU->InitCaliAverage.Az /= hIMU->InitCaliAverage.PointCount;
  hIMU->InitCaliAverage.Gx /= hIMU->InitCaliAverage.PointCount;
  hIMU->InitCaliAverage.Gy /= hIMU->InitCaliAverage.PointCount;
  hIMU->InitCaliAverage.Gz /= hIMU->InitCaliAverage.PointCount;
}

void resetIMU(IMU_Handler* hIMU, float SamplePeriod)
{
  int i;

  hIMU->SamplePeriod = SamplePeriod;

  hIMU->q[0] = 1; hIMU->q[1] = 0; hIMU->q[2] = 0; hIMU->q[3] = 0;

  hIMU->Int_Error[0] = 0; hIMU->Int_Error[1] = 0; hIMU->Int_Error[2] = 0;
  hIMU->PosX = 0; hIMU->PosY = 0; hIMU->PosZ = 0;

  for (i = 0; i < PREPROCESS_BUF_SIZE; i++)
  {
    hIMU->PreprocessBuf[i].Az = Accelerometer_Scale;
  }
  hIMU->LPF_X1 = 0;
}

void IMU_new_data(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
  int t;
  t = (hIMU->Raw_Buffer_Head + 1) % IMU_RAW_DATA_BUFFER_LENGTH;
  if (t == hIMU->Raw_Buffer_Tail)
  {
    Serial.println("!!!!!!IMU_RAW_BUFFER_FULL\n");
    return;
  }
  hIMU->Raw_Buffer[t * 6 + 0] = Gyroscope[0];
  hIMU->Raw_Buffer[t * 6 + 1] = Gyroscope[1];
  hIMU->Raw_Buffer[t * 6 + 2] = Gyroscope[2];
  hIMU->Raw_Buffer[t * 6 + 3] = Accelerometer[0];
  hIMU->Raw_Buffer[t * 6 + 4] = Accelerometer[1];
  hIMU->Raw_Buffer[t * 6 + 5] = Accelerometer[2];
  hIMU->Raw_Buffer_Head = t;

}

void IMU_Process(IMU_Handler* hIMU)
{  
  static int i;
  static char tmp_ChannelCounter;
  static float Filter_tmp1;

  static float Gyro[3], Accelero[3];

  static float MY_q[4], tmp_vector1[4], tmp_vector2[4];

  static int StationaryChannelCounterForThreshold;

  static short TMP1;

  static float Velocity_Error_K_x, Velocity_Error_K_y, Velocity_Error_K_z;

    if (hIMU->Raw_Buffer_Head == hIMU->Raw_Buffer_Tail)
    { 
      return;
    }
    
    hIMU->PreprocessBuf_Pointer = (hIMU->PreprocessBuf_Pointer + 1) % PREPROCESS_BUF_SIZE;

    i = (hIMU->Raw_Buffer_Tail + 1) % IMU_RAW_DATA_BUFFER_LENGTH;


    hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx = hIMU->Raw_Buffer[i * 6 + 0] - hIMU->InitCaliAverage.Gx;
    hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy = hIMU->Raw_Buffer[i * 6 + 1] - hIMU->InitCaliAverage.Gy;
    hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz = hIMU->Raw_Buffer[i * 6 + 2] - hIMU->InitCaliAverage.Gz;
    hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax = hIMU->Raw_Buffer[i * 6 + 3];
    hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay = hIMU->Raw_Buffer[i * 6 + 4];
    hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az = hIMU->Raw_Buffer[i * 6 + 5];
    hIMU->Raw_Buffer_Tail = i;
//    Serial.println();
//    Serial.print("Preprocess G/A: ");
//    Serial.print(hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx); Serial.print("   ");
//    Serial.print(hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy); Serial.print("   ");
//    Serial.print(hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz); Serial.print("   ");
//    Serial.print(hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax); Serial.print("   ");
//    Serial.print(hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay); Serial.print("   ");
//    Serial.println(hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az);

  #define PreprocessBuf_Offset(n) (hIMU->PreprocessBuf[(hIMU->PreprocessBuf_Pointer + PREPROCESS_BUF_SIZE + (n)) % PREPROCESS_BUF_SIZE ])


    tmp_ChannelCounter = 0;
    if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx > hIMU->ChannelThresHold_G[0]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx < hIMU->ChannelThresHold_G[1])) tmp_ChannelCounter++;
    if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy > hIMU->ChannelThresHold_G[2]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy < hIMU->ChannelThresHold_G[3])) tmp_ChannelCounter++;
    if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz > hIMU->ChannelThresHold_G[4]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz < hIMU->ChannelThresHold_G[5])) tmp_ChannelCounter++;
    if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax > hIMU->ChannelThresHold_A[0]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax < hIMU->ChannelThresHold_A[1])) tmp_ChannelCounter++;
    if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay > hIMU->ChannelThresHold_A[2]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay < hIMU->ChannelThresHold_A[3])) tmp_ChannelCounter++;
    if ((hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az > hIMU->ChannelThresHold_A[4]) && (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az < hIMU->ChannelThresHold_A[5])) tmp_ChannelCounter++;


    //First-order IIR filter
    //a(1)*y(n) = b(1)*x(n) + b(2)*x(n - 1) + ... + b(nb + 1)*x(n - nb)
    //           -a(2)*y(n - 1) - ... - a(na + 1)*y(n - na)
    //
    //y(n) = b(1)*x(n) + b(2)*x(n-1) - a(2)*y(n-1)
    //     



    //Low-pass filtering
    //HPF_Y1 is a high-pass filtered result, here as x(n)
    hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].StationaryChannelCounter = tmp_ChannelCounter * 0.0245 + hIMU->LPF_X1 * (0.0245) - PreprocessBuf_Offset(-1).StationaryChannelCounter *(-0.9510);
    hIMU->LPF_X1 = tmp_ChannelCounter;




    //The elements of the end of the loop queue are used to update the elements of the IMU

    Gyro[0] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gx / 180 * Pi;
    Gyro[1] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gy / 180 * Pi;
    Gyro[2] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Gz / 180 * Pi;
    Accelero[0] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ax * G;
    Accelero[1] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Ay * G;
    Accelero[2] = hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].Az * G;

    //Access to PREPROCESS_STATE_CHANGE_CONDITION_LENGTH elements after the element currently 
    //used to update the IMU via PreprocessBuf_Offset
    //in case
    if (hIMU->PreprocessBuf[hIMU->PreprocessBuf_Pointer].StationaryChannelCounter >= 5)
    {
      hIMU->In_Static_State = 1;
    }
    else
    {
      hIMU->In_Static_State = 0;
    }

//    Serial.print("In_Static_State: ");
//    Serial.println(hIMU->In_Static_State == 1 ? 1 : -1);

    if (hIMU->In_Static_State)
    {
      hIMU->Kp = 3.0f;

    }
    else
    {
      hIMU->Kp = 0.0f;
    }


    //Since updateIMU will modify the value of Accelero, copy it here in advance
    tmp_vector1[0] = 0;
    tmp_vector1[1] = Accelero[0];
    tmp_vector1[2] = Accelero[1];
    tmp_vector1[3] = Accelero[2];

    updateIMU(hIMU, Gyro, Accelero);
//    Serial.print("Gyro: ");
//    Serial.print(Gyro[0]); Serial.print("   ");
//    Serial.print(Gyro[1]); Serial.print("   ");
//    Serial.println(Gyro[2]);
//    Serial.print("tmp_vector: ");
//    Serial.print(tmp_vector1[1]); Serial.print("   ");
//    Serial.print(tmp_vector1[2]); Serial.print("   ");
//    Serial.println(tmp_vector1[3]);Serial.print("   ");
//    Serial.print("q: ");
//    Serial.print(hIMU->q[0]); Serial.print("   ");
//    Serial.print(hIMU->q[1]); Serial.print("   ");
//    Serial.print(hIMU->q[2]); Serial.print("   ");
//    Serial.println(hIMU->q[3]);
    if (hIMU->In_Static_State)
    {
      if( countmotion > 130){ 
        if((yaw) >= 0 && yaw <=90) {
          stepout[0] += cos(yaw*PI/180);
          stepout[1] -= sin(yaw*PI/180); 
        } 
        else if (yaw <= 0 && yaw >= -90) {
          stepout[0] += cos(yaw*PI/180);
          stepout[1] -= sin(yaw*PI/180); 
        }
        else if(yaw > 90 && yaw <= 180) {
          stepout[0] -= cos((180-yaw)*PI/180);
          stepout[1] -= sin((180-yaw)*PI/180); 
        }
        else if(yaw < -90 && yaw  >= -180) {
          stepout[0] -= cos((-180-yaw)*PI/180);
          stepout[1] -= sin((-180-yaw)*PI/180); 
        }
        Serial.print("StepX= ");
        Serial.print(stepout[0]);
        Serial.print("  StepY= ");
        Serial.println(stepout[1]);
        Serial.print("yaw= ");
        Serial.println(yaw);
        countmotion = 0;  
      }
//      countmotion = 0;
      //If VelocityBuf_Pointer is not 0, the last time it just went from motion to quiescence
      if (hIMU->VelocityBuf_Pointer != 0)
      {
        Velocity_Error_K_x = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vx / (hIMU->VelocityBuf_Pointer);
        Velocity_Error_K_y = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vy / (hIMU->VelocityBuf_Pointer);
        Velocity_Error_K_z = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vz / (hIMU->VelocityBuf_Pointer);

        hIMU->Z_old_val = hIMU->PosZ;
        for (i = 0; i<hIMU->VelocityBuf_Pointer; i++)
        {                   
          hIMU->PosX += ((hIMU->VelocityBuf[i].Vx - Velocity_Error_K_x * i) * hIMU->SamplePeriod);
          hIMU->PosY += ((hIMU->VelocityBuf[i].Vy - Velocity_Error_K_y * i) * hIMU->SamplePeriod);
          hIMU->PosZ += ((hIMU->VelocityBuf[i].Vz - Velocity_Error_K_z * i) * hIMU->SamplePeriod);              
        }
        
        if (   fabs(hIMU->Z_old_val - hIMU->PosZ ) < hIMU->Z_threadhold   )
        {
          hIMU->PosZ = hIMU->Z_old_val;
        }

        //Clearing means that the previous step 
        //has been processed and there is no need to perform the next processing
        //, and at the same time prepare for storing the next data
        hIMU->VelocityBuf_Pointer = 0;

        // Calculator toa do x y
        
        Serial.println();
        Serial.print("Pos: ");
        Serial.print(hIMU->PosX); Serial.print("\t");
        Serial.print(hIMU->PosY); Serial.print("\t");
        Serial.print(hIMU->PosZ); Serial.println("\t");
//        hIMU->PosX=hIMU->PosY=hIMU->PosZ=0;
      }
      else
      {
//        if( hIMU->PosX != 0 && hIMU->PosY !=0 && hIMU->PosZ !=0 ) {
//          Serial.println();
//          Serial.print("Pos: ");
//          Serial.print(hIMU->PosX); Serial.print("\t");
//          Serial.print(hIMU->PosY); Serial.print("\t");
//          Serial.print(hIMU->PosZ); Serial.println("\t");
//        }
//        hIMU->PosX=hIMU->PosY=hIMU->PosZ=0;
      }
    }
    else  //In motion
    {
      MY_q[0] = hIMU->q[0];
      MY_q[1] = hIMU->q[1];
      MY_q[2] = hIMU->q[2];
      MY_q[3] = hIMU->q[3];

      quaternProd(tmp_vector2, MY_q, tmp_vector1);
      quaternConj(MY_q);
      quaternProd(tmp_vector1, tmp_vector2, MY_q);

      //Loss of gravitational acceleration in the direction of the Z axis
      tmp_vector1[3] = tmp_vector1[3] - G;
//      Serial.print("tmp_vector1 in motion: ");
//      Serial.print(tmp_vector1[1]);
//      Serial.print("    ");
//      Serial.print(tmp_vector1[1]);
//      Serial.print("    ");
//      Serial.println(tmp_vector1[1]);
//      Serial.print("SamplePeriod: ");
//      Serial.println(hIMU->SamplePeriod);
      //Integral acceleration gets speed
      if (hIMU->VelocityBuf_Pointer != 0)
      {
        hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vx = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vx + tmp_vector1[1] * hIMU->SamplePeriod;
        hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vy = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vy + tmp_vector1[2] * hIMU->SamplePeriod;
        hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vz = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vz + tmp_vector1[3] * hIMU->SamplePeriod;
      }
      else
      {
        hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vx = tmp_vector1[1] * hIMU->SamplePeriod;
        hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vy = tmp_vector1[2] * hIMU->SamplePeriod;
        hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vz = tmp_vector1[3] * hIMU->SamplePeriod;
      }


      if (hIMU->VelocityBuf_Pointer < VELOCITY_BUF_SIZE - 1)
      {
        hIMU->VelocityBuf_Pointer++;
        countmotion++;
        if( countmotion > 130){
          if(yaw >= 0 && yaw <=90) {
            stepout[0] += cos(yaw*PI/180);
            stepout[1] -= sin(yaw*PI/180); 
          } 
          else if (yaw <= 0 && yaw >= -90) {
            stepout[0] += cos(yaw*PI/180);
            stepout[1] -= sin(yaw*PI/180); 
          }
          else if(yaw > 90 && yaw <= 180) {
            stepout[0] -= cos((180-yaw)*PI/180);
            stepout[1] -= sin((180-yaw)*PI/180); 
          }
          else if(yaw < -90 && yaw  >= -180) {
            stepout[0] -= cos((-180-yaw)*PI/180);
            stepout[1] -= sin((-180-yaw)*PI/180); 
          }
          Serial.print("StepX= ");
          Serial.print(stepout[0]);
          Serial.print("  StepY= ");
          Serial.println(stepout[1]);
          Serial.print("yaw= ");
          Serial.println(yaw);
          countmotion = 0;  
      }
//        Serial.print("VelocityBuf_Pointer: ");
//        Serial.println(hIMU->VelocityBuf_Pointer);
//        Serial.print("Velocity: ");
//        Serial.print(hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vx); 
//        Serial.print("    ");
//        Serial.print(hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vy); 
//        Serial.print("    ");
//        Serial.println(hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer].Vz);
      }
      else
      {
        //printf("overflow\n");
        Velocity_Error_K_x = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vx / (hIMU->VelocityBuf_Pointer);
        Velocity_Error_K_y = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vy / (hIMU->VelocityBuf_Pointer);
        Velocity_Error_K_z = hIMU->VelocityBuf[hIMU->VelocityBuf_Pointer - 1].Vz / (hIMU->VelocityBuf_Pointer);

        hIMU->Z_old_val = hIMU->PosZ;
        for (i = 0; i<hIMU->VelocityBuf_Pointer; i++)
        {
                    
          hIMU->PosX += ((hIMU->VelocityBuf[i].Vx - Velocity_Error_K_x * i) * hIMU->SamplePeriod);
          hIMU->PosY += ((hIMU->VelocityBuf[i].Vy - Velocity_Error_K_y * i) * hIMU->SamplePeriod);
          hIMU->PosZ += ((hIMU->VelocityBuf[i].Vz - Velocity_Error_K_z * i) * hIMU->SamplePeriod);
        }
        
        if (   fabs(hIMU->Z_old_val - hIMU->PosZ ) < hIMU->Z_threadhold   )
        {
          hIMU->PosZ = hIMU->Z_old_val;
        }
//        Serial.println();
//        Serial.print("Postmp: ");
//        Serial.print(hIMU->PosX); Serial.print("\t");
//        Serial.print(hIMU->PosY); Serial.print("\t");
//        Serial.print(hIMU->PosZ); Serial.println("\t");
        hIMU->VelocityBuf_Pointer = 0;
        
      }

    }
    

#undef PreprocessBuf_Offset
  
  
}

void updateIMU(IMU_Handler* hIMU, float* Gyroscope, float* Accelerometer)
{
  float norm = 1;
  float v[3];
  float error[3];
  float Ref[4];
  float pDot[4];

  norm = sqrt(Accelerometer[0] * Accelerometer[0] + Accelerometer[1] * Accelerometer[1] + Accelerometer[2] * Accelerometer[2]);

  Accelerometer[0] /= norm;
  Accelerometer[1] /= norm;
  Accelerometer[2] /= norm;

  v[0] = 2 * (hIMU->q[1] * hIMU->q[3] - hIMU->q[0] * hIMU->q[2]);
  v[1] = 2 * (hIMU->q[0] * hIMU->q[1] + hIMU->q[2] * hIMU->q[3]);
  v[2] = 2 * (hIMU->q[0] * hIMU->q[0] - hIMU->q[1] * hIMU->q[1] - hIMU->q[2] * hIMU->q[2] + hIMU->q[3] * hIMU->q[3]);

  //cross product
  error[0] = Accelerometer[2] * v[1] - Accelerometer[1] * v[2];
  error[1] = Accelerometer[0] * v[2] - Accelerometer[2] * v[0];
  error[2] = Accelerometer[1] * v[0] - Accelerometer[0] * v[1];

  hIMU->Int_Error[0] += error[0];
  hIMU->Int_Error[1] += error[1];
  hIMU->Int_Error[2] += error[2];


  //Apply feedback terms
  Ref[1] = Gyroscope[0] - (hIMU->Kp * error[0] + hIMU->Ki * hIMU->Int_Error[0]);
  Ref[2] = Gyroscope[1] - (hIMU->Kp * error[1] + hIMU->Ki * hIMU->Int_Error[1]);
  Ref[3] = Gyroscope[2] - (hIMU->Kp * error[2] + hIMU->Ki * hIMU->Int_Error[2]);

  Ref[0] = 0;
  quaternProd(pDot, hIMU->q, Ref);
  pDot[0] *= 0.5;
  pDot[1] *= 0.5;
  pDot[2] *= 0.5;
  pDot[3] *= 0.5;

  hIMU->q[0] += pDot[0] * hIMU->SamplePeriod;
  hIMU->q[1] += pDot[1] * hIMU->SamplePeriod;
  hIMU->q[2] += pDot[2] * hIMU->SamplePeriod;
  hIMU->q[3] += pDot[3] * hIMU->SamplePeriod;

  norm = sqrt(hIMU->q[0] * hIMU->q[0] + hIMU->q[1] * hIMU->q[1] + hIMU->q[2] * hIMU->q[2] + hIMU->q[3] * hIMU->q[3]);
  hIMU->q[0] /= norm;
  hIMU->q[1] /= norm;
  hIMU->q[2] /= norm;
  hIMU->q[3] /= norm;

}

float toEulerAngle(float q[4])
{
  
  // yaw (z-axis rotation)
  double siny = +2.0 * (q[0] * q[3] + q[1] * q[2]);
  double cosy = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);  
  return atan2(siny, cosy);
}

